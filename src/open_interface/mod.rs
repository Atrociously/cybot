use core::ops::Deref;

use cortex_m::interrupt::CriticalSection;

use crate::{bits::*, SpinTimer, get_cybot};

pub mod charging;
mod mode;
mod stasis;

pub use mode::OiMode;
pub use stasis::Stasis;

use crate::measure::{Angle, Distance};

const SENSOR_PACKET_SIZE: usize = 80;

const OPCODE_START: u8 = 128;
const OPCODE_FULL: u8 = 132;
const OPCODE_LEDS: u8 = 139;
const OPCODE_DRIVE_WHEELS: u8 = 145;
const OPCODE_SENSORS: u8 = 142;
const OPCODE_STOP: u8 = 173;
const SENSOR_PACKET_GROUP100: u8 = 100;

pub struct OpenInterface {
    // pointer to state on the heap // not on heap anymore dependencies suck
    state: State,
}

#[derive(Debug, Default)]
pub struct State {
    pub wheel_drop_left: bool,
    pub wheel_drop_right: bool,
    pub wheel_overcurrent_left: bool,
    pub wheel_overcurrent_right: bool,

    pub bump_left: bool,
    pub bump_right: bool,

    pub cliff_left: bool,
    pub cliff_front_left: bool,
    pub cliff_right: bool,
    pub cliff_front_right: bool,

    pub wall_sensor: bool,
    pub wall_virtual: bool,

    pub brush_overcurrent_main: bool,
    pub brush_overcurrent_side: bool,

    pub button_clock: bool,
    pub button_schedule: bool,
    pub button_day: bool,
    pub button_hour: bool,
    pub button_minute: bool,
    pub button_dock: bool,
    pub button_spot: bool,
    pub button_clean: bool,

    pub bumper_left: bool,
    pub bumper_front_left: bool,
    pub bumper_center_left: bool,
    pub bumper_center_right: bool,
    pub bumper_front_right: bool,
    pub bumper_right: bool,

    pub bumper_signal_left: u16,
    pub bumper_signal_front_left: u16,
    pub bumper_signal_center_left: u16,
    pub bumper_signal_right: u16,
    pub bumper_signal_front_right: u16,
    pub bumper_signal_center_right: u16,

    pub cliff_signal_left: u16,
    pub cliff_signal_front_left: u16,
    pub cliff_signal_right: u16,
    pub cliff_signal_front_right: u16,

    pub wall_signal: u16,
    pub dirt_detect: u8,

    pub motor_current_left: i16,
    pub motor_current_right: i16,
    pub motor_current_main: i16,
    pub motor_current_side: i16,

    // motion sensors
    pub distance: Distance,
    pub angle: Angle,
    pub requested_velocity: i16,
    pub requested_radius: i16,
    pub requested_velocity_left: i16,
    pub requested_velocity_right: i16,
    pub encoder_count_left: i16,
    pub encoder_count_right: i16,

    // infrared sensors
    pub infrared_omni: u8,
    pub infrared_left: u8,
    pub infrared_right: u8,

    // battery info
    pub charging_state: charging::State,
    pub charging_sources: charging::Source,
    pub battery_voltage: u16,
    pub battery_current: i16,
    pub battery_temperature: u8,
    pub battery_charge: u16,
    pub battery_capacity: u16,

    // music
    pub song_number: u8,
    pub song_playing: u8,

    // misc
    pub mode: OiMode,
    pub number_of_stream_packets: u8,
    pub stasis: Stasis,

    // private bookeeping
    prev_distance_left: i16,
    prev_distance_right: i16,

    prev_angle_left: i16,
    prev_angle_right: i16,
}

static mut OI: Option<()> = Some(());

impl OpenInterface {
    /// Initialize and take the instance of the OpenInterface
    ///
    /// Guaranteed to return Some on first call, any subsequent calls will return None.
    pub fn take() -> Option<Self> {
        cortex_m::interrupt::free(|_| unsafe { OI.take() })?;

        let mut new = Self {
            state: State::default(),
        };

        cortex_m::interrupt::free(|cs| new.init(cs));
        Some(new)
    }

    /// Set the leds on the iRobot
    pub fn set_leds(&mut self, play: bool, advance: bool, color: u8, intensity: u8) {
        self.uart_send(OPCODE_LEDS);
        let info = u8::from(advance) << 3 & u8::from(play) << 2;
        self.uart_send(info);
        self.uart_send(color);
        self.uart_send(intensity);
    }

    /// Set the wheels to a given speed
    ///
    /// Positive is forwards
    /// Negative is backwards
    pub fn set_wheels(&mut self, left: i16, right: i16) {
        // todo allow calibration
        self.uart_send(OPCODE_DRIVE_WHEELS);
        self.uart_send((right >> 8) as u8);
        #[allow(clippy::cast_possible_truncation)]
        self.uart_send(right as u8);
        self.uart_send((left >> 8) as u8);
        #[allow(clippy::cast_possible_truncation)]
        self.uart_send(left as u8);
    }

    /// Update the OpenInterface state from the iRobot
    ///
    /// Ensures that values within the state change to match iRobot reports.
    pub fn update(&mut self) {
        let mut buf = [0u8; SENSOR_PACKET_SIZE];

        self.uart_send(OPCODE_SENSORS);
        self.uart_send(SENSOR_PACKET_GROUP100);

        for i in buf.iter_mut() {
            *i = self.uart_recieve();
        }

        self.parse_packet(buf);

        SpinTimer.wait_millis(25); // reduces UART errors apparently
    }

    fn init(&mut self, cs: &CriticalSection) {
        self.init_uart(cs);
        self.uart_send(OPCODE_START);
        self.uart_send(OPCODE_FULL);
        self.set_leds(true, true, 7, 255);

        self.update();
        self.update(); // update twice to clear the distance measurements
    }

    fn init_uart(&self, cs: &CriticalSection) {
        let cybot = get_cybot();
        let sysctl = cybot.sysctl.borrow(cs);
        let gpioc = cybot.gpioc.borrow(cs);
        let uart = cybot.uart4.borrow(cs);

        // BRD=SYSCLK/((ClkDiv)(BaudRate)), HSE=0 ClkDiv=16, BaudRate=115,200
        const I_BRD: u32 = 0x08;
        // Fractional remainder is 0.6805, DIVFRAC = (.6805)(64)+0.5 = 44
        const F_BRD: u32 = 44;

        //sysctl.rcgcgpio.write(|w| w.r2().set_bit());

        sysctl.rcgcgpio.modify(|_, w| w.r2().set_bit()); // enable GPIO C
        sysctl.rcgcuart.modify(|_, w| w.r4().set_bit()); // enable UART4

        unsafe {
            gpioc.afsel.modify(|r, w| w.bits(r.bits() | (BIT4 | BIT5)));
            gpioc.pctl.modify(|r, w| w.bits(r.bits() | 0x00110000));
            gpioc.den.modify(|r, w| w.bits(r.bits() | (BIT4 | BIT5)));
            gpioc.dir.modify(|r, w| w.bits(r.bits() | BIT5));
            gpioc.dir.modify(|r, w| w.bits(r.bits() & !BIT4));
        }
        uart.ctl.write(|w| w.uarten().clear_bit()); // disable uart for setup
        unsafe {
            uart.ibrd.write(|w| w.bits(I_BRD));
            uart.fbrd.write(|w| w.bits(F_BRD));
        }
        uart.lcrh.write(|w| w.wlen()._8());
        uart.cc.write(|w| w.cs().sysclk());
        // re-enable uart and Rx / Tx
        uart
            .ctl
            .write(|w| w.rxe().set_bit().txe().set_bit().uarten().set_bit());
    }

    fn parse_packet(&mut self, packet: [u8; SENSOR_PACKET_SIZE]) {
        self.state.wheel_drop_left = (packet[0] & 0x08) >> 3 == 1;
        self.state.wheel_drop_right = (packet[0] & 0x04) >> 2 == 1;
        self.state.bump_left = (packet[0] & 0x02) >> 1 == 1;
        self.state.bump_right = packet[0] & 0x01 == 1;

        self.state.wall_sensor = packet[1] != 0;

        self.state.cliff_left = packet[2] != 0;
        self.state.cliff_front_left = packet[3] != 0;
        self.state.cliff_front_right = packet[4] != 0;
        self.state.cliff_right = packet[5] != 0;

        self.state.wall_virtual = packet[6] != 0;

        self.state.wheel_overcurrent_left = (packet[7] & 0x10) >> 4 == 1;
        self.state.wheel_overcurrent_right = (packet[7] & 0x08) >> 3 == 1;
        self.state.brush_overcurrent_main = (packet[7] & 0x04) >> 2 == 1;
        self.state.brush_overcurrent_side = packet[7] & 0x01 == 1;

        self.state.dirt_detect = packet[8];

        // byte 9 is unused

        self.state.infrared_omni = packet[10];

        self.state.button_clock = (packet[11] & 0x80) >> 7 == 1;
        self.state.button_schedule = (packet[11] & 0x40) >> 6 == 1;
        self.state.button_day = (packet[11] & 0x20) >> 5 == 1;
        self.state.button_hour = (packet[11] & 0x10) >> 4 == 1;
        self.state.button_minute = (packet[11] & 0x08) >> 3 == 1;
        self.state.button_dock = (packet[11] & 0x04) >> 2 == 1;
        self.state.button_spot = (packet[11] & 0x02) >> 1 == 1;
        self.state.button_clean = packet[11] & 0x01 == 1;

        self.state.charging_state = charging::State::from(packet[16]);
        self.state.battery_voltage = Self::parse_u16(&packet[17..]);
        self.state.battery_current = Self::parse_i16(&packet[19..]);
        self.state.battery_temperature = packet[21];
        self.state.battery_charge = Self::parse_u16(&packet[22..]);
        self.state.battery_capacity = Self::parse_u16(&packet[24..]);

        self.state.wall_signal = Self::parse_u16(&packet[26..]);

        self.state.cliff_signal_left = Self::parse_u16(&packet[28..]);
        self.state.cliff_signal_front_left = Self::parse_u16(&packet[30..]);
        self.state.cliff_signal_front_right = Self::parse_u16(&packet[32..]);
        self.state.cliff_signal_right = Self::parse_u16(&packet[34..]);

        // bytes 36-38 unused

        self.state.charging_sources = charging::Source::from(packet[39]);
        self.state.mode = OiMode::from(packet[40]);

        self.state.song_number = packet[41];
        self.state.song_playing = packet[42];

        self.state.number_of_stream_packets = packet[43];

        self.state.requested_velocity = Self::parse_i16(&packet[44..]);
        self.state.requested_radius = Self::parse_i16(&packet[46..]);
        self.state.requested_velocity_right = Self::parse_i16(&packet[48..]);
        self.state.requested_velocity_left = Self::parse_i16(&packet[50..]);
        self.state.encoder_count_left = Self::parse_i16(&packet[52..]);
        self.state.encoder_count_right = Self::parse_i16(&packet[54..]);

        self.state.bumper_right = (packet[56] & 0x20) >> 5 == 1;
        self.state.bumper_front_right = (packet[56] & 0x10) >> 4 == 1;
        self.state.bumper_center_right = (packet[56] & 0x08) >> 3 == 1;
        self.state.bumper_center_left = (packet[56] & 0x04) >> 2 == 1;
        self.state.bumper_front_left = (packet[56] & 0x02) >> 1 == 1;
        self.state.bumper_left = packet[56] & 0x01 == 1;

        self.state.bumper_signal_left = Self::parse_u16(&packet[57..]);
        self.state.bumper_signal_front_left = Self::parse_u16(&packet[59..]);
        self.state.bumper_signal_center_left = Self::parse_u16(&packet[61..]);
        self.state.bumper_signal_center_right = Self::parse_u16(&packet[63..]);
        self.state.bumper_signal_front_right = Self::parse_u16(&packet[65..]);
        self.state.bumper_signal_right = Self::parse_u16(&packet[67..]);

        self.state.infrared_left = packet[69];
        self.state.infrared_right = packet[70];

        self.state.motor_current_left = Self::parse_i16(&packet[71..]);
        self.state.motor_current_right = Self::parse_i16(&packet[73..]);
        self.state.motor_current_main = Self::parse_i16(&packet[75..]);
        self.state.motor_current_side = Self::parse_i16(&packet[77..]);

        self.state.stasis = Stasis::from(packet[79]);

        self.state.distance = self.get_distance();
        self.state.angle = self.get_angle();
    }

    fn parse_u16(packet: &[u8]) -> u16 {
        u16::from_be_bytes([packet[0], packet[1]])
    }

    fn parse_i16(packet: &[u8]) -> i16 {
        i16::from_be_bytes([packet[0], packet[1]])
    }

    fn uart_send(&self, data: u8) {
        cortex_m::interrupt::free(|cs| {
            let uart = get_cybot().uart4.borrow(cs);
            // holds until no data in transmit buffer
            while uart.fr.read().txff().bit() {}

            uart.dr.write(|w| unsafe { w.bits(u32::from(data)) })
        });
    }

    fn uart_recieve(&self) -> u8 {
        cortex_m::interrupt::free(|cs| {
            let uart = get_cybot().uart4.borrow(cs);
            // hold until data is recieved
            while uart.fr.read().rxfe().bit() {}

            uart.dr.read().data().bits()
        })
    }

    fn get_distance(&mut self) -> Distance {
        const MAGIC: f32 = 72.0 * core::f32::consts::PI / 508.8;

        let left_diff = self.encoder_count_left - self.prev_distance_left;
        let right_diff = self.encoder_count_right - self.prev_distance_right;

        let dist_left = f32::from(left_diff) * MAGIC;
        let dist_right = f32::from(right_diff) * MAGIC;

        self.state.prev_distance_left = self.encoder_count_left;
        self.state.prev_distance_right = self.encoder_count_right;

        Distance::from_mm((dist_left + dist_right) / 2.0)
    }

    fn get_angle(&mut self) -> Angle {
        const MAGIC: f32 = 72.0 * core::f32::consts::PI / 508.8;

        let prev_equals_current = self.encoder_count_left == self.prev_angle_left
            && self.encoder_count_right == self.prev_angle_right;
        let prev_angles_zero = self.prev_angle_left == 0 || self.prev_angle_right == 0;
        if prev_equals_current || prev_angles_zero {
            // happens on first pass or when the robot does not move
            self.state.prev_angle_left = self.encoder_count_left;
            self.state.prev_angle_right = self.encoder_count_right;
            return Angle::from_rad(0.0);
        }
        let left_diff = self.encoder_count_left - self.prev_angle_left;
        let right_diff = self.encoder_count_right - self.prev_angle_right;

        let dist_left = f32::from(left_diff) * MAGIC;
        let dist_right = f32::from(right_diff) * MAGIC;
        self.state.prev_angle_left = self.encoder_count_left;
        self.state.prev_angle_right = self.encoder_count_right;

        Angle::from_rad((dist_right - dist_left) / 235.0)
    }
}

impl Deref for OpenInterface {
    type Target = State;

    #[inline]
    fn deref(&self) -> &Self::Target {
        &self.state
    }
}

impl Drop for OpenInterface {
    fn drop(&mut self) {
        self.set_wheels(0, 0);
        self.uart_send(OPCODE_STOP);
    }
}
