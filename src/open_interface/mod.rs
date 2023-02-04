use core::ops::Deref;

use alloc::boxed::Box;
use tm4c123x_hal::tm4c123x::UART4;

use crate::SpinTimer;

pub mod charging;
pub mod distance;
mod mode;
mod stasis;

pub use mode::OiMode;
pub use stasis::Stasis;

const SENSOR_PACKET_SIZE: usize = 80;

const OI_OPCODE_SENSORS: u8 = 142;
const OI_SENSOR_PACKET_GROUP100: u8 = 100;

pub struct OpenInterface {
    // pointer to state on the heap
    state: Box<State>,
    uart: UART4,
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
    pub distance: f32,
    pub angle: f32,
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

    // bookeeping
    prev_distance_left: i16,
    prev_distance_right: i16,

    prev_angle_left: i16,
    prev_angle_right: i16,
}

impl OpenInterface {
    pub(crate) fn new(uart: UART4) -> Self {
        Self {
            state: Box::<State>::default(),
            uart,
        }
    }

    pub fn update(&mut self) {
        let mut buf = [0u8; SENSOR_PACKET_SIZE];

        self.uart_send(OI_OPCODE_SENSORS);
        self.uart_send(OI_SENSOR_PACKET_GROUP100);

        for i in 0..SENSOR_PACKET_SIZE {
            buf[i] = self.uart_recieve();
        }

        self.parse_packet(buf);

        SpinTimer.wait_millis(25); // reduces UART errors apparently
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
        self.state.bumper_front_left = packet[56] & 0x02 == 1;
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
        self.state.angle = self.get_angle_deg();
    }

    fn parse_u16(packet: &[u8]) -> u16 {
        u16::from_be_bytes([packet[0], packet[1]])
    }

    fn parse_i16(packet: &[u8]) -> i16 {
        i16::from_be_bytes([packet[0], packet[1]])
    }

    fn uart_send(&self, data: u8) {
        // holds until no data in transmit buffer
        while self.uart.fr.read().txff().bit() {}

        self.uart.dr.write(|w| unsafe { w.bits(data as u32) })
    }

    fn uart_recieve(&self) -> u8 {
        // hold until data is recieved
        while self.uart.fr.read().rxfe().bit() {}

        self.uart.dr.read().data().bits()
    }

    fn get_distance(&mut self) -> f32 {
        const MAGIC: f32 = 72.0 * core::f32::consts::PI / 508.8;

        let left_diff = self.encoder_count_left - self.prev_distance_left;
        let right_diff = self.encoder_count_right - self.prev_distance_right;

        let dist_left = f32::from(left_diff) * MAGIC;
        let dist_right = f32::from(right_diff) * MAGIC;

        self.state.prev_distance_left = self.encoder_count_left;
        self.state.prev_distance_right = self.encoder_count_right;

        return (dist_left + dist_right) / 2.0;
    }

    fn get_angle_rad(&mut self) -> f32 {
        const MAGIC: f32 = 72.0 * core::f32::consts::PI / 508.8;

        let prev_equals_current = self.encoder_count_left == self.prev_angle_left
            && self.encoder_count_right == self.prev_angle_right;
        let prev_angles_zero = self.prev_angle_left == 0 || self.prev_angle_right == 0;
        if prev_equals_current || prev_angles_zero {
            // happens on first pass or when the robot does not move
            self.state.prev_angle_left = self.encoder_count_left;
            self.state.prev_angle_right = self.encoder_count_right;
            return 0.0;
        }
        let left_diff = self.encoder_count_left - self.prev_angle_left;
        let right_diff = self.encoder_count_right - self.prev_angle_right;

        let dist_left = f32::from(left_diff) * MAGIC;
        let dist_right = f32::from(right_diff) * MAGIC;
        self.state.prev_angle_left = self.encoder_count_left;
        self.state.prev_angle_right = self.encoder_count_right;

        (dist_right - dist_left) / 235.0
    }

    fn get_angle_deg(&mut self) -> f32 {
        self.get_angle_rad() * 180.0 / core::f32::consts::PI
    }
}

impl Deref for OpenInterface {
    type Target = State;

    fn deref(&self) -> &Self::Target {
        self.state.deref()
    }
}
