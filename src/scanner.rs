use tm4c123x_hal::tm4c123x::GPIO_PORTB;

use crate::{measure::Angle, CyBot};

#[derive(Clone, Copy, Debug, Default)]
pub struct ScanResult {
    pub sound_dist: f32,
    pub ir_raw_val: i32,
}

#[derive(Clone, Copy, Debug, Default)]
pub struct ScanOptions {
    pub motor: bool,
    pub ping: bool,
    pub ir: bool,
}

#[derive(Clone, Copy, Debug)]
pub struct Extent {
    left: u32,
    right: u32,
}

pub struct Scanner<'a> {
    gpio: &'a GPIO_PORTB,
    extents: Extent,
    // TODO: fill out
}

static mut SCANNER: Option<()> = Some(());

impl<'a> Scanner<'a> {
    pub fn take(cybot: &'a CyBot) -> Option<Self> {
        cortex_m::interrupt::free(|_| unsafe { SCANNER.take() })?;

        let sysctl = &cybot.peripherals.SYSCTL;
        let gpio = &cybot.peripherals.GPIO_PORTB;

        sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit()); // enable clocks for port b

        // TODO: complete scanner setup

        Some(Self {
            gpio,
            extents: Extent {
                left: 2_000_000,
                right: 300_000,
            },
        })
    }

    /// Set the left extent
    ///
    /// For scanning purposes the left extent is defined as 180 degrees
    pub fn set_extent_left(&mut self, extent: u32) {
        self.extents.left = extent;
    }

    /// Set the right extent
    ///
    /// For scanning purposes the right extent is defined as 0 degrees.
    pub fn set_extent_right(&mut self, extent: u32) {
        self.extents.right = extent;
    }

    /// Returns the current left and right extents
    pub fn get_extents(&self) -> Extent {
        self.extents
    }

    /// Run the calibration program to set left and right extents manually
    ///
    /// If you want to get the new extents use [`get_extents`]
    pub fn run_calibration(&mut self) {
        todo!()
    }

    /// Scan at the given angle with the provided ScanOptions
    /// 
    /// If the motor is not enabled the scan will still happen
    /// but the motor will not move to the specified angle
    pub fn scan(&mut self, angle: Angle, opts: ScanOptions) -> ScanResult {
        todo!()
    }
}
