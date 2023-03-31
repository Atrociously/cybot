use core::ops::RangeInclusive;

mod infrared;
mod ping;

pub use infrared::IrSensor;
pub use ping::Ping;

use crate::{
    get_cybot,
    measure::{Angle, Distance},
};

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

pub struct Scanner {
    extents: Extent,
    // TODO: fill out
}

static mut SCANNER: Option<Scanner> = Some(Scanner {
    extents: Extent {
        left: 2_000_000,
        right: 300_000,
    },
});

impl Scanner {
    pub fn take() -> Option<Self> {
        let cybot = get_cybot();
        let scanner = cortex_m::interrupt::free(|_| unsafe { SCANNER.take() })?;

        cortex_m::interrupt::free(|cs| {
            let sysctl = cybot.sysctl.borrow(cs);
            //let gpiob = cybot.gpiob.borrow(cs);

            sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit()); // enable clocks for port but

            // TODO: complete scanner setup
        });
        Some(scanner)
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
        todo!("{angle:?} {opts:?}")
    }
}

#[allow(dead_code)]
fn convert_ir(raw: i32) -> Option<Distance> {
    const INPUT_RANGE: RangeInclusive<f32> = 0.0..=4096.0;
    const VOLTAGE_RANGE: RangeInclusive<f32> = 0.0..=3.3;
    const INPUT_VOLTAGE_RANGE: RangeInclusive<f32> = 0.4..=2.6;
    const INV_DIST_RANGE: RangeInclusive<f32> = 0.012..=0.111;

    // if the raw data does not fit in an i16 it is out of range anyways
    let raw = i16::try_from(raw).ok()?;
    let raw: f32 = f32::from(raw);

    fn convert_value(v: f32, input: &RangeInclusive<f32>, output: &RangeInclusive<f32>) -> f32 {
        let factor = (output.end() - output.start()) / (input.end() - input.start());
        output.start() + factor * (v - input.start())
    }

    // if the value is outside the valid input range
    // we cannot compute a proper value so abort
    if !INPUT_RANGE.contains(&raw) {
        return None;
    }

    let voltage_val = convert_value(raw, &INPUT_RANGE, &VOLTAGE_RANGE);

    // if the voltage is outside the valid input range of the sensor
    // we cannot compute a proper value, or the data is out of range
    // anyways so abort
    if !INPUT_VOLTAGE_RANGE.contains(&voltage_val) {
        return None;
    }

    let inv_dist_val = convert_value(voltage_val, &INPUT_VOLTAGE_RANGE, &INV_DIST_RANGE);

    let distance = inv_dist_val.recip() - 0.42;
    Some(Distance::from_cm(distance))
}
