use core::sync::atomic::{AtomicBool, Ordering};

mod infrared;
mod ping;
mod servo;

pub use infrared::IrSensor;
pub use ping::Ping;
pub use servo::Servo;

use crate::measure::{Angle, Distance};

#[derive(Clone, Copy, Debug, Default)]
pub struct ScanResult {
    pub sound_dist: Distance,
    pub ir_dist: Distance,
}

pub struct Scanner {
    ir: IrSensor,
    ping: Ping,
    servo: Servo,
}

static TAKEN: AtomicBool = AtomicBool::new(false);

impl Scanner {
    pub fn take() -> Option<Self> {
        if TAKEN.fetch_update(Ordering::Relaxed, Ordering::Relaxed, |_| Some(true)) == Ok(true) {
            return None;
        }
        Some(Scanner {
            ir: IrSensor::take()?,
            ping: Ping::take()?,
            servo: Servo::take()?,
        })
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
    pub fn scan(&mut self, angle: Angle) -> ScanResult {
        self.servo.move_to(angle.as_deg());
        let sound_dist = self.ping.get_distance();
        let ir_dist = self.ir.get_distance();

        ScanResult {
            sound_dist,
            ir_dist,
        }
    }
}
