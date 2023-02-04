use core::arch::asm;

#[derive(Clone, Copy)]
pub struct SpinTimer;

impl SpinTimer {
    /// Spins for `delay` microseconds
    pub fn wait_micros(self, mut delay: u32) {
        if delay <= 2 {
            return; // function overhead is around 1.5us
        } else {
            delay -= 2;
        }
        while delay > 0 {
            unsafe { asm!("NOP", "NOP", "NOP", "NOP", "NOP", "NOP") };
            delay -= 1;
        }
    }

    /// Spins for `delay` milliseconds
    pub fn wait_millis(self, mut delay: u32) {
        while delay > 0 {
            self.wait_micros(1000);
            delay -= 1;
        }
    }

    /// Spins for `delay` seconds
    pub fn wait(self, mut delay: u32) {
        while delay > 0 {
            self.wait_millis(1000);
            delay -= 1;
        }
    }
}
