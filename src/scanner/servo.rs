use crate::get_cybot;
use crate::bits::*;

pub struct Servo {
    pwm_max: f32, // max time in milliseconds for high pulse
    pwm_min: f32, // min time in milliseconds for high pulse
}

const PULSE_TIME: f32 = 20.0;
const MICROS_PER_CYCLE: f32 = 6.25e-5;
const MAX_CYCLES: u32 = (PULSE_TIME / MICROS_PER_CYCLE) as u32;

static mut SERVO: Option<Servo> = Some(Servo { pwm_min: 0.0, pwm_max: 0.0 });

impl Servo {
    pub fn take() -> Option<Self> {
        Self::take_with(0.5, 2.25)
    }

    pub fn take_with(pwm_min: f32, pwm_max: f32) -> Option<Self> {
        let mut servo = cortex_m::interrupt::free(|_| unsafe { SERVO.take() })?;
        servo.pwm_min = pwm_min;
        servo.pwm_max = pwm_max;

        cortex_m::interrupt::free(|cs| {
            let cy = get_cybot();
            let sysctl = cy.sysctl.borrow(cs);
            let gpiob = cy.gpiob.borrow(cs);
            let timer1 = cy.timer1.borrow(cs);

            sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit());
            sysctl.rcgctimer.modify(|_, w| w.r1().set_bit());

            while sysctl.prgpio.read().r1().bit_is_clear() {}
            while sysctl.prtimer.read().r1().bit_is_clear() {}

            gpiob.den.modify(|r, w| unsafe { w.bits(r.bits() | BIT5) });
            gpiob.dir.modify(|r, w| unsafe { w.bits(r.bits() | BIT5) });
            gpiob.afsel.modify(|r, w| unsafe { w.bits(r.bits() | BIT5) });
            gpiob.pctl.modify(|r, w| unsafe { w.bits(r.bits() | 0x700000) });

            timer1.ctl.modify(|_, w| w.tben().clear_bit());

            timer1.cfg.modify(|_, w| w.cfg()._16_bit());
            timer1.tbmr.modify(|_, w| w.tbmr().period().tbams().set_bit());

            timer1.tbpr.write(|w| unsafe { w.bits(MAX_CYCLES >> 16) });
            timer1.tbilr.write(|w| unsafe { w.bits(MAX_CYCLES & 0xFFFF) });
            servo.set_pwm(pwm_min);

            timer1.ctl.modify(|_, w| w.tben().set_bit());
        });
        Some(servo)
    }

    fn set_pwm(&self, high_time: f32) {
        if high_time < 0.0 || high_time.is_nan() {
            return;
        }
        let high_cyles: u32 = libm::floorf(high_time / MICROS_PER_CYCLE) as u32;
        let low_point = MAX_CYCLES - high_cyles;

        cortex_m::interrupt::free(|cs| {
            let cy = get_cybot();
            let timer1 = cy.timer1.borrow(cs);

            timer1.ctl.modify(|_, w| w.tben().clear_bit());
            timer1.tbpmr.write(|w| unsafe { w.bits(low_point >> 16) });
            timer1.tbmatchr.write(|w| unsafe { w.bits(low_point & 0xFFFF) });
            timer1.ctl.modify(|_, w| w.tben().set_bit());
        });
    }

    pub fn set_min(&mut self, min: f32) {
        self.pwm_min = min.min(0.0).max(20.0);
    }

    pub fn set_max(&mut self, max: f32) {
        self.pwm_max = max.min(0.0).max(20.0);
    }

    pub fn move_to(&self, angle: f32) {
        let angle = angle.max(0.0).min(180.0);

        let pwm_range = self.pwm_min - self.pwm_min;
        let pwm = ((angle * pwm_range) / 180.0) + self.pwm_min;

        self.set_pwm(pwm);
    }
}
