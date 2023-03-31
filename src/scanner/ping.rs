use cortex_m::peripheral::NVIC;

use crate::hal::interrupt;
use crate::measure::Distance;
use crate::util::CriticalCell;
use crate::{get_cybot, bits::*, SpinTimer};

#[derive(Clone, Copy)]
enum PingState {
    Low,
    High,
    Done(u32), // number of ticks between lows
}

impl PingState {
    /// Returns `true` if the ping state is [`Done`].
    ///
    /// [`Done`]: PingState::Done
    #[must_use]
    fn is_done(self) -> bool {
        matches!(self, Self::Done(..))
    }

    fn as_done(self) -> u32 {
        if let Self::Done(v) = self {
            v
        } else {
            panic!("cast to done when not done")
        }
    }
}

const MICROS_PER_TICK: f32 = 0.0625;
const SOUND_CM_PER_MICRO: f32 = 0.0343;

static STATE: CriticalCell<PingState> = CriticalCell::new(PingState::Low);

static mut PING: Option<Ping> = Some(Ping(()));

pub struct Ping(());

impl Ping {
    pub fn take() -> Option<Self> {
        let cy = get_cybot();
        cortex_m::interrupt::free(|cs| {
            let ping = unsafe { PING.take() }?;
            
            let sysctl = cy.sysctl.borrow(cs);
            let gpiob = cy.gpiob.borrow(cs);
            let timer = cy.timer3.borrow(cs);

            sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit());  // enable port b
            sysctl.rcgctimer.modify(|_, w| w.r3().set_bit()); // enable timer 3
            
            while sysctl.prgpio.read().r1().bit_is_clear() {} // wait for ready
            while sysctl.prtimer.read().r3().bit_is_clear() {} // wait for ready

            // setup gpio for digital output when we need to trigger the start pulse
            gpiob.den.modify(|r, w| unsafe { w.bits(r.bits() | BIT3) });
            gpiob.dir.modify(|r, w| unsafe { w.bits(r.bits() | BIT3) });

            // disable timer for setup
            timer.ctl.modify(|_, w| w.tben().clear_bit());

            timer.cfg.modify(|_, w| unsafe { w.bits(0x4) }); // setup timers as 16-bit
            timer.tbmr.modify(|_, w| w.tbcmr().set_bit()); // setup timer for capture mode
            
            unsafe { NVIC::unmask(interrupt::TIMER3B) }; // enable interrupts for timer3
            timer.icr.write(|w| w.cbmcint().set_bit().cbecint().set_bit()); // clear interrupts
            timer.imr.modify(|_, w| w.cbmim().clear_bit().cbeim().clear_bit()); // enable interupts
            Some(ping)
        })
    }

    pub fn ping_trigger(&self) {
        let cy = get_cybot();
        cortex_m::interrupt::free(|cs| {
            let mut state = STATE.borrow_mut(cs);
            *state = PingState::Low;

            let timer = cy.timer3.borrow(cs);
            let gpio = cy.gpiob.borrow(cs);

            timer.ctl.modify(|_, w| w.tben().clear_bit());
            timer.imr.modify(|_, w| w.cbeim().clear_bit());
            gpio.afsel.modify(|r, w| unsafe { w.bits(r.bits() | !BIT3) });

            // trigger start pulse
            gpio.data.modify(|r, w| unsafe { w.bits(r.bits() | BIT3) });
            SpinTimer.wait_micros(2);
            gpio.data.modify(|r, w| unsafe { w.bits(r.bits() | !BIT3)});

            timer.icr.write(|w| w.cbecint().set_bit()); // clear erroneous interrupt
            gpio.afsel.modify(|r, w| unsafe { w.bits(r.bits() | BIT3) }); // enable alternate fn
            timer.imr.modify(|_, w| w.cbeim().set_bit()); // enable interrupts on timer
            timer.ctl.modify(|_, w| w.tben().set_bit()); // enable timer
        });
    }

    pub fn get_distance(&self) -> Distance {
        self.ping_trigger();
        while !matches!(STATE.read(), PingState::Done(..)) {}
        let value = STATE.read().as_done();
        cortex_m::interrupt::free(|cs| {
            let mut state = STATE.borrow_mut(cs);
            *state = PingState::Low;
            // todo reset code
        });
        let micros = MICROS_PER_TICK * value as f32;
        Distance::from_cm(micros * SOUND_CM_PER_MICRO)
    }
}

#[interrupt]
fn TIMER3B() {
    let cy = get_cybot();
    cortex_m::interrupt::free(|cs| {
        let timer = cy.timer3.borrow(cs);

        if timer.ris.read().cberis().bit_is_set() {
        }
    });
}
