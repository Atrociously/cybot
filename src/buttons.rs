use core::ops::BitAnd;
use core::ops::BitAndAssign;
use core::ops::BitOr;
use core::ops::BitOrAssign;

use tm4c123x_hal::tm4c123x::GPIO_PORTE;

use crate::bits::*;
use crate::CyBot;

const BTN_PINS: u32 = BIT0 | BIT1 | BIT2 | BIT3;

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Buttons(u8);

impl Buttons {
    pub const BUTTON1: Buttons = Buttons(0x1);
    pub const BUTTON2: Buttons = Buttons(0x2);
    pub const BUTTON3: Buttons = Buttons(0x4);
    pub const BUTTON4: Buttons = Buttons(0x8);

    pub const NONE: Buttons = Buttons(0);
    pub const ALL: Buttons = Buttons(Self::BUTTON1.0 | Self::BUTTON2.0 | Self::BUTTON3.0 | Self::BUTTON4.0);
}

pub struct ButtonManager<'a> {
    gpio: &'a GPIO_PORTE,
}

static mut BTNMGR: Option<()> = Some(());

impl<'a> ButtonManager<'a> {
    /// Initialize and take the button manager instance
    ///
    /// Guaranteed to return Some on first call, all subsequent calls return None
    pub fn take(cybot: &'a CyBot) -> Option<Self> {
        // ensure only one button manager can be created
        cortex_m::interrupt::free(|_| unsafe { BTNMGR.take() })?;

        let sysctl = &cybot.peripherals.SYSCTL;
        let gpio = &cybot.peripherals.GPIO_PORTE;

        sysctl.rcgcgpio.modify(|_, w| w.r4().set_bit()); // enable gpio e
        while sysctl.prgpio.read().r4().bit_is_clear() {} // wait for the gpio to be ready
                                                          // setup gpio pins to be inputs and enable them
        gpio.dir
            .modify(|r, w| unsafe { w.bits(r.bits() & !BTN_PINS) });
        gpio.den
            .modify(|r, w| unsafe { w.bits(r.bits() | BTN_PINS) });

        Some(Self { gpio })
    }

    /// Get the current state of the buttons
    ///
    /// Does not wait for a button to be pressed
    /// just returns the current state.
    pub fn get_buttons(&self) -> Buttons {
        let data = self.gpio.data.read().bits();
        let data: u8 = (data & BTN_PINS).try_into().unwrap();
        // the buttons are zero if they are pressed therefore invert the data bits
        Buttons(!data & 0x0F)
    }

    /// Wait for a specific button or buttons to be pressed
    ///
    /// Returns the state when the button was pressed
    pub fn wait_buttons(&self, btn: Buttons) -> Buttons {
        let mut status = self.get_buttons();
        while status & btn == Buttons::NONE {
            status = self.get_buttons();
        }
        status
    }
}

impl BitOr for Buttons {
    type Output = Self;

    fn bitor(self, rhs: Self) -> Self::Output {
        Buttons(self.0 | rhs.0)
    }
}

impl BitOrAssign for Buttons {
    fn bitor_assign(&mut self, rhs: Self) {
        self.0 |= rhs.0;
    }
}

impl BitAnd for Buttons {
    type Output = Self;

    fn bitand(self, rhs: Self) -> Self::Output {
        Buttons(self.0 & rhs.0)
    }
}

impl BitAndAssign for Buttons {
    fn bitand_assign(&mut self, rhs: Self) {
        self.0 &= rhs.0;
    }
}
