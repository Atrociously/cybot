//! Abstraction for the ISU CyBot Platform
//!
//! Provides easy to use abstractions for many of the features of the CyBot Platform.

#![feature(core_intrinsics)]
#![feature(alloc_error_handler)]
#![feature(const_option)]
#![no_std]

extern crate alloc;

pub extern crate cortex_m;
pub extern crate cortex_m_rt;
use cfg_if::cfg_if;
pub use cortex_m_rt::entry;

use core::panic::PanicInfo;
use core::mem::MaybeUninit;
use alloc::alloc::Layout;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 4096;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

mod lcd;
mod open_interface;
mod time;

pub use lcd::{Lcd, LCD_HEIGHT, LCD_TOTAL_CHARS, LCD_WIDTH};
pub use open_interface::{charging, distance, OiMode, OpenInterface, Stasis};
pub use time::SpinTimer;

pub struct CyBot {
    pub lcd: Lcd,
    pub oi: OpenInterface,
}

static mut CYBOT_GUARD: bool = false;

pub fn init() -> CyBot {
    if unsafe { CYBOT_GUARD } {
        panic!("Cannot initialize the cybot library more than once!");
    }
    unsafe { CYBOT_GUARD = true }
    let p = tm4c123x_hal::Peripherals::take().expect("unable to aquire peripherals make sure they aren't in use elsewhere");

    // initialize the heap available to us
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

    let lcd = Lcd::new(p.GPIO_PORTD, p.GPIO_PORTF, &p.SYSCTL.rcgcgpio);
    let oi = OpenInterface::new(p.UART4);

    CyBot { lcd, oi }
}

#[alloc_error_handler]
fn oom(_: Layout) -> ! {
    cfg_if! {
        if #[cfg(debug_assertions)] {
            cortex_m::asm::bkpt();
            loop {}
        } else {
            core::intrinsics::abort();
        }
    }
}

#[panic_handler]
fn panic(_: &PanicInfo) -> ! {
    cfg_if! {
        if #[cfg(debug_assertions)] {
            cortex_m::asm::bkpt();
            loop {}
        } else {
            core::intrinsics::abort();
        }
    }
}
