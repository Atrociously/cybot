//! Abstraction for the ISU CyBot Platform
//!
//! Provides easy to use abstractions for many of the features of the CyBot Platform.

#![feature(alloc_error_handler)]
#![deny(
    clippy::cast_lossless,
    clippy::cast_possible_wrap,
    clippy::cast_possible_truncation
)]
#![no_std]

extern crate alloc;
pub extern crate libm;

pub extern crate cortex_m;
pub extern crate cortex_m_rt;
pub use cybot_macros::entry;

use cfg_if::cfg_if;
use tm4c123x_hal::{CorePeripherals, Peripherals};

use alloc::alloc::Layout;
use core::mem::MaybeUninit;
use core::panic::PanicInfo;

use embedded_alloc::Heap;

#[global_allocator]
static HEAP: Heap = Heap::empty();
const HEAP_SIZE: usize = 4096;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];

mod bits;
mod buttons;
mod lcd;
mod open_interface;
mod scanner;
mod time;
mod uartcom;

pub mod measure;

pub use buttons::{ButtonManager, Buttons};
pub use lcd::Lcd;
pub use open_interface::{charging, OiMode, OpenInterface, Stasis};
pub use scanner::{ScanOptions, ScanResult, Scanner};
pub use time::SpinTimer;
pub use uartcom::UartCom;

pub struct CyBot {
    #[allow(dead_code)]
    pub(crate) core: CorePeripherals,
    pub(crate) peripherals: Peripherals,
}

static mut CYBOT: Option<()> = Some(());

fn init() -> CyBot {
    cortex_m::interrupt::free(|_| unsafe { CYBOT.take() })
        .expect("Cannot initialize the cybot library more than once!");
    let core = tm4c123x_hal::CorePeripherals::take()
        .expect("unable to aquire core peripherals make sure they aren't in use elsewhere");
    let peripherals = tm4c123x_hal::Peripherals::take()
        .expect("unable to aquire peripherals make sure they aren't in use elsewhere");

    // initialize the heap available to us
    unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }

    CyBot { core, peripherals }
}

pub fn run<T, F: FnOnce(&CyBot) -> T>(f: F) -> ! {
    let cybot = init();
    // this ensures that any variables created within
    // the scope of the function are dropped
    f(&cybot);

    // once program has completed we are finished
    panic!("complete")
}

#[alloc_error_handler]
#[allow(clippy::empty_loop)]
fn oom(_: Layout) -> ! {
    cfg_if! {
        if #[cfg(debug_assertions)] {
            cortex_m::asm::bkpt();
            loop {}
        } else {
            cortex_m::asm::udf()
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
            cortex_m::asm::udf()
        }
    }
}
