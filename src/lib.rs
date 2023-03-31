//! Abstraction for the ISU CyBot Platform
//!
//! Provides easy to use abstractions for many of the features of the CyBot Platform.
//!
//! ## Hello World Example
//! ```notest
//! use core::fmt::Write;
//! use cybot::{entry, CyBot, Lcd};
//!
//! #[entry]
//! fn main(cybot: &Cybot) {
//!     let lcd = &mut Lcd::take(cybot).unwrap();
//!     write!(lcd, "Hello, World!").unwrap();
//! }
//! ```

#![feature(once_cell, alloc_error_handler)]
#![deny(
    clippy::cast_lossless,
    clippy::cast_possible_wrap,
    clippy::cast_possible_truncation
)]
#![no_std]

extern crate alloc;
pub extern crate libm;

use hal::tm4c123x::TIMER3;
pub use tm4c123x_hal as hal;
pub use hal::tm4c123x as cpu;
use cpu::{ADC0, GPIO_PORTB, GPIO_PORTD, GPIO_PORTE, GPIO_PORTF, SYSCTL, UART1, UART4, GPIO_PORTC};

pub extern crate cortex_m;
pub extern crate cortex_m_rt;
use cortex_m::{interrupt::Mutex, peripheral::NVIC};
pub use cortex_m_rt::entry;

use cfg_if::cfg_if;

use alloc::alloc::Layout;
use core::{panic::PanicInfo, mem::MaybeUninit};
use util::{CriticalCell, CriticalOnce};

use embedded_alloc::Heap;

mod util;

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
pub use scanner::{ScanOptions, ScanResult, Scanner, IrSensor, Ping};
pub use time::SpinTimer;
pub use uartcom::UartCom;

const HEAP_SIZE: usize = 4096;
static mut HEAP_MEM: [MaybeUninit<u8>; HEAP_SIZE] = [MaybeUninit::uninit(); HEAP_SIZE];
#[global_allocator]
static HEAP: Heap = Heap::empty();

static CYBOT: CriticalOnce<CyBot> = CriticalOnce::new();

pub struct CyBot {
    pub(crate) adc0: Mutex<ADC0>,
    pub(crate) timer3: Mutex<TIMER3>,
    pub(crate) uart1: Mutex<UART1>,
    pub(crate) uart4: Mutex<UART4>,
    pub(crate) gpiob: Mutex<GPIO_PORTB>,
    pub(crate) gpiod: Mutex<GPIO_PORTD>,
    pub(crate) gpioc: Mutex<GPIO_PORTC>,
    pub(crate) gpioe: Mutex<GPIO_PORTE>,
    pub(crate) gpiof: Mutex<GPIO_PORTF>,
    pub(crate) sysctl: Mutex<SYSCTL>,
    pub(crate) nvic: CriticalCell<NVIC>, // allow mutability within critical sections
}

fn get_cybot() -> &'static CyBot {
    cortex_m::interrupt::free(|cs| {
        CYBOT.get_or_init(cs, || {
            // initialize heap once
            unsafe { HEAP.init(HEAP_MEM.as_ptr() as usize, HEAP_SIZE) }
            

            let core = hal::CorePeripherals::take().unwrap();
            let perp = hal::Peripherals::take().unwrap();

            CyBot {
                adc0: Mutex::new(perp.ADC0),
                timer3: Mutex::new(perp.TIMER3),
                uart1: Mutex::new(perp.UART1),
                uart4: Mutex::new(perp.UART4),
                gpiob: Mutex::new(perp.GPIO_PORTB),
                gpiod: Mutex::new(perp.GPIO_PORTD),
                gpioc: Mutex::new(perp.GPIO_PORTC),
                gpioe: Mutex::new(perp.GPIO_PORTE),
                gpiof: Mutex::new(perp.GPIO_PORTF),
                sysctl: Mutex::new(perp.SYSCTL),
                nvic: CriticalCell::new(core.NVIC),
            }
        })
    })
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
