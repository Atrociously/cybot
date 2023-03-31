use crate::{bits::*, get_cybot, util::CriticalCell};
//use alloc::collections::VecDeque;
use cortex_m::peripheral::NVIC;
use tm4c123x_hal::interrupt;

const MAX_UNICODE_CHAR: usize = 4;

pub struct UartCom(());

struct StackQueue<const N: usize> {
    size: usize,
    buf: [u8; N],
}

impl<const N: usize> StackQueue<N> {
    pub const fn new() -> Self {
        Self {
            size: 0,
            buf: [0; N],
        }
    }

    pub fn push_back(&mut self, v: u8) -> bool {
        if self.size >= N {
            return false;
        }

        self.buf[self.size] = v;
        self.size += 1;
        
        true
    }

    pub fn pop_front(&mut self) -> Option<u8> {
        // if there are no elements return None
        if self.size == 0 {
            return None;
        }
        let v = self.buf[0];
        // shift the queue
        for i in 1..self.size {
            self.buf[i-1] = self.buf[i];
        }
        self.size -= 1;

        Some(v)
    }
}

static UARTBUF: CriticalCell<StackQueue<10>> = CriticalCell::new(StackQueue::new());
static mut UARTCOM: Option<UartCom> = Some(UartCom(()));

impl UartCom {
    /// Initializes and takes the instance of the uart communicator
    ///
    /// Guaranteed to return Some on the first call, all subsequent calls return None
    pub fn take() -> Option<Self> {
        let uart_com = cortex_m::interrupt::free(|_| unsafe { UARTCOM.take() })?;
        let cybot = get_cybot();

        cortex_m::interrupt::free(|cs| {
            let sysctl = cybot.sysctl.borrow(cs);
            let gpio = cybot.gpiob.borrow(cs);
            let uart = cybot.uart1.borrow(cs);

            sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit()); // enable gpio b
            sysctl.rcgcuart.modify(|_, w| w.r1().set_bit()); // enable uart 1

            // wait for ready
            while sysctl.pruart.read().r1().bit_is_clear() {}
            while sysctl.prgpio.read().r1().bit_is_clear() {}

            gpio.afsel
                .modify(|r, w| unsafe { w.bits(r.bits() | BIT0 | BIT1) }); // set alternate function
                                                                           // set port control to uart1 for each gpio pin see page 1351 of datasheet for mappings
            gpio.pctl
                .modify(|r, w| unsafe { w.bits(r.bits() | 0x00000011) });
            gpio.den
                .modify(|r, w| unsafe { w.bits(r.bits() | BIT0 | BIT1) }); // enable uart ports
            gpio.dir.modify(|r, w| unsafe { w.bits(r.bits() | BIT1) }); // set Tx output
            gpio.dir.modify(|r, w| unsafe { w.bits(r.bits() & !BIT0) }); // set Rx input

            uart.ctl.modify(|_, w| w.uarten().clear_bit()); // disable uart for setup
                                                            // SYSCLK for lab cybot is 16Mhz
                                                            // BRD=SYSCLK/((ClkDiv)(BaudRate)), HSE=0 ClkDiv=16, BaudRate=115,200
                                                            // 16,000,000 / (16) / (115,200) = 8
            const I_BRD: u32 = 8;
            // Fractional remainder is 0.6805, DIVFRAC = (.6805)(64)+0.5 = 44
            const F_BRD: u32 = 44;
            uart.ibrd.write(|w| unsafe { w.bits(I_BRD) });
            uart.fbrd.write(|w| unsafe { w.bits(F_BRD) });
            uart.lcrh.write(|w| w.wlen()._8()); // set bits per frame to 8
            uart.cc.write(|w| w.cs().sysclk()); // set baud clock source to SYSCLK

            // enable interrupts
            uart.icr.write(|w| w.rxic().set_bit());
            uart.im.modify(|_, w| w.rxim().clear_bit());
            let cy = get_cybot();
            let mut nvic = cy.nvic.borrow_mut(cs);
            unsafe {
                NVIC::unmask(interrupt::UART1);
                nvic.set_priority(interrupt::UART1, 1);
            }

            // re-enable uart
            uart.ctl
                .modify(|_, w| w.rxe().set_bit().txe().set_bit().uarten().set_bit());
        });
        Some(uart_com)
    }

    pub fn enable_buffer(&mut self) {
        // enable interrupts
        cortex_m::interrupt::free(|cs| {
            let cy = get_cybot();
            let uart = cy.uart1.borrow(cs);
            uart.icr.write(|w| w.rxic().set_bit());
            uart.im.modify(|_, w| w.rxim().set_bit());
            let mut nvic = cy.nvic.borrow_mut(cs);
            unsafe {
                NVIC::unmask(interrupt::UART1);
                nvic.set_priority(interrupt::UART1, 1);
            }
        })
    }

    pub fn disable_buffer(&self) {
        // disable interrupts
        cortex_m::interrupt::free(|cs| {
            let uart = get_cybot().uart1.borrow(cs);
            uart.icr.write(|w| w.rxic().set_bit());
            uart.im.modify(|_, w| w.rxim().clear_bit());
            NVIC::mask(interrupt::UART1);
        })
    }

    fn uart_send(&self, data: u8) {
        cortex_m::interrupt::free(|cs| {
            let uart = get_cybot().uart1.borrow(cs);
            // holds until no data in transmit buffer
            while uart.fr.read().txff().bit() {}

            uart.dr.write(|w| unsafe { w.bits(u32::from(data)) })
        });
    }

    fn uart_recieve(&self) -> u8 {
        cortex_m::interrupt::free(|cs| {
            let uart = get_cybot().uart1.borrow(cs);
            // hold until data is recieved
            while uart.fr.read().rxfe().bit() {}

            uart.dr.read().data().bits()
        })
    }

    /// Put a char to the uart communication
    pub fn putc(&self, data: char) {
        let mut buf = [0; MAX_UNICODE_CHAR];
        data.encode_utf8(&mut buf);

        for val in buf.into_iter().take(data.len_utf8()) {
            self.uart_send(val)
        }
    }

    /// Put a string to the uart communication
    pub fn puts(&self, data: &str) {
        for ch in data.chars() {
            self.putc(ch);
        }
    }

    /// Wait for a char from the uart
    ///
    /// Any char that is more than one byte of representation is ignored
    /// and will return None
    pub fn getc(&self) -> Option<char> {
        let next = self.uart_recieve();
        next.is_ascii()
            .then(|| char::from_u32(next.into()))
            .flatten()
    }

    /// Try to retrieve a char from the interrupt buffer
    ///
    /// Any char that is more than one byte of representation is ignored
    /// and will return None
    pub fn tryc(&mut self) -> Option<char> {
        let next = cortex_m::interrupt::free(|cs| {
            let mut buf = UARTBUF.borrow_mut(cs);
            buf.pop_front()
        });
        next.and_then(|b| {
            b.is_ascii()
                .then(|| char::from_u32(b.into()))
                .flatten()
        })
    }

    pub fn iter_blocking(&self) -> BlockingIter<'_> {
        self.disable_buffer();
        BlockingIter(self)
    }

    pub fn iter_buffer(&mut self) -> BufferIter<'_> {
        self.enable_buffer();
        BufferIter(self)
    }
}

/// The blocking iterator will block on each call to next
/// waiting for a new char to enter the uart it will
/// return None if the uart recieves a byte that is not
/// a valid char
pub struct BlockingIter<'a>(&'a UartCom);

impl<'a> Iterator for BlockingIter<'a> {
    type Item = char;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.getc()
    }
}

/// The buffer iterator will return Some while the buffer has chars
/// And it will then return None, but it may eventually return
/// Some again when more data enters the buffer
pub struct BufferIter<'a>(&'a mut UartCom);

impl<'a> Iterator for BufferIter<'a> {
    type Item = char;

    fn next(&mut self) -> Option<Self::Item> {
        self.0.tryc()
    }
}

#[interrupt]
fn UART1() {
    let cy = get_cybot();
    cortex_m::interrupt::free(|cs| {
        let uart1 = cy.uart1.borrow(cs);
        // if there was an interrupt on read
        if uart1.mis.read().rxmis().bit() {
            let val = uart1.dr.read().data().bits();
            let mut buf = UARTBUF.borrow_mut(cs);
            buf.push_back(val);

            // clear the interrupt
            uart1.icr.write(|w| w.rxic().set_bit());
        }
    })
}
