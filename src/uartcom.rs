use tm4c123x_hal::tm4c123x::UART1;
use crate::{bits::*, CyBot};

const MAX_UNICODE_CHAR: usize = 4;

pub struct UartCom<'a> {
    uart: &'a UART1,
}

static mut UARTCOM: Option<()> = Some(());

impl<'a> UartCom<'a> {
    pub fn take(cybot: &'a CyBot) -> Option<Self> {
        cortex_m::interrupt::free(|_| unsafe { UARTCOM.take() })?;

        let sysctl = &cybot.peripherals.SYSCTL;
        let gpio = &cybot.peripherals.GPIO_PORTB;
        let uart = &cybot.peripherals.UART1;

        sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit()); // enable gpio b
        sysctl.rcgcuart.modify(|_, w| w.r1().set_bit()); // enable uart 1

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

        uart.ctl
            .modify(|_, w| w.rxe().set_bit().txe().set_bit().uarten().set_bit());
        Some(Self { uart })
    }

    fn uart_send(&self, data: u8) {
        // holds until no data in transmit buffer
        while self.uart.fr.read().txff().bit() {}

        self.uart.dr.write(|w| unsafe { w.bits(u32::from(data)) })
    }

    fn uart_recieve(&self) -> u8 {
        // hold until data is recieved
        while self.uart.fr.read().rxfe().bit() {}

        self.uart.dr.read().data().bits()
    }

    pub fn putc(&self, data: char) {
        let mut buf = [0; MAX_UNICODE_CHAR];
        data.encode_utf8(&mut buf);

        for val in buf.into_iter().take(data.len_utf8()) {
            self.uart_send(val)
        }
    }

    pub fn puts(&self, data: &str) {
        for ch in data.chars() {
            self.putc(ch);
        }
    }

    pub fn getc(&self) -> Option<char> {
        let next = self.uart_recieve();
        if next.is_ascii() {
            char::from_u32(next.into())
        } else {
            None
        }
    }
}
