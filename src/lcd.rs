use tm4c123x_hal::tm4c123x::{sysctl::RCGCGPIO, GPIO_PORTD, GPIO_PORTF};

use crate::time::SpinTimer;

const HD_LCD_CLEAR: u8 = 0x01;
const HD_RETURN_HOME: u8 = 0x02;

pub const LCD_WIDTH: u8 = 20;
pub const LCD_HEIGHT: u8 = 4;
pub const LCD_TOTAL_CHARS: u8 = LCD_WIDTH * LCD_HEIGHT;
const LCD_DDRAM_WRITE: u8 = 0x80;
#[allow(dead_code)]
const LCD_CGRAM_WRITE: u8 = 0x40;

const EN_PIN: u32 = 0x04;
const RS_PIN: u32 = 0x08;
const RW_PIN: u32 = 0x40;

const LINE_ADDRESSES: [u8; 4] = [0x00, 0x40, 0x14, 0x54];

pub struct Lcd {
    control: GPIO_PORTD,
    data: GPIO_PORTF,
}

impl Lcd {
    pub(crate) fn new(control: GPIO_PORTD, data: GPIO_PORTF, clocks: &RCGCGPIO) -> Self {
        let mut new = Self { control, data };
        new.init(clocks);
        new
    }

    fn init(&mut self, clocks: &RCGCGPIO) {
        clocks.modify(|_, w| w.r3().set_bit().r5().set_bit());
        self.data.dir.modify(|r, w| unsafe { w.bits(r.bits() | 0x0E) });
        self.data.den.modify(|r, w| unsafe { w.bits(r.bits() | 0x0E) });

        self.control
            .dir
            .modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN | RS_PIN | RW_PIN) });
        self.control
            .den
            .modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN | RS_PIN | RW_PIN) });

        self.control
            .data
            .write_with_zero(|w| unsafe { w.bits(!(EN_PIN | RW_PIN | RS_PIN)) });

        let timer = SpinTimer;

        timer.wait_millis(50);

        self.send_nibble(0x03);
        timer.wait_millis(10);

        self.send_nibble(0x03);
        timer.wait_micros(170);

        self.send_nibble(0x03);
        timer.wait_micros(170);

        self.send_nibble(0x02);
        timer.wait_millis(1);

        self.send_command(0x28);
        timer.wait_millis(1);

        self.send_command(0x0F);
        timer.wait_millis(1);

        self.send_command(0x28);
        timer.wait_millis(1);

        self.send_command(0x06);
        timer.wait_millis(1);

        self.send_command(0x01);
        timer.wait_millis(1);

        self.clear();
        timer.wait_millis(1);
    }

    fn send_command(&mut self, data: u8) {
        self.control.data.modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN) });
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !(RW_PIN | RS_PIN)) });

        self.send_nibble(data >> 4);
        SpinTimer.wait_micros(1);
        self.send_nibble(data & 0x0F);

        // TODO: poll busy flag
        SpinTimer.wait_millis(1);
    }

    fn send_nibble(&mut self, nibble: u8) {
        let nibble: u32 = nibble.into();
        self.control.data.modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN) });
        self.data
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() | (nibble & 0x0F) << 1) });

        SpinTimer.wait_micros(20);
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !EN_PIN) });
        SpinTimer.wait_micros(20);

        self.data
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0x0F << 1)) });
    }

    pub fn clear(&mut self) {
        self.send_command(HD_LCD_CLEAR);
        SpinTimer.wait_millis(2);
    }

    pub fn home(&mut self) {
        self.send_command(HD_RETURN_HOME);
    }

    pub fn goto_line(&mut self, line: u8) {
        let line: usize = line.into();
        let line = 0x03 & (line - 1);
        self.send_command(LCD_DDRAM_WRITE | LINE_ADDRESSES[line]);
    }

    pub fn set_cursor_pos(&mut self, x: u8, y: u8) {
        if x >= LCD_WIDTH || y >= LCD_HEIGHT {
            return;
        }

        let index: u8 = LINE_ADDRESSES[usize::from(y)] + x;
        self.send_command(0x80 | index);
    }

    pub fn putc(&mut self, data: char) {
        let Ok(data) = u8::try_from(data) else {
            return;
        };

        self.control.data.modify(|r, w| unsafe { w.bits(r.bits() | RS_PIN) });
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !RW_PIN) });

        self.send_nibble(data >> 4);
        SpinTimer.wait_micros(43);
        self.send_nibble(data & 0x0F);

        // TODO: Poll busy flag
    }

    pub fn puts(&mut self, data: &str) {
        for ch in data.chars() {
            self.putc(ch);
        }
    }
}

impl core::fmt::Write for Lcd {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.puts(s);
        Ok(())
    }

    fn write_char(&mut self, c: char) -> core::fmt::Result {
        self.putc(c);
        Ok(())
    }
}
