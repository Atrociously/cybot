use tm4c123x_hal::tm4c123x::{GPIO_PORTD, GPIO_PORTF};

use crate::{time::SpinTimer, CyBot};

const HD_LCD_CLEAR: u8 = 0x01;
const HD_RETURN_HOME: u8 = 0x02;

const LCD_DDRAM_WRITE: u8 = 0x80;
#[allow(dead_code)]
const LCD_CGRAM_WRITE: u8 = 0x40;

const EN_PIN: u32 = 0x04;
const RS_PIN: u32 = 0x08;
const RW_PIN: u32 = 0x40;

const LINE_ADDRESSES: [u8; 4] = [0x00, 0x40, 0x14, 0x54];

static mut LCD: Option<()> = Some(());

pub struct Lcd<'a> {
    control: &'a GPIO_PORTD,
    data: &'a GPIO_PORTF,
}

impl<'a> Lcd<'a> {
    pub const WIDTH: u8 = 20;
    pub const HEIGHT: u8 = 4;
    pub const TOTAL_CHARS: u8 = Self::WIDTH * Self::HEIGHT;

    /// Initialize and take the lcd instance
    ///
    /// Guaranteed to return Some on first call, all subsequent calls return None
    pub fn take(cybot: &'a CyBot) -> Option<Self> {
        cortex_m::interrupt::free(|_| unsafe { LCD.take() })?;

        let control = &cybot.peripherals.GPIO_PORTD;
        let data = &cybot.peripherals.GPIO_PORTF;

        let mut new = Self { control, data };
        new.init(cybot);
        Some(new)
    }

    fn init(&mut self, cybot: &CyBot) {
        let sysctl = &cybot.peripherals.SYSCTL;

        sysctl
            .rcgcgpio
            .modify(|_, w| w.r3().set_bit().r5().set_bit());

        self.data.dir.write(|w| unsafe { w.bits(0x1E) });
        self.data.den.write(|w| unsafe { w.bits(0x1E) });

        self.control
            .dir
            .write(|w| unsafe { w.bits(EN_PIN | RS_PIN | RW_PIN) });
        self.control
            .den
            .write(|w| unsafe { w.bits(EN_PIN | RS_PIN | RW_PIN) });

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
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN) });
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
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN) });
        self.data
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() | ((nibble & 0x0F) << 1)) });

        SpinTimer.wait_micros(20);
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !EN_PIN) });
        SpinTimer.wait_micros(20);

        self.data
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !(0x0F << 1)) });
    }

    /// Clear the lcd screen
    ///
    /// Also returns the cursor
    pub fn clear(&mut self) {
        self.send_command(HD_LCD_CLEAR);
        SpinTimer.wait_millis(2);
    }

    /// Return the cursor home without clearing
    pub fn home(&mut self) {
        self.send_command(HD_RETURN_HOME);
    }

    /// Go to a specific line in the lcd
    ///
    /// lines are zero indexed so the first line is line 0
    /// there are 4 total lines so the max value is 3
    pub fn goto_line(&mut self, line: u8) {
        let line: usize = line.into();
        let line = 0x03 & (line - 1);
        self.send_command(LCD_DDRAM_WRITE | LINE_ADDRESSES[line]);
    }


    /// Set the exact x and y value of the cursor
    ///
    /// x must be less than [`Self::WIDTH`]
    /// y must be less than [`Self::HEIGHT`]
    /// the function will do nothing if your bounds are incorrect
    pub fn set_cursor_pos(&mut self, x: u8, y: u8) {
        if x >= Self::WIDTH || y >= Self::HEIGHT {
            return;
        }

        let index: u8 = LINE_ADDRESSES[usize::from(y)] + x;
        self.send_command(0x80 | index);
    }


    /// Put a character on the lcd screen
    ///
    /// Will put the given char onto the lcd
    /// the char must be encodable as a u8
    /// if it is not the function will do nothing.
    pub fn putc(&mut self, data: char) {
        let Ok(data) = u8::try_from(data) else {
            return;
        };

        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() | RS_PIN) });
        self.control
            .data
            .modify(|r, w| unsafe { w.bits(r.bits() & !RW_PIN) });

        self.send_nibble(data >> 4);
        SpinTimer.wait_micros(43);
        self.send_nibble(data & 0x0F);

        // TODO: Poll busy flag
    }

    /// Put a string onto the lcd screen
    ///
    /// Every char in the string must be encodable as a u8
    pub fn puts(&mut self, data: &str) {
        for ch in data.chars() {
            self.putc(ch);
        }
    }
}

impl<'a> core::fmt::Write for Lcd<'a> {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.puts(s);
        Ok(())
    }
}
