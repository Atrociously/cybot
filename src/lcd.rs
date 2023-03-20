use cortex_m::interrupt::CriticalSection;

use crate::{get_cybot, time::SpinTimer};

const HD_LCD_CLEAR: u8 = 0x01;
const HD_RETURN_HOME: u8 = 0x02;

const LCD_DDRAM_WRITE: u8 = 0x80;
#[allow(dead_code)]
const LCD_CGRAM_WRITE: u8 = 0x40;

const EN_PIN: u32 = 0x04;
const RS_PIN: u32 = 0x08;
const RW_PIN: u32 = 0x40;

const LINE_ADDRESSES: [u8; 4] = [0x00, 0x40, 0x14, 0x54];

static mut LCD: Option<Lcd> = Some(Lcd(()));

pub struct Lcd(());

impl Lcd {
    pub const WIDTH: u8 = 20;
    pub const HEIGHT: u8 = 4;
    pub const TOTAL_CHARS: u8 = Self::WIDTH * Self::HEIGHT;

    /// Initialize and take the lcd instance
    ///
    /// Guaranteed to return Some on first call, all subsequent calls return None
    pub fn take() -> Option<Self> {
        let mut lcd = cortex_m::interrupt::free(|_| unsafe { LCD.take() })?;

        cortex_m::interrupt::free(|cs| {
            lcd.init(cs);
        });
        Some(lcd)
    }

    fn init(&mut self, cs: &CriticalSection) {
        let cybot = get_cybot();

        let sysctl = cybot.sysctl.borrow(cs);
        let control = cybot.gpiod.borrow(cs);
        let data = cybot.gpiof.borrow(cs);

        sysctl
            .rcgcgpio
            .modify(|_, w| w.r3().set_bit().r5().set_bit());

        data.dir.write(|w| unsafe { w.bits(0x1E) });
        data.den.write(|w| unsafe { w.bits(0x1E) });

        control
            .dir
            .write(|w| unsafe { w.bits(EN_PIN | RS_PIN | RW_PIN) });
        control
            .den
            .write(|w| unsafe { w.bits(EN_PIN | RS_PIN | RW_PIN) });

        control
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
        cortex_m::interrupt::free(|cs| {
            let cybot = get_cybot();
            let control = cybot.gpiod.borrow(cs);
            control
                .data
                .modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN) });
            control
                .data
                .modify(|r, w| unsafe { w.bits(r.bits() & !(RW_PIN | RS_PIN)) });
        });

        self.send_nibble(data >> 4);
        SpinTimer.wait_micros(1);
        self.send_nibble(data & 0x0F);

        // TODO: poll busy flag
        SpinTimer.wait_millis(1);
    }

    fn send_nibble(&mut self, nibble: u8) {
        let nibble: u32 = nibble.into();

        cortex_m::interrupt::free(|cs| {
            let cybot = get_cybot();
            let control = cybot.gpiod.borrow(cs);
            let data = cybot.gpiof.borrow(cs);
            control
                .data
                .modify(|r, w| unsafe { w.bits(r.bits() | EN_PIN) });
            data.data
                .modify(|r, w| unsafe { w.bits(r.bits() | ((nibble & 0x0F) << 1)) });

            SpinTimer.wait_micros(20);
            control
                .data
                .modify(|r, w| unsafe { w.bits(r.bits() & !EN_PIN) });
            SpinTimer.wait_micros(20);

            data.data
                .modify(|r, w| unsafe { w.bits(r.bits() & !(0x0F << 1)) });
        });
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

        cortex_m::interrupt::free(|cs| {
            let control = get_cybot().gpiod.borrow(cs);
            control
                .data
                .modify(|r, w| unsafe { w.bits(r.bits() | RS_PIN) });
            control
                .data
                .modify(|r, w| unsafe { w.bits(r.bits() & !RW_PIN) });
        });

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

impl core::fmt::Write for Lcd {
    fn write_str(&mut self, s: &str) -> core::fmt::Result {
        self.puts(s);
        Ok(())
    }
}
