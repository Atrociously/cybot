#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Source(u8);

impl Source {
    pub const INTERNAL: Source = Source(0x01);
    pub const HOME: Source = Source(0x02);

    pub fn internal(self) -> bool {
        self.0 & Self::INTERNAL.0 == 1
    }

    pub fn home(self) -> bool {
        self.0 & Self::HOME.0 >> 1 == 1
    }
}

impl From<u8> for Source {
    fn from(value: u8) -> Self {
        Self(value)
    }
}
