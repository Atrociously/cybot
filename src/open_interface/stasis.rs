#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Stasis(u8);

impl Stasis {
    pub const TOGGLING: Stasis = Stasis(0x01);
    pub const DISABLED: Stasis = Stasis(0x02);

    pub fn moving_foreward(self) -> bool {
        self.0 & Self::TOGGLING.0 == 1
    }

    pub fn disabled(self) -> bool {
        self.0 & Self::DISABLED.0 >> 1 == 1
    }
}

impl From<u8> for Stasis {
    fn from(value: u8) -> Self {
        Self(value)
    }
}
