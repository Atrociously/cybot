#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum OiMode {
    #[default]
    Off = 0,
    Passive = 1,
    Safe = 2,
    Full = 3,
}

impl From<u8> for OiMode {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::Off,
            1 => Self::Passive,
            2 => Self::Safe,
            3 => Self::Full,
            _ => Self::default(),
        }
    }
}
