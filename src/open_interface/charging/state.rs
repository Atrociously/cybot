#[repr(u8)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum State {
    #[default]
    NotCharging = 0,
    RecoditioningCharging = 1,
    FullCharging = 2,
    TrickleCharging = 3,
    Waiting = 4,
    ChargingFault = 5,
}

impl From<u8> for State {
    fn from(value: u8) -> Self {
        match value {
            0 => Self::NotCharging,
            1 => Self::RecoditioningCharging,
            2 => Self::FullCharging,
            3 => Self::TrickleCharging,
            4 => Self::Waiting,
            5 => Self::ChargingFault,
            _ => Self::default(),
        }
    }
}
