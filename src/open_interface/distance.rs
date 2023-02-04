#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Millimeter(pub i16);

pub trait IntoMillimeter {
    fn into_mm(value: Self) -> Millimeter;
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Centimeter(pub i16);

impl IntoMillimeter for Centimeter {
    fn into_mm(value: Self) -> Millimeter {
        Millimeter(value.0 * 10)
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Meter(pub i8);

impl IntoMillimeter for Meter {
    fn into_mm(value: Self) -> Millimeter {
        Millimeter(i16::from(value.0) * 1_000)
    }
}

macro_rules! generic_impl {
    ($($ty:ty),+) => {
        $(
        impl IntoMillimeter for $ty {
            fn into_mm(value: Self) -> Millimeter {
                Millimeter(i16::from(value))
            }
        }
        )+
    };
}
generic_impl!(u8, i8, i16);
