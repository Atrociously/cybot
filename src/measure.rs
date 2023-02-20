use libm::fabsf;

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Distance {
    cm: f32,
}

impl Distance {
    pub const fn from_cm(cm: f32) -> Self {
        Self { cm }
    }

    pub fn from_mm(mm: f32) -> Self {
        Self { cm: mm / 10.0 }
    }

    pub fn from_nm(nm: f32) -> Self {
        Self { cm: nm / 1e7 }
    }

    pub const fn as_cm(&self) -> f32 {
        self.cm
    }

    pub fn as_mm(&self) -> f32 {
        self.cm * 10.0
    }

    pub fn as_nm(&self) -> f32 {
       self.cm * 1e7
    }

    pub fn magnitude(&self) -> Self {
        Self::from_cm(fabsf(self.cm))
    }
}

#[repr(transparent)]
#[derive(Clone, Copy, Debug, Default, PartialEq, PartialOrd)]
pub struct Angle {
    rad: f32,
}

impl Angle {
    pub const fn from_rad(rad: f32) -> Self {
        Self { rad }
    }

    pub fn from_deg(deg: f32) -> Self {
        Self {
            rad: deg * (core::f32::consts::PI / 180.0),
        }
    }

    pub const fn as_rad(&self) -> f32 {
        self.rad
    }

    pub fn as_deg(&self) -> f32 {
        self.rad * (180.0 / core::f32::consts::PI)
    }

    pub fn magnitude(&self) -> Self {
        Self::from_rad(fabsf(self.rad))
    }
}

macro_rules! impl_ops {
    ($name:ident::$field:ident) => {
        impl ::core::ops::Add for $name {
            type Output = Self;
            fn add(self, rhs: Self) -> Self::Output {
                Self {
                    $field: self.$field + rhs.$field,
                }
            }
        }
        impl ::core::ops::Sub for $name {
            type Output = Self;
            fn sub(self, rhs: Self) -> Self::Output {
                Self {
                    $field: self.$field - rhs.$field,
                }
            }
        }
        impl ::core::ops::Mul for $name {
            type Output = Self;
            fn mul(self, rhs: Self) -> Self::Output {
                Self {
                    $field: self.$field * rhs.$field,
                }
            }
        }
        impl ::core::ops::Div for $name {
            type Output = Self;
            fn div(self, rhs: Self) -> Self::Output {
                Self {
                    $field: self.$field / rhs.$field,
                }
            }
        }
        impl ::core::ops::AddAssign for $name {
            fn add_assign(&mut self, rhs: Self) {
                self.$field += rhs.$field;
            }
        }
        impl ::core::ops::SubAssign for $name {
            fn sub_assign(&mut self, rhs: Self) {
                self.$field -= rhs.$field;
            }
        }
        impl ::core::ops::MulAssign for $name {
            fn mul_assign(&mut self, rhs: Self) {
                self.$field *= rhs.$field;
            }
        }
        impl ::core::ops::DivAssign for $name {
            fn div_assign(&mut self, rhs: Self) {
                self.$field /= rhs.$field;
            }
        }
    };
}

impl_ops!(Distance::cm);
impl_ops!(Angle::rad);
