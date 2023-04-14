use crate::{get_cybot, bits::*, measure::Distance};

static mut IR: Option<IrSensor> = Some(IrSensor(()));

pub struct IrSensor(());

impl IrSensor {
    pub const VREFP: f32 = 3.3;
    pub const VREFN: f32 = 0.0;
    pub const MAX_RAW: u16 = 4096;

    pub fn take() -> Option<Self> {
        let cy = get_cybot();
        cortex_m::interrupt::free(|cs| {
            let sensor = unsafe { IR.take() }?;
            let sysctl = cy.sysctl.borrow(cs);
            let gpiob = cy.gpiob.borrow(cs);
            let adc0 = cy.adc0.borrow(cs);

            // ensure that adc0 and gpiob are enabled
            sysctl.rcgcgpio.modify(|_, w| w.r1().set_bit());
            sysctl.rcgcadc.modify(|_, w| w.r0().set_bit());

            // wait for ready
            while sysctl.prgpio.read().r1().bit_is_clear() {}
            while sysctl.pradc.read().r0().bit_is_clear() {}

            // enable the gpio to use analog input on pin 4
            gpiob.amsel.modify(|r, w| unsafe { w.bits(r.bits() | BIT4) });
            gpiob.den.modify(|r, w| unsafe { w.bits(r.bits() | BIT4) });
            gpiob.dir.modify(|r, w| unsafe { w.bits(r.bits() & !BIT4) });

            adc0.cc.modify(|_, w| w.cs().syspll());
            adc0.ssctl3.modify(|_, w| w.ie0().set_bit());
            adc0.im.modify(|_, w| w.mask3().clear_bit());
            adc0.actss.modify(|_, w| w.asen3().set_bit());
            adc0.emux.modify(|_, w| w.em3().processor());
            adc0.ssmux3.modify(|_, w| unsafe { w.mux0().bits(10) });
            Some(sensor)
        })
    }

    pub fn get_raw_value(&self) -> u16 {
        let cy = get_cybot();
        cortex_m::interrupt::free(|cs| {
            let adc0 = cy.adc0.borrow(cs);
            adc0.pssi.modify(|_, w| w.ss3().set_bit()); // trigger sampling
            while adc0.ris.read().inr3().bit_is_clear() {} // wait for interrupt to signal finished
            let result = adc0.ssfifo3.read().bits() & 0x00000FFF;
            adc0.isc.modify(|_, w| w.in3().set_bit());
            result as u16
        })
    }

    pub fn get_voltage(&self) -> f32 {
        let factor = (Self::VREFP - Self::VREFN) / f32::from(Self::MAX_RAW);
        factor * f32::from(self.get_raw_value())
    }

    pub fn get_distance(&self) -> Distance {
        const MAGIC_N: f32 = -1.2006; // -1.218
        const MAGIC_M: f32 = 134_078.23; // 5.8855

        let raw_value = f32::from(self.get_raw_value());
        let cm = MAGIC_M * libm::powf(raw_value, MAGIC_N);
        Distance::from_cm(cm)
    }
}
