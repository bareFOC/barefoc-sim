//! # adapters::current_sense — Simulated phase current sensing
//!
//! Implements the [`CurrentSense`] trait, translating rotating $d$-$q$ currents
//! from the physics solver into stationary $(I_\alpha, I_\beta)$ and 3-phase currents $(I_a, I_b, I_c)$.

use core::cell::Cell;
use barefoc::math::{_SQRT3_2, PhaseCurrent};
use barefoc::traits::current_sense::{CurrentSense, CurrentSenseBase};
use barefoc::traits::driver::DriverType;

/// Simulated three-phase inline / low-side current sense.
pub struct SimulatedCurrentSense {
    base: CurrentSenseBase,
    ia: Cell<f32>,
    ib: Cell<f32>,
    ic: Cell<f32>,
}

impl Default for SimulatedCurrentSense {
    fn default() -> Self {
        Self::new()
    }
}

impl SimulatedCurrentSense {
    /// Construct a new simulated current sense initialized and ready for FOC current control.
    pub fn new() -> Self {
        let mut base = CurrentSenseBase::new();
        base.skip_align = true;
        base.initialized = true;
        base.driver_type = DriverType::BLDC;

        Self {
            base,
            ia: Cell::new(0.0),
            ib: Cell::new(0.0),
            ic: Cell::new(0.0),
        }
    }

    /// Set three-phase currents directly.
    #[inline]
    pub fn set_phase_currents(&self, ia: f32, ib: f32, ic: f32) {
        self.ia.set(ia);
        self.ib.set(ib);
        self.ic.set(ic);
    }

    /// Set simulated currents from rotating $d$-$q$ stator currents at given electrical angle $\theta_e$.
    #[inline]
    pub fn set_dq(&self, id: f32, iq: f32, angle_el: f32) {
        let s = libm::sinf(angle_el);
        let c = libm::cosf(angle_el);

        // Inverse Park:
        let i_alpha = id * c - iq * s;
        let i_beta = id * s + iq * c;

        // Inverse Clarke:
        let i_a = i_alpha;
        let i_b = -0.5 * i_alpha + _SQRT3_2 * i_beta;
        let i_c = -0.5 * i_alpha - _SQRT3_2 * i_beta;

        self.ia.set(i_a);
        self.ib.set(i_b);
        self.ic.set(i_c);
    }
}

impl CurrentSense for SimulatedCurrentSense {
    type Error = core::convert::Infallible;

    fn base(&self) -> &CurrentSenseBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut CurrentSenseBase {
        &mut self.base
    }

    fn init(&mut self) -> Result<(), Self::Error> {
        self.base.initialized = true;
        Ok(())
    }

    fn get_phase_currents(&mut self) -> PhaseCurrent {
        PhaseCurrent {
            a: self.ia.get(),
            b: self.ib.get(),
            c: self.ic.get(),
        }
    }

    fn driver_align(&mut self, _voltage: f32, _modulation_centered: bool) -> i32 {
        1 // Successful alignment
    }
}
