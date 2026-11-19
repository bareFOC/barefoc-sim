//! # adapters::driver — Simulated 3-phase BLDC motor driver
//!
//! Implements the [`BLDCDriver`] and [`FOCDriver`] traits, capturing commanded
//! phase voltages $(U_a, U_b, U_c)$ and output states from BareFOC.

use core::cell::Cell;
use barefoc::traits::driver::{BLDCDriver, DriverType, FOCDriver, FocDriverParams, PhaseState};

/// Simulated three-phase BLDC motor driver tracking commanded phase voltages.
pub struct SimulatedBLDCDriver {
    /// Driver configuration and status parameters.
    pub params: FocDriverParams,
    /// Commanded phase A voltage in Volts.
    pub ua: Cell<f32>,
    /// Commanded phase B voltage in Volts.
    pub ub: Cell<f32>,
    /// Commanded phase C voltage in Volts.
    pub uc: Cell<f32>,
    /// Phase A switching state.
    pub sa: Cell<PhaseState>,
    /// Phase B switching state.
    pub sb: Cell<PhaseState>,
    /// Phase C switching state.
    pub sc: Cell<PhaseState>,
    /// Output stage enable status.
    pub enabled: Cell<bool>,
}

impl Default for SimulatedBLDCDriver {
    fn default() -> Self {
        Self::new(12.0, 12.0)
    }
}

impl SimulatedBLDCDriver {
    /// Construct a new simulated BLDC driver with given supply voltage and limit.
    pub fn new(voltage_power_supply: f32, voltage_limit: f32) -> Self {
        let mut params = FocDriverParams::new();
        params.voltage_power_supply = voltage_power_supply;
        params.voltage_limit = voltage_limit;
        params.initialized = true;

        Self {
            params,
            ua: Cell::new(0.0),
            ub: Cell::new(0.0),
            uc: Cell::new(0.0),
            sa: Cell::new(PhaseState::PHASE_ON),
            sb: Cell::new(PhaseState::PHASE_ON),
            sc: Cell::new(PhaseState::PHASE_ON),
            enabled: Cell::new(true),
        }
    }

    /// Read latest effective phase terminal voltages $(U_a, U_b, U_c)$ in Volts.
    ///
    /// Returns `(0.0, 0.0, 0.0)` when driver is disabled (high-impedance).
    #[inline]
    pub fn voltages(&self) -> (f32, f32, f32) {
        if self.enabled.get() {
            (self.ua.get(), self.ub.get(), self.uc.get())
        } else {
            (0.0, 0.0, 0.0)
        }
    }

    /// Whether the output stage is enabled.
    #[inline]
    pub fn is_enabled(&self) -> bool {
        self.enabled.get()
    }
}

impl FOCDriver for SimulatedBLDCDriver {
    type Error = core::convert::Infallible;

    fn init(&mut self) -> Result<(), Self::Error> {
        self.params.initialized = true;
        Ok(())
    }

    fn enable(&mut self) {
        self.enabled.set(true);
    }

    fn disable(&mut self) {
        self.enabled.set(false);
        self.ua.set(0.0);
        self.ub.set(0.0);
        self.uc.set(0.0);
    }

    fn driver_type(&self) -> DriverType {
        DriverType::BLDC
    }

    fn params(&self) -> &FocDriverParams {
        &self.params
    }

    fn params_mut(&mut self) -> &mut FocDriverParams {
        &mut self.params
    }
}

impl BLDCDriver for SimulatedBLDCDriver {
    fn set_pwm(&mut self, ua: f32, ub: f32, uc: f32) {
        self.ua.set(ua);
        self.ub.set(ub);
        self.uc.set(uc);
    }

    fn set_phase_state(&mut self, sa: PhaseState, sb: PhaseState, sc: PhaseState) {
        self.sa.set(sa);
        self.sb.set(sb);
        self.sc.set(sc);
    }
}
