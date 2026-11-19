//! # physics::solver — Numerical ODE solver for BLDC motor electromechanics
//!
//! Integrates coupled stator electromagnetic differential equations in the rotating $d$-$q$
//! frame alongside Newton's second law for mechanical rotor rotation.
//!
//! Stiff systems with small electrical time constants ($\tau = L / R \approx 10\text{--}30\,\mu\text{s}$)
//! are handled robustly via sub-stepping ($\Delta t_{\text{phys}} \le 2\,\mu\text{s}$) and 4th-order
//! Runge-Kutta (RK4) integration.

use crate::physics::motor::{MotorParams, MotorState};
use barefoc::math::_1_SQRT3;

/// Numerical integrator simulating the electromechanical dynamics of a BLDC motor.
#[derive(Clone, Copy, Debug)]
pub struct MotorSolver {
    params: MotorParams,
    state: MotorState,
    max_substep_dt: f32,
}

impl MotorSolver {
    /// Create a new solver with the given motor parameters and default sub-step ($\le 1.0\,\mu\text{s}$).
    pub fn new(params: MotorParams) -> Self {
        Self {
            params,
            state: MotorState::new(),
            max_substep_dt: 1.0e-6,
        }
    }

    /// Create a solver with a customized maximum physics sub-step duration.
    pub fn with_substep(params: MotorParams, max_substep_dt: f32) -> Self {
        Self {
            params,
            state: MotorState::new(),
            max_substep_dt,
        }
    }

    /// Current dynamic state of the motor.
    #[inline]
    pub fn state(&self) -> &MotorState {
        &self.state
    }

    /// Mutable reference to dynamic state (e.g. to set initial rotor position).
    #[inline]
    pub fn state_mut(&mut self) -> &mut MotorState {
        &mut self.state
    }

    /// Motor configuration parameters.
    #[inline]
    pub fn params(&self) -> &MotorParams {
        &self.params
    }

    /// Maximum physics integration sub-step duration ($\Delta t_{\text{phys}}$).
    #[inline]
    pub fn max_substep_dt(&self) -> f32 {
        self.max_substep_dt
    }

    /// Set maximum physics integration sub-step duration.
    #[inline]
    pub fn set_max_substep_dt(&mut self, dt: f32) {
        self.max_substep_dt = dt;
    }

    /// Evaluates dynamic state derivatives $[dI_d, dI_q, d\omega_m, d\theta_m]$.
    #[inline]
    fn derivatives(
        params: &MotorParams,
        state: &[f32; 4], // [i_d, i_q, omega_m, theta_m]
        v_d: f32,
        v_q: f32,
        tau_load: f32,
    ) -> [f32; 4] {
        let i_d = state[0];
        let i_q = state[1];
        let omega_m = state[2];
        let pp = params.pole_pairs as f32;
        let omega_e = pp * omega_m;

        // Stator voltage balance equations in d-q frame:
        // dId/dt = (Vd - Rs*Id + we*Lq*Iq) / Ld
        // dIq/dt = (Vq - Rs*Iq - we*Ld*Id - we*lambda_m) / Lq
        let d_id = (v_d - params.r_phase * i_d + omega_e * params.l_q * i_q) / params.l_d;
        let d_iq = (v_q - params.r_phase * i_q - omega_e * params.l_d * i_d
            - omega_e * params.flux_linkage)
            / params.l_q;

        // Electromagnetic torque:
        // tau_e = 1.5 * PP * [lambda_m * Iq + (Ld - Lq) * Id * Iq]
        let tau_e = 1.5 * pp * (params.flux_linkage * i_q + (params.l_d - params.l_q) * i_d * i_q);

        // Friction: viscous damping + smooth Coulomb friction
        let tau_friction = params.viscous_damping * omega_m
            + params.coulomb_friction * libm::tanhf(10.0 * omega_m);

        // Mechanical acceleration:
        // d(omega_m)/dt = (tau_e - tau_load - tau_friction) / J
        let d_omega_m = (tau_e - tau_load - tau_friction) / params.inertia;
        let d_theta_m = omega_m;

        [d_id, d_iq, d_omega_m, d_theta_m]
    }

    /// Executes a single RK4 integration sub-step of duration `dt`.
    #[inline]
    fn rk4_step(&mut self, v_d: f32, v_q: f32, tau_load: f32, dt: f32) {
        let y0 = [
            self.state.i_d,
            self.state.i_q,
            self.state.shaft_velocity,
            self.state.shaft_angle,
        ];

        let k1 = Self::derivatives(&self.params, &y0, v_d, v_q, tau_load);

        let mut y1 = [0.0f32; 4];
        for i in 0..4 {
            y1[i] = y0[i] + 0.5 * dt * k1[i];
        }
        let k2 = Self::derivatives(&self.params, &y1, v_d, v_q, tau_load);

        let mut y2 = [0.0f32; 4];
        for i in 0..4 {
            y2[i] = y0[i] + 0.5 * dt * k2[i];
        }
        let k3 = Self::derivatives(&self.params, &y2, v_d, v_q, tau_load);

        let mut y3 = [0.0f32; 4];
        for i in 0..4 {
            y3[i] = y0[i] + dt * k3[i];
        }
        let k4 = Self::derivatives(&self.params, &y3, v_d, v_q, tau_load);

        self.state.i_d = y0[0] + (dt / 6.0) * (k1[0] + 2.0 * k2[0] + 2.0 * k3[0] + k4[0]);
        self.state.i_q = y0[1] + (dt / 6.0) * (k1[1] + 2.0 * k2[1] + 2.0 * k3[1] + k4[1]);
        self.state.shaft_velocity =
            y0[2] + (dt / 6.0) * (k1[2] + 2.0 * k2[2] + 2.0 * k3[2] + k4[2]);
        self.state.shaft_angle = y0[3] + (dt / 6.0) * (k1[3] + 2.0 * k2[3] + 2.0 * k3[3] + k4[3]);

        let pp = self.params.pole_pairs as f32;
        self.state.electrical_torque = 1.5
            * pp
            * (self.params.flux_linkage * self.state.i_q
                + (self.params.l_d - self.params.l_q) * self.state.i_d * self.state.i_q);
    }

    /// Advance the motor simulation over an interval `dt_total` with commanded phase voltages $(U_a, U_b, U_c)$.
    ///
    /// Automatically divides `dt_total` into $N$ sub-steps of $\Delta t \le \text{max\_substep\_dt}$
    /// and projects applied phase voltages into the rotating $d$-$q$ frame at each sub-step.
    pub fn step(&mut self, ua: f32, ub: f32, uc: f32, tau_load: f32, dt_total: f32) {
        if dt_total <= 0.0 {
            return;
        }

        // Sub-stepping count calculation:
        let n_substeps = libm::ceilf(dt_total / self.max_substep_dt) as usize;
        let n_substeps = if n_substeps == 0 { 1 } else { n_substeps };
        let dt_sub = dt_total / (n_substeps as f32);

        // Three-phase neutral voltage:
        let v_neutral = (ua + ub + uc) / 3.0;
        let v_alpha = ua - v_neutral;
        let v_beta = (ub - uc) * _1_SQRT3;

        for _ in 0..n_substeps {
            // Re-project into rotor d-q reference frame with updated electrical angle:
            let angle_el = self.state.electrical_angle(self.params.pole_pairs);
            let s = libm::sinf(angle_el);
            let c = libm::cosf(angle_el);

            let v_d = v_alpha * c + v_beta * s;
            let v_q = v_beta * c - v_alpha * s;

            self.rk4_step(v_d, v_q, tau_load, dt_sub);
        }
    }

    /// Direct $d$-$q$ voltage stepping without three-phase modulation.
    pub fn step_dq(&mut self, v_d: f32, v_q: f32, tau_load: f32, dt_total: f32) {
        if dt_total <= 0.0 {
            return;
        }

        let n_substeps = libm::ceilf(dt_total / self.max_substep_dt) as usize;
        let n_substeps = if n_substeps == 0 { 1 } else { n_substeps };
        let dt_sub = dt_total / (n_substeps as f32);

        for _ in 0..n_substeps {
            self.rk4_step(v_d, v_q, tau_load, dt_sub);
        }
    }
}
