//! # physics::motor — BLDC motor electromechanical state and parameter specifications
//!
//! Models a 3-phase Permanent Magnet Synchronous Motor (PMSM / BLDC) in the
//! rotor $d$-$q$ reference frame and continuous mechanical rotor domain.
//!
//! ## Mathematical Model
//!
//! - Stator flux linkage: $\lambda_m = \frac{60}{2\pi \cdot \sqrt{3} \cdot PP \cdot KV}$
//! - Electrical torque: $\tau_e = \frac{3}{2} \cdot PP \cdot [\lambda_m \cdot I_q + (L_d - L_q) I_d I_q]$
//! - Rotor mechanical acceleration: $\frac{d\omega_m}{dt} = \frac{1}{J} (\tau_e - \tau_{\text{load}} - B \omega_m - \tau_{\text{coulomb}} \operatorname{sgn}(\omega_m))$
//! - Rotor mechanical position: $\frac{d\theta_m}{dt} = \omega_m$

use barefoc::math::{self, _SQRT3_2, PhaseCurrent};

/// Electrical and mechanical parameters defining a BLDC motor profile.
#[derive(Clone, Copy, Debug)]
pub struct MotorParams {
    /// Number of rotor magnetic pole pairs.
    pub pole_pairs: i32,
    /// Stator phase resistance in Ohms ($\Omega$).
    pub r_phase: f32,
    /// Direct-axis inductance in Henries (H).
    pub l_d: f32,
    /// Quadrature-axis inductance in Henries (H).
    pub l_q: f32,
    /// Permanent magnet rotor flux linkage in Webers (Wb or $\text{V}\cdot\text{s}/\text{rad}$).
    pub flux_linkage: f32,
    /// Combined rotor and shaft moment of inertia in $\text{kg}\cdot\text{m}^2$.
    pub inertia: f32,
    /// Viscous damping coefficient in $\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$.
    pub viscous_damping: f32,
    /// Coulomb friction torque in $\text{N}\cdot\text{m}$.
    pub coulomb_friction: f32,
    /// Motor velocity rating in RPM / Volt.
    pub kv_rating: f32,
}

impl MotorParams {
    /// Construct motor parameters by explicitly specifying flux linkage.
    pub fn new(
        pole_pairs: i32,
        r_phase: f32,
        l_phase: f32,
        flux_linkage: f32,
        inertia: f32,
        viscous_damping: f32,
    ) -> Self {
        let kv = if flux_linkage > 1e-9 {
            60.0 / (2.0 * math::_PI * math::_SQRT3 * (pole_pairs as f32) * flux_linkage)
        } else {
            0.0
        };

        Self {
            pole_pairs,
            r_phase,
            l_d: l_phase,
            l_q: l_phase,
            flux_linkage,
            inertia,
            viscous_damping,
            coulomb_friction: 0.0,
            kv_rating: kv,
        }
    }

    /// Construct motor parameters from standard motor datasheet specifications ($KV$ rating).
    ///
    /// Computes flux linkage using the canonical formula:
    /// $$\lambda_m = \frac{60}{2\pi \cdot \sqrt{3} \cdot PP \cdot KV}$$
    pub fn from_kv(
        pole_pairs: i32,
        r_phase: f32,
        kv_rating: f32,
        l_phase: f32,
        inertia: f32,
        viscous_damping: f32,
    ) -> Self {
        let flux_linkage =
            60.0 / (2.0 * math::_PI * math::_SQRT3 * (pole_pairs as f32) * kv_rating);

        Self {
            pole_pairs,
            r_phase,
            l_d: l_phase,
            l_q: l_phase,
            flux_linkage,
            inertia,
            viscous_damping,
            coulomb_friction: 0.0,
            kv_rating,
        }
    }

    /// Preconfigured profile for the Miniata 1104 4300KV micro BLDC motor.
    ///
    /// - Pole pairs: 7
    /// - Resistance: $0.55\,\Omega$
    /// - Inductance: $15\,\mu\text{H}$ ($L/R \approx 27\,\mu\text{s}$)
    /// - KV: 4300 RPM/V
    /// - Inertia: $1.2 \times 10^{-6}\,\text{kg}\cdot\text{m}^2$
    /// - Damping: $1.0 \times 10^{-6}\,\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$
    pub fn miniata_1104_4300kv() -> Self {
        Self::from_kv(
            7,
            0.55,
            4300.0,
            15.0e-6,
            1.2e-6,
            1.0e-6,
        )
    }

    /// Preconfigured profile for a standard 2205 2300KV drone / multicopter motor.
    ///
    /// - Pole pairs: 7
    /// - Resistance: $0.08\,\Omega$
    /// - Inductance: $25\,\mu\text{H}$
    /// - KV: 2300 RPM/V
    /// - Inertia: $8.0 \times 10^{-6}\,\text{kg}\cdot\text{m}^2$
    /// - Damping: $5.0 \times 10^{-6}\,\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$
    pub fn generic_drone_2205() -> Self {
        Self::from_kv(
            7,
            0.08,
            2300.0,
            25.0e-6,
            8.0e-6,
            5.0e-6,
        )
    }

    /// Preconfigured profile for a high-resistance gimbal motor (2804 140KV).
    ///
    /// - Pole pairs: 7
    /// - Resistance: $10.0\,\Omega$
    /// - Inductance: $1.5\,\text{mH}$
    /// - KV: 140 RPM/V
    /// - Inertia: $1.5 \times 10^{-5}\,\text{kg}\cdot\text{m}^2$
    /// - Damping: $2.0 \times 10^{-5}\,\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$
    pub fn generic_gimbal_2804() -> Self {
        Self::from_kv(
            7,
            10.0,
            140.0,
            1.5e-3,
            1.5e-5,
            2.0e-5,
        )
    }

    /// Back-EMF peak line-to-neutral voltage at given mechanical velocity $\omega_m$.
    #[inline]
    pub fn bemf_voltage(&self, velocity: f32) -> f32 {
        (self.pole_pairs as f32) * velocity * self.flux_linkage
    }

    /// Torque constant $K_t$ in $\text{N}\cdot\text{m}/\text{A}$.
    #[inline]
    pub fn torque_constant(&self) -> f32 {
        1.5 * (self.pole_pairs as f32) * self.flux_linkage
    }
}

/// Instantaneous dynamic state of the simulated BLDC motor.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorState {
    /// Direct-axis stator current in Amperes ($I_d$).
    pub i_d: f32,
    /// Quadrature-axis stator current in Amperes ($I_q$).
    pub i_q: f32,
    /// Continuous unwrapped mechanical shaft angle in radians ($\theta_m$).
    pub shaft_angle: f32,
    /// Mechanical shaft angular velocity in radians / second ($\omega_m$).
    pub shaft_velocity: f32,
    /// Instantaneous electromagnetic torque generated in $\text{N}\cdot\text{m}$ ($\tau_e$).
    pub electrical_torque: f32,
}

impl MotorState {
    /// Creates a zeroed initial motor state.
    pub const fn new() -> Self {
        Self {
            i_d: 0.0,
            i_q: 0.0,
            shaft_angle: 0.0,
            shaft_velocity: 0.0,
            electrical_torque: 0.0,
        }
    }

    /// Mechanical shaft angle wrapped to $[0, 2\pi)$ radians.
    #[inline]
    pub fn mechanical_angle(&self) -> f32 {
        math::normalize_angle(self.shaft_angle)
    }

    /// Electrical rotor angle wrapped to $[0, 2\pi)$ radians.
    #[inline]
    pub fn electrical_angle(&self, pole_pairs: i32) -> f32 {
        math::normalize_angle(math::electrical_angle(self.shaft_angle, pole_pairs))
    }

    /// Electrical rotor angular velocity in radians / second.
    #[inline]
    pub fn electrical_velocity(&self, pole_pairs: i32) -> f32 {
        self.shaft_velocity * (pole_pairs as f32)
    }

    /// Stationary $\alpha$-$\beta$ stator currents matching current electrical rotor position.
    #[inline]
    pub fn clarke_currents(&self, pole_pairs: i32) -> (f32, f32) {
        let angle_el = self.electrical_angle(pole_pairs);
        let s = libm::sinf(angle_el);
        let c = libm::cosf(angle_el);
        let i_alpha = self.i_d * c - self.i_q * s;
        let i_beta = self.i_d * s + self.i_q * c;
        (i_alpha, i_beta)
    }

    /// Three-phase stator currents $(I_a, I_b, I_c)$ matching current electrical rotor position.
    #[inline]
    pub fn phase_currents(&self, pole_pairs: i32) -> PhaseCurrent {
        let (i_alpha, i_beta) = self.clarke_currents(pole_pairs);
        let i_a = i_alpha;
        let i_b = -0.5 * i_alpha + _SQRT3_2 * i_beta;
        let i_c = -0.5 * i_alpha - _SQRT3_2 * i_beta;
        PhaseCurrent { a: i_a, b: i_b, c: i_c }
    }
}
