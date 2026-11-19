//! # adapters::sensor — Simulated rotor position and velocity sensor
//!
//! Implements the [`Sensor`] trait, returning simulated mechanical angle $\theta_m$
//! and velocity $\omega_m$ with configurable quantization (e.g. 14-bit).

use core::cell::Cell;
use barefoc::common::time::Clock;
use barefoc::math::{self, _2PI};
use barefoc::traits::sensor::{Sensor, SensorBase};

/// Simulated rotor angle and velocity sensor.
pub struct SimulatedSensor {
    base: SensorBase,
    mechanical_angle: Cell<f32>,
    mechanical_velocity: Cell<f32>,
    quantization_bits: Option<u8>,
    use_exact_velocity: bool,
}

impl Default for SimulatedSensor {
    fn default() -> Self {
        Self::new()
    }
}

impl SimulatedSensor {
    /// Construct a simulated sensor with continuous floating-point resolution and exact velocity.
    pub fn new() -> Self {
        Self {
            base: SensorBase::new(),
            mechanical_angle: Cell::new(0.0),
            mechanical_velocity: Cell::new(0.0),
            quantization_bits: None,
            use_exact_velocity: true,
        }
    }

    /// Construct a simulated sensor with digital quantization matching an $N$-bit encoder (e.g. 14 bits for MT6701/AS5047P).
    pub fn with_quantization(bits: u8) -> Self {
        Self {
            base: SensorBase::new(),
            mechanical_angle: Cell::new(0.0),
            mechanical_velocity: Cell::new(0.0),
            quantization_bits: Some(bits),
            use_exact_velocity: true,
        }
    }

    /// Update the simulated true physical mechanical state from the physics solver.
    #[inline]
    pub fn set_state(&self, angle: f32, velocity: f32) {
        self.mechanical_angle.set(angle);
        self.mechanical_velocity.set(velocity);
    }

    /// Configure whether `get_velocity()` returns the exact physical velocity or discrete numerical derivative.
    pub fn set_use_exact_velocity(&mut self, exact: bool) {
        self.use_exact_velocity = exact;
    }

    /// Read raw shaft angle in `[0, 2π)` with quantization applied if configured.
    #[inline]
    pub fn get_sensor_angle(&self) -> f32 {
        let norm = math::normalize_angle(self.mechanical_angle.get());

        if let Some(bits) = self.quantization_bits {
            let n = (1u32 << bits) as f32;
            let step = _2PI / n;
            let count = libm::roundf(norm / step) as u32;
            let count = count % (1u32 << bits);
            count as f32 * step
        } else {
            norm
        }
    }
}

impl<C: Clock> Sensor<C> for SimulatedSensor {
    fn base(&self) -> &SensorBase {
        &self.base
    }

    fn base_mut(&mut self) -> &mut SensorBase {
        &mut self.base
    }

    fn get_sensor_angle(&self) -> f32 {
        Self::get_sensor_angle(self)
    }

    fn get_velocity(&mut self) -> f32 {
        if self.use_exact_velocity {
            self.mechanical_velocity.get()
        } else {
            // Discrete numerical derivative from SensorBase:
            let ts_raw = self.base.angle_prev_ts.wrapping_sub(self.base.vel_angle_prev_ts) as i64;
            let ts_seconds = ts_raw as f32 * 1e-6_f32;
            if ts_seconds < 0.0 {
                self.base.vel_angle_prev = self.base.angle_prev;
                self.base.vel_full_rotations = self.base.full_rotations;
                self.base.vel_angle_prev_ts = self.base.angle_prev_ts;
                return self.base.velocity;
            }
            if ts_seconds < self.base.min_elapsed_time {
                return self.base.velocity;
            }

            let (current_angle, prev_angle) = if self.base.full_rotations == self.base.vel_full_rotations {
                (self.base.angle_prev, self.base.vel_angle_prev)
            } else {
                (
                    self.base.full_rotations as f32 * _2PI + self.base.angle_prev,
                    self.base.vel_full_rotations as f32 * _2PI + self.base.vel_angle_prev,
                )
            };
            let delta_angle = current_angle - prev_angle;

            if libm::fabsf(delta_angle) > 1e-8 {
                let v = delta_angle / ts_seconds;
                self.base.velocity = v;
                self.base.vel_angle_prev = self.base.angle_prev;
                self.base.vel_full_rotations = self.base.full_rotations;
                self.base.vel_angle_prev_ts = self.base.angle_prev_ts;
                v
            } else {
                self.base.velocity
            }
        }
    }
}
