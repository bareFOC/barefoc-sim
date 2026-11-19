//! # adapters::clock — Simulated microsecond monotonic clock
//!
//! Implements the [`Clock`] trait, providing a deterministic virtual microsecond
//! time base for FOC control loops, PIDs, and low-pass filters.

use core::cell::Cell;
use barefoc::common::time::Clock;

/// Deterministic virtual monotonic clock providing microsecond timestamps.
pub struct SimClock {
    micros: Cell<u64>,
}

impl Default for SimClock {
    fn default() -> Self {
        Self::new()
    }
}

impl SimClock {
    /// Construct a new simulated clock starting at time `0 µs`.
    pub const fn new() -> Self {
        Self {
            micros: Cell::new(0),
        }
    }

    /// Set current simulation time in microseconds.
    #[inline]
    pub fn set_micros(&self, us: u64) {
        self.micros.set(us);
    }

    /// Advance simulation clock by `us` microseconds.
    #[inline]
    pub fn advance_us(&self, us: u32) {
        self.micros.set(self.micros.get().wrapping_add(us as u64));
    }

    /// Advance simulation clock by `dt` seconds.
    #[inline]
    pub fn advance_dt(&self, dt_seconds: f32) {
        if dt_seconds > 0.0 {
            let us = (dt_seconds * 1.0e6) as u32;
            self.advance_us(us);
        }
    }
}

impl Clock for SimClock {
    #[inline]
    fn now_us(&self) -> u32 {
        self.micros.get() as u32
    }

    #[inline]
    fn delay_ms(&self, ms: u32) {
        self.advance_us(ms.saturating_mul(1000));
    }
}
