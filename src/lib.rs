//! # barefoc-sim
//!
//! Software-in-the-Loop (SIL) electromechanical BLDC motor simulator and ODrive GUI
//! live tuning bridge for the BareFOC library.
//!
//! ## Overview
//!
//! BareFOC algorithms run unmodified on host architectures (`x86_64` / `arm64`) by
//! implementing mock driver, sensor, and current sense adapters backed by an accurate
//! continuous electromechanical physics engine.
//!
//! Key features:
//! - **Numerical Physics Solver**: Continuous $d$-$q$ frame stator voltage balance differential
//!   equations coupled with rotor Newtonian mechanics.
//! - **Sub-stepping Stability**: Sub-stepped RK4 integration ($\Delta t_{\text{phys}} \le 1\text{--}2\,\mu\text{s}$)
//!   guaranteeing stability even on stiff, ultra-low inductance motors ($L \approx 15\,\mu\text{H}$).
//! - **Hardware Trait Adapters**: [`SimulatedBLDCDriver`], [`SimulatedSensor`], and [`SimulatedCurrentSense`]
//!   implementing the core BareFOC traits.
//! - **Faster-than-Realtime CI**: Deterministic automated step response unit tests executing
//!   thousands of times faster than real time.
//! - **ODrive Web GUI Tuning Bridge**: Built-in Fibre TCP server on `127.0.0.1:8000` (`feature = "std"`),
//!   enabling live parameter tuning and real-time step response plotting in Google Chrome.

#![cfg_attr(not(feature = "std"), no_std)]

pub mod adapters;
pub mod physics;
pub mod runner;

#[cfg(feature = "std")]
pub mod server;

// Re-exports
pub use adapters::{SimClock, SimulatedBLDCDriver, SimulatedCurrentSense, SimulatedSensor};
pub use physics::{MotorParams, MotorSolver, MotorState};
pub use runner::SimRunner;

#[cfg(feature = "std")]
pub use server::SimFibreServer;
