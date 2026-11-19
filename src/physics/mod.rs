//! # physics — Electromechanical motor modeling and numerical physics solver

pub mod motor;
pub mod solver;

pub use motor::{MotorParams, MotorState};
pub use solver::MotorSolver;
