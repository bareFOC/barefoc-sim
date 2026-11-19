//! # adapters — BareFOC trait implementations backed by the simulation model

pub mod clock;
pub mod current_sense;
pub mod driver;
pub mod sensor;

pub use clock::SimClock;
pub use current_sense::SimulatedCurrentSense;
pub use driver::SimulatedBLDCDriver;
pub use sensor::SimulatedSensor;
