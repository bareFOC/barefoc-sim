//! # runner — Simulation orchestrator coupling physics sub-stepping with BareFOC
//!
//! Synchronizes the BareFOC discrete control loop (`loop_foc`, `move_motor`)
//! with the continuous electromechanical physics sub-stepping solver.

use crate::adapters::{SimClock, SimulatedBLDCDriver, SimulatedCurrentSense, SimulatedSensor};
use crate::physics::{MotorParams, MotorSolver, MotorState};
use barefoc::motor::base::FOCMotor;
use barefoc::motor::bldc::BLDCMotor;
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::driver::FOCDriver;
use barefoc::traits::sensor::Direction;

/// Top-level Software-in-the-Loop simulation orchestrator.
pub struct SimRunner<'clk> {
    /// Production BareFOC 3-phase motor instance.
    pub motor: BLDCMotor<'clk, SimulatedSensor, SimulatedBLDCDriver, SimulatedCurrentSense, SimClock>,
    /// High-fidelity numerical electromechanical physics solver.
    pub solver: MotorSolver,
    /// Virtual monotonic clock.
    pub clock: &'clk SimClock,
    /// FOC control loop execution period in seconds (e.g. $40\,\mu\text{s}$ for $25\,\text{kHz}$).
    pub dt_foc: f32,
    /// External disturbance / load torque applied to the rotor shaft in $\text{N}\cdot\text{m}$.
    pub tau_load: f32,
}

impl<'clk> SimRunner<'clk> {
    /// Construct a new simulation runner with the specified motor profile and FOC cycle period.
    pub fn new(clock: &'clk SimClock, params: MotorParams, dt_foc: f32) -> Self {
        let v_supply = 12.0;
        let v_limit = 12.0;

        let mut driver = SimulatedBLDCDriver::new(v_supply, v_limit);
        let _ = driver.init();

        let sensor = SimulatedSensor::new();
        let mut current_sense = SimulatedCurrentSense::new();
        let _ = current_sense.init();

        let mut motor = BLDCMotor::new(
            params.pole_pairs,
            params.r_phase,
            params.kv_rating,
            params.l_q,
            params.l_d,
            clock,
        );

        motor.link_driver(driver);
        motor.link_sensor(sensor);
        motor.link_current_sense(current_sense);

        // Preconfigure sensor direction and zero offset so init_foc completes immediately:
        motor.cfg_mut().sensor_direction = Direction::CW;
        motor.cfg_mut().zero_electric_angle = 0.0;
        motor.cfg_mut().voltage_limit = v_limit;
        motor.cfg_mut().current_limit = 10.0;
        motor.cfg_mut().velocity_limit = 1000.0;

        let solver = MotorSolver::new(params);

        Self {
            motor,
            solver,
            clock,
            dt_foc,
            tau_load: 0.0,
        }
    }

    /// Convenience helper to initialize the driver and motor algorithms.
    pub fn init(&mut self) -> i32 {
        self.motor.init()
    }

    /// Convenience helper to initialize the FOC commutation and calibrate sensors.
    pub fn init_foc(&mut self) -> i32 {
        self.motor.init_foc(self.clock)
    }

    /// Execute a single simulation step:
    /// 1. Run BareFOC hot loop (`loop_foc`).
    /// 2. Run BareFOC outer loop (`move_motor`).
    /// 3. Read phase voltages from the virtual driver.
    /// 4. Integrate physical dynamics across `dt_foc` via sub-stepping.
    /// 5. Feed back simulated mechanical state and currents to sensor and current sense.
    /// 6. Advance virtual clock by `dt_foc`.
    pub fn step(&mut self, target: f32) {
        // 1 & 2: Run BareFOC control algorithms
        self.motor.loop_foc(self.clock);
        self.motor.move_motor(target, self.clock);

        // 3: Read commanded terminal voltages from driver
        let (ua, ub, uc) = if let Some(d) = self.motor.driver() {
            d.voltages()
        } else {
            (0.0, 0.0, 0.0)
        };

        // 4: Physics integration
        self.solver.step(ua, ub, uc, self.tau_load, self.dt_foc);

        // 5: Update sensor and current sense state
        let state = *self.solver.state();
        let pp = self.solver.params().pole_pairs;

        if let Some(s) = self.motor.sensor_mut() {
            s.set_state(state.shaft_angle, state.shaft_velocity);
        }
        if let Some(cs) = self.motor.current_sense_mut() {
            cs.set_dq(state.i_d, state.i_q, state.electrical_angle(pp));
        }

        // 6: Advance virtual time
        self.clock.advance_dt(self.dt_foc);
    }

    /// Run simulation for a specified duration in seconds.
    ///
    /// Returns the total number of FOC control cycles executed.
    pub fn step_for(&mut self, duration_s: f32, target: f32) -> usize {
        if duration_s <= 0.0 || self.dt_foc <= 0.0 {
            return 0;
        }

        let steps = (duration_s / self.dt_foc) as usize;
        for _ in 0..steps {
            self.step(target);
        }
        steps
    }

    /// Read the instantaneous physical state from the solver.
    #[inline]
    pub fn state(&self) -> &MotorState {
        self.solver.state()
    }

    /// Access mutable solver (e.g. to set disturbance torque or sub-step duration).
    #[inline]
    pub fn solver_mut(&mut self) -> &mut MotorSolver {
        &mut self.solver
    }
}
