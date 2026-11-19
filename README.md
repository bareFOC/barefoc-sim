# barefoc-sim

Software-in-the-Loop (SIL) electromechanical BLDC motor simulator and ODrive GUI tuning bridge for the [BareFOC](https://github.com/bareFOC/bareFOC) ecosystem.

This crate allows testing, tuning, and verifying Field Oriented Control loops on a host machine without physical hardware or flashing microcontrollers.

---

## Capabilities

- **Electromechanical Physics Model**: Solves coupled stator $d$-$q$ differential equations and Newtonian rotational dynamics including back-EMF, viscous damping, and Coulomb friction.
- **Sub-stepped Integration**: Decouples the outer FOC control loop frequency (e.g. 25 kHz) from continuous physics integration ($\Delta t_{\text{phys}} \le 1\text{--}2\,\mu\text{s}$), maintaining numerical stability on low-inductance motors (e.g., $L = 15\,\mu\text{H}$).
- **Trait Adapters**: Provides mock implementations of `barefoc` traits: `BLDCDriver`, `Sensor`, `CurrentSense`, and `Clock`.
- **Automated CI Benchmarking**: Host unit tests verify step response, settling time, and tracking error in fractional execution time.
- **Live GUI Tuning**: Built-in TCP server implementing the Fibre RPC protocol, supporting direct connection from the [ODrive Web GUI](https://gui.odriverobotics.com) or `odrivetool`.

---

## Motor Presets

Pre-configured parameters are available for common motor types:
- `MotorParams::miniata_1104_4300kv()`: Micro BLDC ($R = 0.55\,\Omega$, $L = 15\,\mu\text{H}$, $PP = 7$, $KV = 4300$)
- `MotorParams::generic_drone_2205()`: Drone motor ($R = 0.08\,\Omega$, $L = 25\,\mu\text{H}$, $PP = 7$, $KV = 2300$)
- `MotorParams::generic_gimbal_2804()`: High-resistance gimbal motor ($R = 10.0\,\Omega$, $L = 1.5\,\text{mH}$, $PP = 7$, $KV = 140$)

---

## Usage in Host Tests

```rust
use barefoc_sim::{MotorParams, SimClock, SimRunner};
use barefoc::motor::enums::MotionControlType;
use barefoc::motor::base::FOCMotor;

#[test]
fn test_velocity_step_response() {
    let clock = SimClock::new();
    let params = MotorParams::miniata_1104_4300kv();
    let mut runner = SimRunner::new(&clock, params, 40e-6);

    runner.motor.cfg_mut().update_motion_control_type(MotionControlType::velocity);
    runner.motor.cfg_mut().PID_velocity.P = 0.05;
    runner.motor.cfg_mut().PID_velocity.I = 1.0;

    runner.init();
    runner.init_foc();

    // Simulate 0.5 seconds commanding 50 rad/s
    runner.step_for(0.5, 50.0);

    // Verify velocity convergence
    let vel = runner.state().shaft_velocity;
    assert!((vel - 50.0).abs() < 1.0);
}
```

---

## Running the ODrive GUI Tuning Server

To launch the Fibre TCP bridge on localhost:

```bash
cargo run -p barefoc-sim --example sil_gui_tuning
```

Navigate to `https://gui.odriverobotics.com` in Google Chrome and connect to TCP `127.0.0.1:8000` to inspect and tune control variables in real time.

---

## Testing

Run simulation test suite:

```bash
cargo test -p barefoc-sim
```

---

## License

This project is licensed under the MIT License. See [`LICENSE`](LICENSE) for details.
