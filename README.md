# barefoc-sim

Software-in-the-Loop (SIL) electromechanical BLDC motor simulator and ODrive GUI live tuning bridge for BareFOC.

## Features

- **Electromechanical Physics ODE Model**: Solves coupled stator $d$-$q$ differential equations and Newtonian rotational dynamics with back-EMF, viscous damping, and Coulomb friction.
- **Numerical Stability via Sub-stepping**: Decouples FOC execution rate ($25\,\text{kHz}$) from continuous physics integration ($\Delta t_{\text{phys}} \le 1\text{--}2\,\mu\text{s}$), enabling unconditional numerical stability even on low-inductance micro-motors (e.g. Miniata 1104 with $L = 15\,\mu\text{H}$).
- **Hardware Trait Bridges**: Seamless implementations of `BLDCDriver`, `Sensor`, and `CurrentSense`.
- **Zero Real-Time CI Harness**: Fast automated step-response benchmarks running thousands of times faster than real time in `cargo test`.
- **Live GUI Tuning**: Built-in TCP server implementing the Fibre RPC protocol, allowing direct connection from the official [ODrive Web GUI](https://gui.odriverobotics.com) or `odrivetool`.

## Motor Presets

- `MotorParams::miniata_1104_4300kv()`: Micro BLDC ($R=0.55\,\Omega$, $L=15\,\mu\text{H}$, $PP=7$, $KV=4300$)
- `MotorParams::generic_drone_2205()`: Multicopter drone motor ($R=0.08\,\Omega$, $L=25\,\mu\text{H}$, $PP=7$, $KV=2300$)
- `MotorParams::generic_gimbal_2804()`: High-resistance gimbal motor ($R=10.0\,\Omega$, $L=1.5\,\text{mH}$, $PP=7$, $KV=140$)

## Usage in Host Unit Tests

```rust
use barefoc_sim::{MotorParams, SimClock, SimRunner};
use barefoc::motor::enums::MotionControlType;
use barefoc::motor::base::FOCMotor;

let clock = SimClock::new();
let params = MotorParams::miniata_1104_4300kv();
let mut runner = SimRunner::new(&clock, params, 40e-6);

runner.motor.cfg_mut().update_motion_control_type(MotionControlType::velocity);
runner.motor.cfg_mut().PID_velocity.P = 0.05;
runner.motor.cfg_mut().PID_velocity.I = 1.0;

runner.init();
runner.init_foc();

// Simulate 0.5 seconds of operation commanding 50 rad/s
runner.step_for(0.5, 50.0);

// Check convergence
let vel = runner.state().shaft_velocity;
assert!((vel - 50.0).abs() < 1.0);
```

## Running the ODrive GUI Tuning Server

```bash
cargo run -p barefoc-sim --example sil_gui_tuning
```
Then open Google Chrome, navigate to `https://gui.odriverobotics.com`, and connect via TCP to `127.0.0.1:8000`.
