//! Software-in-the-Loop interactive simulation bridge for ODrive Web GUI tuning.
//!
//! Run with:
//! ```bash
//! cargo run -p barefoc-sim --example sil_gui_tuning
//! ```
//! Then navigate to <https://gui.odriverobotics.com> and select TCP connection
//! to `127.0.0.1:8000`.

use std::thread;
use std::time::{Duration, Instant};

use barefoc::motor::base::FOCMotor;
use barefoc::motor::enums::MotionControlType;
use barefoc_fibre::odrive::{create_odrive_schema, make_motor_config_endpoints};
use barefoc_fibre::server::FibreServer;
use barefoc_sim::{MotorParams, SimClock, SimFibreServer, SimRunner};

fn main() {
    println!("============================================================");
    println!(" BareFOC SIL Motor Simulator & ODrive GUI Tuning Bridge");
    println!("============================================================");

    let clock = SimClock::new();
    let params = MotorParams::miniata_1104_4300kv();
    let dt_foc = 40.0e-6; // 25 kHz FOC rate

    let mut runner = SimRunner::new(&clock, params, dt_foc);

    // Configure velocity closed-loop controller
    runner
        .motor
        .cfg_mut()
        .update_motion_control_type(MotionControlType::velocity);
    runner.motor.cfg_mut().PID_velocity.set_p(0.05);
    runner.motor.cfg_mut().PID_velocity.set_i(1.0);
    runner.motor.cfg_mut().PID_velocity.set_d(0.0);
    runner.motor.cfg_mut().target = 20.0; // initial target 20 rad/s

    println!("[sim] Initializing motor and FOC algorithms...");
    let init_res = runner.init();
    let foc_res = runner.init_foc();
    println!(
        "[sim] Motor initialized: {}, FOC initialized: {}",
        init_res, foc_res
    );

    // Initialize Fibre RPC server
    let schema = create_odrive_schema();
    let endpoints = make_motor_config_endpoints::<SimClock>();
    let fibre_server = FibreServer::new(schema, &endpoints);

    let mut tcp_server = match SimFibreServer::bind("127.0.0.1:8000") {
        Ok(srv) => {
            println!("[server] Fibre TCP server listening on 127.0.0.1:8000");
            println!("[server] Connect your browser to https://gui.odriverobotics.com");
            srv
        }
        Err(e) => {
            eprintln!("[server] Failed to bind TCP listener: {}", e);
            return;
        }
    };

    println!("[sim] Simulation running in real-time. Press Ctrl+C to stop.");

    // Loop pacing in blocks of 25 FOC steps (1 ms real-time per block)
    let steps_per_block = 25;
    let block_duration = Duration::from_micros(1000);

    loop {
        let block_start = Instant::now();

        // Poll incoming network packets from ODrive GUI
        tcp_server.poll(&fibre_server, runner.motor.cfg_mut());

        // Step simulation for 1 millisecond (25 cycles of 40 µs)
        let target = runner.motor.cfg().target;
        for _ in 0..steps_per_block {
            runner.step(target);
        }

        // Pace to wall-clock real time
        let elapsed = block_start.elapsed();
        if elapsed < block_duration {
            thread::sleep(block_duration - elapsed);
        }
    }
}
