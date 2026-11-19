use barefoc::math::{self, clarke, park, DQCurrent};
use barefoc::motor::base::FOCMotor;
use barefoc::motor::enums::MotionControlType;
use barefoc::traits::current_sense::CurrentSense;
use barefoc::traits::sensor::Sensor;
use barefoc_sim::{
    MotorParams, MotorSolver, SimClock, SimRunner, SimulatedCurrentSense, SimulatedSensor,
};

#[test]
fn test_electrical_torque_and_bemf() {
    let params = MotorParams::miniata_1104_4300kv();
    let mut solver = MotorSolver::new(params);

    // Apply constant Vq = 1.0 V at zero velocity with shaft locked (tau_load equals tau_e)
    let v_q = 1.0;
    let expected_i_q_steady = v_q / params.r_phase; // 1.0 / 0.55 = ~1.818 A
    let expected_torque = 1.5 * (params.pole_pairs as f32) * params.flux_linkage * expected_i_q_steady;

    // Run for 1 ms with locked rotor
    for _ in 0..100 {
        let current_tau = solver.state().electrical_torque;
        solver.step_dq(0.0, v_q, current_tau, 10.0e-6);
    }

    let state = solver.state();
    assert!(
        (state.i_q - expected_i_q_steady).abs() < 0.05,
        "Expected steady Iq ~ {}, got {}",
        expected_i_q_steady,
        state.i_q
    );
    assert!(
        (state.electrical_torque - expected_torque).abs() < 0.001,
        "Expected steady torque ~ {}, got {}",
        expected_torque,
        state.electrical_torque
    );

    // Verify back-EMF at speed:
    // When rotor spins at omega_m = 200 rad/s, Bemf = PP * omega_m * lambda_m
    let omega_m = 200.0;
    solver.state_mut().shaft_velocity = omega_m;
    let e_q = (params.pole_pairs as f32) * omega_m * params.flux_linkage;

    // At steady state with speed, Iq = (Vq - Eq) / Rs
    let expected_i_q_spinning = (v_q - e_q) / params.r_phase;
    for _ in 0..100 {
        solver.state_mut().shaft_velocity = omega_m; // hold speed
        solver.step_dq(0.0, v_q, 0.0, 10.0e-6);
    }

    assert!(
        (solver.state().i_q - expected_i_q_spinning).abs() < 0.05,
        "Expected Iq under back-EMF ~ {}, got {}",
        expected_i_q_spinning,
        solver.state().i_q
    );
}

#[test]
fn test_substepping_stability_low_inductance() {
    // Miniata 1104 has L = 15 uH, Rs = 0.55 Ohm -> tau = L/Rs = 27.27 us.
    // If stepped at dt = 100 us with single-step Forward Euler, it would violently explode.
    // Our sub-stepping solver breaks it into dt_sub <= 1 us and stably converges.
    let params = MotorParams::miniata_1104_4300kv();
    let mut solver = MotorSolver::new(params);

    // Apply large 10V step over 50 us intervals (almost 2x electrical time constant)
    for _ in 0..200 {
        solver.step_dq(0.0, 10.0, 0.0, 50.0e-6);
        assert!(!solver.state().i_d.is_nan());
        assert!(!solver.state().i_q.is_nan());
        assert!(!solver.state().shaft_velocity.is_nan());
    }

    assert!(solver.state().i_q.is_finite());
    assert!(solver.state().shaft_velocity > 0.0);
}

#[test]
fn test_closed_loop_velocity_step_convergence() {
    let clock = SimClock::new();
    let params = MotorParams::miniata_1104_4300kv();
    let dt_foc = 40.0e-6; // 25 kHz

    let mut runner = SimRunner::new(&clock, params, dt_foc);

    // Velocity control
    runner
        .motor
        .cfg_mut()
        .update_motion_control_type(MotionControlType::velocity);
    runner.motor.cfg_mut().PID_velocity.set_p(0.05);
    runner.motor.cfg_mut().PID_velocity.set_i(1.0);
    runner.motor.cfg_mut().PID_velocity.set_d(0.0);
    runner.motor.cfg_mut().voltage_limit = 12.0;

    assert_eq!(runner.init(), 1);
    assert_eq!(runner.init_foc(), 1);

    let target_velocity = 50.0; // 50 rad/s
    runner.step_for(0.5, target_velocity);

    let final_velocity = runner.state().shaft_velocity;
    let error_percent = ((final_velocity - target_velocity) / target_velocity).abs() * 100.0;

    assert!(
        error_percent < 2.0,
        "Velocity did not converge to within 2%: target={}, actual={}, error={:.2}%",
        target_velocity,
        final_velocity,
        error_percent
    );
}

#[test]
fn test_closed_loop_angle_step_convergence() {
    let clock = SimClock::new();
    let params = MotorParams::miniata_1104_4300kv();
    let dt_foc = 40.0e-6; // 25 kHz

    let mut runner = SimRunner::new(&clock, params, dt_foc);

    // Cascaded angle control (Angle -> Velocity -> Voltage)
    runner
        .motor
        .cfg_mut()
        .update_motion_control_type(MotionControlType::angle);
    runner.motor.cfg_mut().P_angle.set_p(20.0);
    runner.motor.cfg_mut().PID_velocity.set_p(0.08);
    runner.motor.cfg_mut().PID_velocity.set_i(1.2);
    runner.motor.cfg_mut().velocity_limit = 300.0;

    assert_eq!(runner.init(), 1);
    assert_eq!(runner.init_foc(), 1);

    let target_angle = core::f32::consts::PI; // pi radians (180 degrees)
    runner.step_for(0.6, target_angle);

    let final_angle = runner.state().shaft_angle;
    let angle_error = (final_angle - target_angle).abs();

    assert!(
        angle_error < 0.05,
        "Angle did not converge to within 0.05 rad: target={}, actual={}, error={}",
        target_angle,
        final_angle,
        angle_error
    );
}

#[test]
fn test_sensor_quantization_14bit() {
    let clock = SimClock::new();
    let mut sensor = SimulatedSensor::with_quantization(14); // 16384 counts
    sensor.init(&clock);

    let step = math::_2PI / 16384.0;

    // Set angle to 1.5 * step
    sensor.set_state(1.5 * step, 0.0);
    let read_angle = sensor.get_sensor_angle();

    // With round half away from zero, 1.5 rounds to 2.0 steps
    let expected = 2.0 * step;
    assert!(
        (read_angle - expected).abs() < 1e-6,
        "Expected {}, got {}",
        expected,
        read_angle
    );

    // Verify quantization granularity
    sensor.set_state(0.1 * step, 0.0);
    assert_eq!(sensor.get_sensor_angle(), 0.0);
}

#[test]
fn test_simulated_current_sense_roundtrip() {
    let cs = SimulatedCurrentSense::new();
    let angle_el = 0.75;
    let id_in = 1.5;
    let iq_in = -2.3;

    cs.set_dq(id_in, iq_in, angle_el);

    let mut cs_mut = cs;
    let phase = cs_mut.get_phase_currents();
    let ab = clarke(phase);
    let dq_out: DQCurrent = park(ab, angle_el);

    assert!((dq_out.d - id_in).abs() < 1e-4);
    assert!((dq_out.q - iq_in).abs() < 1e-4);
}

#[test]
fn test_open_loop_velocity_rotation() {
    let clock = SimClock::new();
    let params = MotorParams::miniata_1104_4300kv();
    let mut runner = SimRunner::new(&clock, params, 40e-6);

    runner
        .motor
        .cfg_mut()
        .update_motion_control_type(MotionControlType::velocity_openloop);
    runner.motor.cfg_mut().voltage_limit = 5.0;

    runner.init();
    runner.init_foc();

    runner.step_for(0.2, 30.0);

    // In open loop, shaft angle should have accumulated rotation
    assert!(runner.motor.cfg().shaft_angle > 1.0);
}

#[test]
fn test_sim_fibre_tcp_endpoint0_query() {
    use std::io::{Read, Write};
    use std::net::TcpStream;
    use barefoc_fibre::crc::PROTOCOL_VERSION;
    use barefoc_fibre::framing::PacketWrapper;
    use barefoc_fibre::odrive::{create_odrive_schema, make_motor_config_endpoints};
    use barefoc_fibre::server::FibreServer;
    use barefoc_sim::SimFibreServer;

    let mut server = SimFibreServer::bind("127.0.0.1:8999").expect("bind port");
    let schema = create_odrive_schema();
    let endpoints = make_motor_config_endpoints::<SimClock>();
    let fibre_server = FibreServer::new(schema, &endpoints);

    let clock = SimClock::new();
    let mut config = barefoc::motor::base::MotorConfig::new(&clock);

    // Client connects
    let mut client = TcpStream::connect("127.0.0.1:8999").expect("connect");
    client.set_nonblocking(false).expect("set blocking");

    // Client sends request to ep 0: seq_no=1, ep=0 | 0x8000, rx_len=32, offset=0, trailer=PROTOCOL_VERSION
    let mut req = [0u8; 12];
    req[0..2].copy_from_slice(&1u16.to_le_bytes());
    req[2..4].copy_from_slice(&0x8000u16.to_le_bytes());
    req[4..6].copy_from_slice(&32u16.to_le_bytes());
    req[6..10].copy_from_slice(&0u32.to_le_bytes());
    req[10..12].copy_from_slice(&PROTOCOL_VERSION.to_le_bytes());

    let mut framed_req = [0u8; 64];
    let frame_len = PacketWrapper::wrap(&req, &mut framed_req).unwrap();
    client.write_all(&framed_req[..frame_len]).unwrap();

    // Server polls
    for _ in 0..10 {
        server.poll(&fibre_server, &mut config);
        std::thread::sleep(std::time::Duration::from_millis(5));
    }

    // Client reads response
    let mut resp = [0u8; 64];
    let n = client.read(&mut resp).unwrap();
    assert!(n > 0);
    assert_eq!(resp[0], barefoc_fibre::CANONICAL_PREFIX);
}

