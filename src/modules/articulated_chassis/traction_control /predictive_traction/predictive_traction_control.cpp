#include "predictive_traction_control.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

PredictiveTractionControl::PredictiveTractionControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // Initialize state vectors
    _vehicle_state.setZero();
    _vehicle_state_dot.setZero();
    _current_control.setZero();
    _optimal_control_sequence.setZero();
    _predicted_states.setZero();
    _predicted_slip_front.setZero();
    _predicted_slip_rear.setZero();
    _predicted_risk.setZero();

    // Initialize module status
    _status = {};
    _status.timestamp = hrt_absolute_time();

    // Initialize prediction output
    _prediction_output = {};
}

bool PredictiveTractionControl::init()
{
    // Update MPC parameters from PX4 parameters
    _mpc_params.weight_slip_tracking = _weight_slip.get();
    _mpc_params.weight_stability = _weight_stability.get();
    _mpc_params.weight_control_effort = _weight_control.get();
    _mpc_params.max_iterations = _max_iterations.get();
    _mpc_params.convergence_tolerance = _convergence_tolerance.get();

    _learning.adaptation_rate = _adaptation_rate.get();
    _learning.learning_active = _learning_enable.get();

    PX4_INFO("Predictive Traction Control initialized");
    PX4_INFO("Prediction horizon: %.1f s, Control rate: %.1f Hz",
             (double)_prediction_horizon_s.get(), (double)CONTROL_RATE_HZ);
    PX4_INFO("MPC weights - Slip: %.1f, Stability: %.1f, Control: %.1f",
             (double)_mpc_params.weight_slip_tracking,
             (double)_mpc_params.weight_stability,
             (double)_mpc_params.weight_control_effort);

    return true;
}

void PredictiveTractionControl::Run()
{
    if (!init()) {
        PX4_ERR("Predictive traction control initialization failed");
        return;
    }

    // Main control loop
    while (!should_exit()) {
        _computation_start_time = hrt_absolute_time();

        // Check for parameter updates
        if (_parameter_update_sub.updated()) {
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);
            updateParams();

            // Update MPC parameters
            _mpc_params.weight_slip_tracking = _weight_slip.get();
            _mpc_params.weight_stability = _weight_stability.get();
            _mpc_params.weight_control_effort = _weight_control.get();
            _mpc_params.max_iterations = _max_iterations.get();
            _mpc_params.convergence_tolerance = _convergence_tolerance.get();

            _learning.adaptation_rate = _adaptation_rate.get();
            _learning.learning_active = _learning_enable.get();
        }

        // Update vehicle state from sensors
        update_vehicle_state();

        // Update terrain and vehicle model
        update_terrain_model();
        update_system_model();

        // Perform MPC prediction and optimization
        predict_future_states();
        optimize_control_sequence();

        // Apply MPC solution
        apply_mpc_solution();

        // Update learning model if enabled
        if (_learning.learning_active) {
            update_learning_model();
        }

        // Publish prediction results
        publish_prediction_results();

        // Update performance metrics
        uint64_t computation_time = hrt_elapsed_time(&_computation_start_time);
        _performance.avg_computation_time_ms = 0.9f * _performance.avg_computation_time_ms +
                                               0.1f * (computation_time / 1000.0f);

        // Sleep until next control cycle
        px4_usleep(CONTROL_INTERVAL_US);
    }
}

void PredictiveTractionControl::update_vehicle_state()
{
    bool state_updated = false;

    // Get slip estimation data
    if (_slip_estimation_sub.updated()) {
        slip_estimation_s slip_data;
        _slip_estimation_sub.copy(&slip_data);

        _sensor_data.slip_front = slip_data.slip_ratio_front;
        _sensor_data.slip_rear = slip_data.slip_ratio_rear;
        _sensor_data.lateral_slip_rad = slip_data.slip_angle_front_rad;
        _sensor_data.friction_estimate = slip_data.friction_coefficient;
        _sensor_data.vehicle_speed_ms = slip_data.longitudinal_slip_velocity;

        state_updated = true;
    }

    // Get vehicle attitude for orientation information
    if (_vehicle_attitude_sub.updated()) {
        vehicle_attitude_s attitude;
        _vehicle_attitude_sub.copy(&attitude);

        // Convert quaternion to Euler angles if needed for vehicle state
        matrix::Eulerf euler(matrix::Quatf(attitude.q));
        // Store attitude in vehicle state if needed
        state_updated = true;
    }

    // Get vehicle angular velocity for yaw rate
    if (_vehicle_angular_velocity_sub.updated()) {
        vehicle_angular_velocity_s angular_vel;
        _vehicle_angular_velocity_sub.copy(&angular_vel);

        _sensor_data.yaw_rate_rad_s = angular_vel.xyz[2]; // Z-axis is yaw rate in body frame
        state_updated = true;
    }

    // Get steering status
    if (_steering_status_sub.updated()) {
        steering_status_s steering;
        _steering_status_sub.copy(&steering);

        _sensor_data.steering_angle_rad = steering.actual_angle_rad;
        state_updated = true;
    }

    // Get load information
    if (_load_sensing_sub.updated()) {
        load_sensing_s load_data;
        _load_sensing_sub.copy(&load_data);

        _sensor_data.payload_mass_kg = load_data.payload_mass_kg;
        _vehicle_params.mass_kg = 12000.0f + load_data.payload_mass_kg;
        state_updated = true;
    }

    if (state_updated) {
        // Update vehicle state vector
        _vehicle_state(0) = _sensor_data.vehicle_speed_ms;  // longitudinal velocity
        _vehicle_state(1) = 0.0f;  // lateral velocity (calculated from slip angle)
        _vehicle_state(2) = _sensor_data.yaw_rate_rad_s;   // yaw rate
        _vehicle_state(3) = _sensor_data.vehicle_speed_ms / _vehicle_params.tire_radius_m; // front wheel speed
        _vehicle_state(4) = _sensor_data.vehicle_speed_ms / _vehicle_params.tire_radius_m; // rear wheel speed
        _vehicle_state(5) = _sensor_data.steering_angle_rad; // steering angle

        _sensor_data.last_update_time = hrt_absolute_time();
        _sensor_data.data_valid = true;
    }
}

void PredictiveTractionControl::update_terrain_model()
{
    if (_terrain_adaptation_sub.updated()) {
        terrain_adaptation_s terrain;
        _terrain_adaptation_sub.copy(&terrain);

        _sensor_data.terrain_friction = terrain.friction_coefficient;
        _sensor_data.slope_angle_rad = terrain.slope_angle_rad;

        // Adapt vehicle parameters based on terrain
        _vehicle_params.max_friction_coeff = terrain.friction_coefficient;

        // Update terrain-specific learning model
        if (terrain.terrain_type < 10) {
            _learning.learned_friction_model[terrain.terrain_type] =
                0.9f * _learning.learned_friction_model[terrain.terrain_type] +
                0.1f * terrain.friction_coefficient;
        }
    }
}

void PredictiveTractionControl::update_system_model()
{
    // Update vehicle parameters based on current state
    adapt_control_parameters();
}

void PredictiveTractionControl::predict_future_states()
{
    if (!_sensor_data.data_valid) {
        return;
    }

    // Initialize prediction with current state
    _predicted_states.col(0) = _vehicle_state;

    // Predict future states using vehicle dynamics model
    for (int k = 0; k < PREDICTION_HORIZON - 1; k++) {
        Vector<float, 6> current_state = _predicted_states.col(k);
        Vector<float, 3> control_input;

        // Use current control or planned control sequence
        if (k < CONTROL_HORIZON) {
            control_input = _optimal_control_sequence.col(k);
        } else {
            control_input = _optimal_control_sequence.col(CONTROL_HORIZON - 1);
        }

        // Propagate dynamics forward one time step
        Vector<float, 6> state_derivative = vehicle_dynamics_model(current_state, control_input);
        _predicted_states.col(k + 1) = current_state + state_derivative * PREDICTION_TIME_STEP;

        // Calculate predicted slip values
        float wheel_speed_front = _predicted_states(3, k + 1);
        float wheel_speed_rear = _predicted_states(4, k + 1);
        float vehicle_speed = _predicted_states(0, k + 1);

        if (vehicle_speed > 0.1f) {
            _predicted_slip_front(k + 1) = (wheel_speed_front * _vehicle_params.tire_radius_m - vehicle_speed) / vehicle_speed;
            _predicted_slip_rear(k + 1) = (wheel_speed_rear * _vehicle_params.tire_radius_m - vehicle_speed) / vehicle_speed;
        } else {
            _predicted_slip_front(k + 1) = 0.0f;
            _predicted_slip_rear(k + 1) = 0.0f;
        }

        // Calculate risk level for this prediction step
        _predicted_risk(k + 1) = calculate_slip_risk(_predicted_slip_front(k + 1), _predicted_slip_rear(k + 1));
    }

    _prediction_active = true;
}

Vector<float, 6> PredictiveTractionControl::vehicle_dynamics_model(const Vector<float, 6>& state,
                                                                   const Vector<float, 3>& control)
{
    Vector<float, 6> state_dot;

    // Extract state variables
    float vx = state(0);     // longitudinal velocity
    float vy = state(1);     // lateral velocity
    float yaw_rate = state(2); // yaw rate
    // float wheel_speed_f = state(3); // front wheel speed (unused in simplified model)
    // float wheel_speed_r = state(4); // rear wheel speed (unused in simplified model)
    float steering_angle = state(5); // steering angle

    // Extract control inputs
    float torque_distribution = control(0); // -1 to 1
    float steering_correction = control(1); // rad
    float brake_intervention = control(2);  // 0 to 1

    // Vehicle parameters
    float mass = _vehicle_params.mass_kg;
    float L = _vehicle_params.wheelbase_m;
    float Iz = _vehicle_params.moment_inertia_kg_m2;
    float Cf = _vehicle_params.cornering_stiffness_n_rad;
    float Cr = _vehicle_params.cornering_stiffness_n_rad;
    float R = _vehicle_params.tire_radius_m;

    // Calculate tire forces (simplified bicycle model)
    float alpha_f = steering_angle + steering_correction - atan2f(vy + L/2 * yaw_rate, vx);
    float alpha_r = -atan2f(vy - L/2 * yaw_rate, vx);

    float Fyf = Cf * alpha_f;
    float Fyr = Cr * alpha_r;

    // Longitudinal forces from torque distribution
    float total_torque = 1000.0f; // Base torque (Nm)
    float torque_front = total_torque * (0.5f + 0.5f * torque_distribution);
    float torque_rear = total_torque * (0.5f - 0.5f * torque_distribution);

    float Fxf = torque_front / R;
    float Fxr = torque_rear / R;

    // Apply brake intervention
    Fxf *= (1.0f - brake_intervention);
    Fxr *= (1.0f - brake_intervention);

    // Vehicle dynamics equations
    state_dot(0) = (Fxf + Fxr) / mass + vy * yaw_rate;  // vx_dot
    state_dot(1) = (Fyf + Fyr) / mass - vx * yaw_rate;  // vy_dot
    state_dot(2) = (Fyf * L/2 - Fyr * L/2) / Iz;        // yaw_rate_dot

    // Wheel dynamics (simplified)
    state_dot(3) = (torque_front - Fxf * R) / (10.0f); // Front wheel acceleration
    state_dot(4) = (torque_rear - Fxr * R) / (10.0f);  // Rear wheel acceleration
    state_dot(5) = 0.0f; // Steering angle rate (controlled separately)

    return state_dot;
}

void PredictiveTractionControl::optimize_control_sequence()
{
    if (!_prediction_active) {
        return;
    }

    // Simple gradient descent optimization for MPC
    // In a real implementation, this would use a more sophisticated optimizer

    Matrix<float, 3, CONTROL_HORIZON> best_control = _optimal_control_sequence;
    float best_cost = INFINITY;

    // Evaluate current control sequence
    evaluate_cost_function(best_control, best_cost);

    // Optimization iterations
    for (int iter = 0; iter < _mpc_params.max_iterations; iter++) {
        Matrix<float, 3, CONTROL_HORIZON> trial_control = best_control;

        // Add small perturbations to find better solution
        for (int i = 0; i < CONTROL_HORIZON; i++) {
            for (int j = 0; j < 3; j++) {
                float perturbation = 0.01f * (2.0f * (rand() / (float)RAND_MAX) - 1.0f);
                trial_control(j, i) += perturbation;
            }
        }

        // Check constraints
        if (!check_constraints(trial_control)) {
            continue;
        }

        // Evaluate cost
        float trial_cost;
        evaluate_cost_function(trial_control, trial_cost);

        // Update best solution
        if (trial_cost < best_cost) {
            best_cost = trial_cost;
            best_control = trial_control;
        }

        // Check convergence
        if (iter > 0 && fabsf(best_cost) < _mpc_params.convergence_tolerance) {
            break;
        }
    }

    _optimal_control_sequence = best_control;
}

void PredictiveTractionControl::evaluate_cost_function(const Matrix<float, 3, CONTROL_HORIZON>& control_sequence,
                                                       float& cost)
{
    cost = 0.0f;

    // Simulate system with this control sequence
    Matrix<float, 6, PREDICTION_HORIZON> temp_states;
    temp_states.col(0) = _vehicle_state;

    for (int k = 0; k < PREDICTION_HORIZON - 1; k++) {
        Vector<float, 3> control_input;
        if (k < CONTROL_HORIZON) {
            control_input = control_sequence.col(k);
        } else {
            control_input = control_sequence.col(CONTROL_HORIZON - 1);
        }

        Vector<float, 6> state_derivative = vehicle_dynamics_model(temp_states.col(k), control_input);
        temp_states.col(k + 1) = temp_states.col(k) + state_derivative * PREDICTION_TIME_STEP;

        // Calculate slip at this time step
        float vx = temp_states(0, k + 1);
        float wheel_speed_f = temp_states(3, k + 1);
        float wheel_speed_r = temp_states(4, k + 1);

        float slip_front = 0.0f, slip_rear = 0.0f;
        if (vx > 0.1f) {
            slip_front = (wheel_speed_f * _vehicle_params.tire_radius_m - vx) / vx;
            slip_rear = (wheel_speed_r * _vehicle_params.tire_radius_m - vx) / vx;
        }

        // Slip tracking cost
        float slip_error = fabsf(slip_front) + fabsf(slip_rear);
        cost += _mpc_params.weight_slip_tracking * slip_error * slip_error;

        // Stability cost (lateral acceleration)
        float vy = temp_states(1, k + 1);
        float yaw_rate = temp_states(2, k + 1);
        float lateral_accel = vy * yaw_rate;
        cost += _mpc_params.weight_stability * lateral_accel * lateral_accel;

        // Control effort cost
        if (k < CONTROL_HORIZON) {
            for (int j = 0; j < 3; j++) {
                cost += _mpc_params.weight_control_effort * control_input(j) * control_input(j);
            }
        }

        // Risk penalty
        float risk = calculate_slip_risk(slip_front, slip_rear);
        cost += _mpc_params.weight_risk_penalty * risk * risk;
    }

    // Control rate penalty
    for (int k = 1; k < CONTROL_HORIZON; k++) {
        for (int j = 0; j < 3; j++) {
            float control_rate = control_sequence(j, k) - control_sequence(j, k-1);
            cost += _mpc_params.weight_control_rate * control_rate * control_rate;
        }
    }
}

bool PredictiveTractionControl::check_constraints(const Matrix<float, 3, CONTROL_HORIZON>& control_sequence)
{
    for (int k = 0; k < CONTROL_HORIZON; k++) {
        // Torque distribution constraints
        if (control_sequence(0, k) < -1.0f || control_sequence(0, k) > 1.0f) {
            return false;
        }

        // Steering correction constraints
        if (fabsf(control_sequence(1, k)) > 0.5f) {
            return false;
        }

        // Brake intervention constraints
        if (control_sequence(2, k) < 0.0f || control_sequence(2, k) > 1.0f) {
            return false;
        }

        // Rate constraints
        if (k > 0) {
            float torque_rate = fabsf(control_sequence(0, k) - control_sequence(0, k-1)) / PREDICTION_TIME_STEP;
            if (torque_rate > _max_torque_rate.get()) {
                return false;
            }

            float steering_rate = fabsf(control_sequence(1, k) - control_sequence(1, k-1)) / PREDICTION_TIME_STEP;
            if (steering_rate > _max_steering_rate.get()) {
                return false;
            }
        }
    }

    return true;
}

float PredictiveTractionControl::calculate_slip_risk(float slip_front, float slip_rear)
{
    float max_slip = fmaxf(fabsf(slip_front), fabsf(slip_rear));

    if (max_slip < _slip_warning_threshold.get()) {
        return 0.0f;
    } else if (max_slip < _slip_critical_threshold.get()) {
        return (max_slip - _slip_warning_threshold.get()) /
               (_slip_critical_threshold.get() - _slip_warning_threshold.get());
    } else {
        return 1.0f;
    }
}

void PredictiveTractionControl::apply_mpc_solution()
{
    if (!_prediction_active) {
        return;
    }

    // Apply first control action from optimal sequence
    _current_control = _optimal_control_sequence.col(0);

    // Determine if intervention is required
    _current_risk_level = _predicted_risk(1); // Risk one step ahead
    _intervention_active = (_current_risk_level > STABILITY_WARNING_THRESHOLD);

    // Publish traction control command
    traction_control_s traction_cmd{};
    traction_cmd.timestamp = hrt_absolute_time();
    traction_cmd.torque_distribution = _current_control(0);

    // Apply steering correction with safety limits
    float steering_correction = _current_control(1);
    const float MAX_STEERING_CORRECTION = 0.2f; // 0.2 rad (~11.5 degrees) max correction
    steering_correction = math::constrain(steering_correction, -MAX_STEERING_CORRECTION, MAX_STEERING_CORRECTION);
    traction_cmd.steering_correction_rad = steering_correction;

    traction_cmd.slip_ratio_front = _predicted_slip_front(1);
    traction_cmd.slip_ratio_rear = _predicted_slip_rear(1);
    traction_cmd.target_slip_ratio = 0.1f; // Target optimal slip ratio
    traction_cmd.traction_control_active = _intervention_active;
    traction_cmd.slip_detected = (_current_risk_level > 0.1f);
    traction_cmd.intervention_active = _intervention_active;

    // Set traction mode based on risk level
    if (_current_risk_level < 0.1f) {
        traction_cmd.traction_mode = 1; // Eco mode
        traction_cmd.intervention_level = 0; // No intervention
    } else if (_current_risk_level < 0.2f) {
        traction_cmd.traction_mode = 2; // Normal mode
        traction_cmd.intervention_level = 1; // Mild intervention
    } else if (_current_risk_level < STABILITY_WARNING_THRESHOLD) {
        traction_cmd.traction_mode = 2; // Normal mode
        traction_cmd.intervention_level = 2; // Moderate intervention
    } else {
        traction_cmd.traction_mode = 3; // Aggressive mode
        traction_cmd.intervention_level = 3; // Aggressive intervention
    }

    // Set surface friction estimate from terrain model
    traction_cmd.surface_friction = _vehicle_params.max_friction_coeff;

    _traction_control_pub.publish(traction_cmd);

    if (_intervention_active) {
        _last_intervention_time = hrt_absolute_time();
        _performance.interventions_count++;
    }

    // Shift control sequence for next iteration
    for (int k = 0; k < CONTROL_HORIZON - 1; k++) {
        _optimal_control_sequence.col(k) = _optimal_control_sequence.col(k + 1);
    }
    // Last control action remains the same
}

void PredictiveTractionControl::update_learning_model()
{
    // Simple learning algorithm to improve prediction accuracy
    // In practice, this could be a neural network or other ML model

    // Calculate prediction error from last cycle
    float actual_slip_front = _sensor_data.slip_front;
    float actual_slip_rear = _sensor_data.slip_rear;

    if (_learning.learning_samples > 0) {
        float predicted_slip_front = _predicted_slip_front(1);
        float predicted_slip_rear = _predicted_slip_rear(1);

        float prediction_error = fabsf(actual_slip_front - predicted_slip_front) +
                                fabsf(actual_slip_rear - predicted_slip_rear);

        // Update prediction error history
        for (int i = 19; i > 0; i--) {
            _learning.prediction_error_history[i] = _learning.prediction_error_history[i-1];
        }
        _learning.prediction_error_history[0] = prediction_error;

        // Calculate prediction accuracy
        float avg_error = 0.0f;
        for (int i = 0; i < 20; i++) {
            avg_error += _learning.prediction_error_history[i];
        }
        avg_error /= 20.0f;
        _performance.prediction_accuracy = 1.0f - fminf(avg_error, 1.0f);

        // Adapt terrain model based on prediction error
        if (avg_error > 0.1f) {
            _learning.terrain_adaptation_factor *= (1.0f - _learning.adaptation_rate);
        } else {
            _learning.terrain_adaptation_factor *= (1.0f + _learning.adaptation_rate * 0.1f);
        }

        _learning.terrain_adaptation_factor = math::constrain(_learning.terrain_adaptation_factor, 0.5f, 2.0f);
    }

    _learning.learning_samples++;
}

void PredictiveTractionControl::adapt_control_parameters()
{
    // Adapt MPC weights based on terrain and loading conditions

    // Increase stability weight on slippery terrain
    if (_sensor_data.terrain_friction < 0.4f) {
        _mpc_params.weight_stability = _weight_stability.get() * 1.5f;
        _mpc_params.weight_slip_tracking = _weight_slip.get() * 1.2f;
    } else {
        _mpc_params.weight_stability = _weight_stability.get();
        _mpc_params.weight_slip_tracking = _weight_slip.get();
    }

    // Adapt based on load
    if (_sensor_data.payload_mass_kg > 5000.0f) {
        _mpc_params.weight_control_effort = _weight_control.get() * 0.8f; // More aggressive control
    } else {
        _mpc_params.weight_control_effort = _weight_control.get();
    }

    // Apply learning adaptation factor
    _mpc_params.weight_slip_tracking *= _learning.terrain_adaptation_factor;
}

void PredictiveTractionControl::publish_prediction_results()
{
    _prediction_output.timestamp = hrt_absolute_time();

    // Publish current predictions (first element of prediction vectors)
    _prediction_output.predicted_slip_front = _predicted_slip_front(0);
    _prediction_output.predicted_slip_rear = _predicted_slip_rear(0);
    _prediction_output.optimal_torque_front = _current_control(0) > 0 ? _current_control(0) : 0.0f;
    _prediction_output.optimal_torque_rear = _current_control(0) < 0 ? -_current_control(0) : 0.0f;

    // Set control status
    _prediction_output.predictive_active = _prediction_active;
    _prediction_output.mpc_converged = true; // Simplified for now
    _prediction_output.prediction_horizon_s = PREDICTION_HORIZON * PREDICTION_TIME_STEP;
    _prediction_output.cost_function_value = 0.0f; // TODO: implement cost calculation
    _prediction_output.slip_tracking_error = 0.0f; // TODO: implement error calculation
    _prediction_output.torque_smoothness = 1.0f; // TODO: implement smoothness metric
    _prediction_output.mpc_status = 0; // 0 = converged
    _prediction_output.solver_iterations = 1;
    _prediction_output.solve_time_ms = 1.0f;

    _predictive_traction_pub.publish(_prediction_output);
}

void PredictiveTractionControl::reset_mpc_state()
{
    _optimal_control_sequence.setZero();
    _predicted_states.setZero();
    _predicted_slip_front.setZero();
    _predicted_slip_rear.setZero();
    _predicted_risk.setZero();
    _prediction_active = false;
    _intervention_active = false;
    _current_risk_level = 0.0f;
}

void PredictiveTractionControl::set_learning_enabled(bool enabled)
{
    _learning.learning_active = enabled;
    _learning_enable.set(enabled);
    PX4_INFO("Learning %s", enabled ? "enabled" : "disabled");
}

void PredictiveTractionControl::reset_learned_parameters()
{
    // Reset learning state
    _learning.learning_active = false;
    _learning.learning_samples = 0;
    _learning.terrain_adaptation_factor = 1.0f;
    _learning.adaptation_rate = 0.1f;

    // Reset learned friction model to defaults
    for (int i = 0; i < 10; i++) {
        _learning.learned_friction_model[i] = 0.6f;
    }

    // Reset prediction error history
    for (int i = 0; i < 20; i++) {
        _learning.prediction_error_history[i] = 0.0f;
    }

    // Reset vehicle parameters to defaults
    _vehicle_params.max_friction_coeff = 0.8f;       // Default friction coefficient
    _vehicle_params.cornering_stiffness_n_rad = 50000.0f; // Default tire stiffness

    // Reset current risk assessment
    _current_risk_level = 0.0f;

    PX4_INFO("Learned parameters reset to defaults");
}

int PredictiveTractionControl::print_status()
{
    PX4_INFO("Predictive Traction Control Status:");
    PX4_INFO("  Prediction Active: %s", _prediction_active ? "YES" : "NO");
    PX4_INFO("  Intervention Active: %s", _intervention_active ? "YES" : "NO");
    PX4_INFO("  Current Risk Level: %.3f", (double)_current_risk_level);
    PX4_INFO("  Torque Distribution: %.3f", (double)_current_control(0));
    PX4_INFO("  Steering Correction: %.3f rad", (double)_current_control(1));
    PX4_INFO("  Brake Intervention: %.3f", (double)_current_control(2));
    PX4_INFO("  Performance:");
    PX4_INFO("    Prediction Accuracy: %.1f%%", (double)(_performance.prediction_accuracy * 100.0f));
    PX4_INFO("    Avg Computation Time: %.1f ms", (double)_performance.avg_computation_time_ms);
    PX4_INFO("    Total Interventions: %lu", (unsigned long)_performance.interventions_count);
    PX4_INFO("    Terrain Adaptation Factor: %.3f", (double)_learning.terrain_adaptation_factor);
    PX4_INFO("  Learning:");
    PX4_INFO("    Learning Active: %s", _learning.learning_active ? "YES" : "NO");
    PX4_INFO("    Learning Samples: %lu", (unsigned long)_learning.learning_samples);
    PX4_INFO("    Adaptation Rate: %.3f", (double)_learning.adaptation_rate);

    return 0;
}
