/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "chassis_mpc_controller.hpp"
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;
using namespace matrix;

ChassisMPCController::ChassisMPCController()
{
    // Initialize control sequence with zeros
    control_sequence.setZero();
    predicted_states.setZero();
}

void ChassisMPCController::init(float wheelbase, float max_velocity, float max_steering)
{
    this->wheelbase = wheelbase;
    this->max_velocity = max_velocity;
    this->max_steering = max_steering;
    this->max_acceleration = max_velocity / 2.0f;  // Reasonable default
}

void ChassisMPCController::set_weights(const Vector<float, STATE_DIM> &state_weights,
                                       const Vector<float, CONTROL_DIM> &control_weights)
{
    Q = state_weights;
    R = control_weights;
}

Vector<float, ChassisMPCController::CONTROL_DIM> ChassisMPCController::solve(
    const Vector<float, STATE_DIM> &current_state,
    const chassis_trajectory_setpoint_s &reference_point,
    float dt)
{
    // Convert reference to state vector
    Vector<float, STATE_DIM> reference_state;
    reference_state(0) = reference_point.x_position;   // x
    reference_state(1) = reference_point.y_position;   // y
    reference_state(2) = reference_point.yaw;          // theta

    // Calculate velocity magnitude from x,y,z components
    float velocity_magnitude = sqrtf(reference_point.x_velocity * reference_point.x_velocity +
                                    reference_point.y_velocity * reference_point.y_velocity +
                                    reference_point.z_velocity * reference_point.z_velocity);
    reference_state(3) = velocity_magnitude;           // v

    // Solve MPC problem
    Vector<float, CONTROL_DIM> optimal_control = solve_mpc_problem(current_state, reference_state);

    // Apply constraints
    apply_control_constraints(optimal_control);

    // Store for next iteration (warm start)
    previous_control = optimal_control;

    return optimal_control;
}

Vector<float, ChassisMPCController::STATE_DIM> ChassisMPCController::dynamics(
    const Vector<float, STATE_DIM> &state,
    const Vector<float, CONTROL_DIM> &control,
    float dt)
{
    // Bicycle model kinematics
    // state = [x, y, theta, v]
    // control = [a, delta]

    float x = state(0);
    float y = state(1);
    float theta = state(2);
    float v = state(3);
    float a = control(0);
    float delta = control(1);

    Vector<float, STATE_DIM> state_dot;
    state_dot(0) = v * cosf(theta);                    // x_dot
    state_dot(1) = v * sinf(theta);                    // y_dot
    state_dot(2) = v * tanf(delta) / wheelbase;        // theta_dot
    state_dot(3) = a;                                  // v_dot

    // Euler integration
    return state + state_dot * dt;
}

Vector<float, ChassisMPCController::CONTROL_DIM> ChassisMPCController::solve_mpc_problem(
    const Vector<float, STATE_DIM> &initial_state,
    const Vector<float, STATE_DIM> &reference_state)
{
    // Initialize control sequence with previous solution (warm start)
    Matrix<float, CONTROL_DIM, HORIZON_LENGTH> best_control_sequence = control_sequence;

    // Shift previous solution
    for (int i = 0; i < HORIZON_LENGTH - 1; ++i) {
        best_control_sequence.setColumn(i, control_sequence.col(i + 1));
    }
    best_control_sequence.setColumn(HORIZON_LENGTH - 1, previous_control);

    float best_cost = INFINITY;

    // Simple gradient descent optimization
    for (int iter = 0; iter < max_iterations; ++iter) {
        // Forward simulate with current control sequence
        forward_simulate(initial_state, best_control_sequence);

        // Calculate cost
        float cost = calculate_cost(predicted_states, best_control_sequence, reference_state);

        if (cost < best_cost) {
            best_cost = cost;
            control_sequence = best_control_sequence;
        }

        // Compute gradient numerically and update controls
        Matrix<float, CONTROL_DIM, HORIZON_LENGTH> gradient;
        gradient.setZero();

        const float epsilon = 0.01f;

        for (int i = 0; i < HORIZON_LENGTH; ++i) {
            for (int j = 0; j < CONTROL_DIM; ++j) {
                // Positive perturbation
                Matrix<float, CONTROL_DIM, HORIZON_LENGTH> perturbed_control = best_control_sequence;
                perturbed_control(j, i) += epsilon;

                forward_simulate(initial_state, perturbed_control);
                float cost_plus = calculate_cost(predicted_states, perturbed_control, reference_state);

                // Negative perturbation
                perturbed_control = best_control_sequence;
                perturbed_control(j, i) -= epsilon;

                forward_simulate(initial_state, perturbed_control);
                float cost_minus = calculate_cost(predicted_states, perturbed_control, reference_state);

                // Central difference
                gradient(j, i) = (cost_plus - cost_minus) / (2.0f * epsilon);
            }
        }

        // Update control sequence
        best_control_sequence = best_control_sequence - gradient * step_size;

        // Apply constraints to each control
        for (int i = 0; i < HORIZON_LENGTH; ++i) {
            Vector<float, CONTROL_DIM> control_i = best_control_sequence.col(i);
            apply_control_constraints(control_i);
            best_control_sequence.setColumn(i, control_i);
        }
    }

    // Store best solution
    control_sequence = best_control_sequence;
    forward_simulate(initial_state, control_sequence);

    // Return first control action
    return control_sequence.col(0);
}

void ChassisMPCController::forward_simulate(
    const Vector<float, STATE_DIM> &initial_state,
    const Matrix<float, CONTROL_DIM, HORIZON_LENGTH> &control_sequence)
{
    Vector<float, STATE_DIM> state = initial_state;
    predicted_states.setColumn(0, state);

    for (int i = 0; i < HORIZON_LENGTH - 1; ++i) {
        Vector<float, CONTROL_DIM> control = control_sequence.col(i);
        state = dynamics(state, control, DT);
        predicted_states.setColumn(i + 1, state);
    }
}

float ChassisMPCController::calculate_cost(
    const Matrix<float, STATE_DIM, HORIZON_LENGTH> &states,
    const Matrix<float, CONTROL_DIM, HORIZON_LENGTH> &controls,
    const Vector<float, STATE_DIM> &reference_state)
{
    float cost = 0.0f;

    // State tracking cost
    for (int i = 0; i < HORIZON_LENGTH; ++i) {
        Vector<float, STATE_DIM> state_error = states.col(i) - reference_state;

        // Wrap angle error
        state_error(2) = wrap_pi(state_error(2));

        // Weighted state cost
        for (int j = 0; j < STATE_DIM; ++j) {
            cost += Q(j) * state_error(j) * state_error(j);
        }
    }

    // Control effort cost
    for (int i = 0; i < HORIZON_LENGTH; ++i) {
        Vector<float, CONTROL_DIM> control = controls.col(i);

        for (int j = 0; j < CONTROL_DIM; ++j) {
            cost += R(j) * control(j) * control(j);
        }
    }

    // Control rate cost (smoothness)
    for (int i = 1; i < HORIZON_LENGTH; ++i) {
        Vector<float, CONTROL_DIM> control_rate = controls.col(i) - controls.col(i - 1);

        for (int j = 0; j < CONTROL_DIM; ++j) {
            cost += R(j) * 0.1f * control_rate(j) * control_rate(j);  // Small weight on rate
        }
    }

    return cost;
}

void ChassisMPCController::apply_control_constraints(Vector<float, CONTROL_DIM> &control)
{
    // Acceleration limits
    control(0) = math::constrain(control(0), -max_acceleration, max_acceleration);

    // Steering angle limits
    control(1) = math::constrain(control(1), -max_steering, max_steering);
}
