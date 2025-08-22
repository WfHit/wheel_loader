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

#pragma once

#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>
#include <uORB/topics/chassis_trajectory_setpoint.h>
#include "trajectory_types.hpp"

namespace wheel_loader
{

/**
 * TinyMPC implementation for chassis trajectory following
 *
 * State vector: [x, y, theta, v]  (position, heading, velocity)
 * Control vector: [a, delta]      (acceleration, steering angle)
 *
 * Uses bicycle model for prediction with embedded MPC solver
 */
class ChassisMPCController
{
public:
    // MPC parameters
    static constexpr int HORIZON_LENGTH = 15;        // Prediction horizon (shorter for real-time)
    static constexpr int STATE_DIM = 4;              // State dimension
    static constexpr int CONTROL_DIM = 2;            // Control dimension
    static constexpr float DT = 0.1f;                // Time step (100ms)

    ChassisMPCController();
    ~ChassisMPCController() = default;

    /**
     * Initialize MPC controller with vehicle parameters
     */
    void init(float wheelbase, float max_velocity, float max_steering);

    /**
     * Update MPC weights for tuning
     */
    void set_weights(const matrix::Vector<float, STATE_DIM> &state_weights,
                     const matrix::Vector<float, CONTROL_DIM> &control_weights);

    /**
     * Solve MPC problem for current state and reference trajectory
     *
     * @param current_state Current vehicle state [x, y, theta, v]
     * @param reference_point Reference trajectory point
     * @param dt Time step
     * @return Optimal control action [acceleration, steering]
     */
    matrix::Vector<float, CONTROL_DIM> solve(
        const matrix::Vector<float, STATE_DIM> &current_state,
        const chassis_trajectory_setpoint_s &reference_point,
        float dt);

    /**
     * Get predicted trajectory from last solution
     */
    matrix::Matrix<float, STATE_DIM, HORIZON_LENGTH> get_predicted_trajectory() const {
        return predicted_states;
    }

private:
    /**
     * Bicycle model dynamics
     * x_dot = v * cos(theta)
     * y_dot = v * sin(theta)
     * theta_dot = v * tan(delta) / L
     * v_dot = a
     */
    matrix::Vector<float, STATE_DIM> dynamics(
        const matrix::Vector<float, STATE_DIM> &state,
        const matrix::Vector<float, CONTROL_DIM> &control,
        float dt);

    /**
     * Simplified MPC solver using gradient descent
     * (Approximation of TinyMPC for embedded systems)
     */
    matrix::Vector<float, CONTROL_DIM> solve_mpc_problem(
        const matrix::Vector<float, STATE_DIM> &initial_state,
        const matrix::Vector<float, STATE_DIM> &reference_state);

    /**
     * Apply box constraints to control inputs
     */
    void apply_control_constraints(matrix::Vector<float, CONTROL_DIM> &control);

    /**
     * Forward simulate trajectory
     */
    void forward_simulate(const matrix::Vector<float, STATE_DIM> &initial_state,
                         const matrix::Matrix<float, CONTROL_DIM, HORIZON_LENGTH> &control_sequence);

    /**
     * Calculate cost function
     */
    float calculate_cost(const matrix::Matrix<float, STATE_DIM, HORIZON_LENGTH> &states,
                        const matrix::Matrix<float, CONTROL_DIM, HORIZON_LENGTH> &controls,
                        const matrix::Vector<float, STATE_DIM> &reference_state);

    // Vehicle parameters
    float wheelbase{2.5f};           // Vehicle wheelbase (m)
    float max_velocity{3.0f};        // Maximum velocity (m/s)
    float max_steering{0.5f};        // Maximum steering angle (rad)
    float max_acceleration{2.0f};    // Maximum acceleration (m/s^2)

    // MPC weights
    matrix::Vector<float, STATE_DIM> Q{10.0f, 10.0f, 5.0f, 1.0f};      // State weights
    matrix::Vector<float, CONTROL_DIM> R{1.0f, 10.0f};                 // Control weights

    // MPC state
    matrix::Matrix<float, STATE_DIM, HORIZON_LENGTH> predicted_states{};
    matrix::Matrix<float, CONTROL_DIM, HORIZON_LENGTH> control_sequence{};

    // Previous control for warm start
    matrix::Vector<float, CONTROL_DIM> previous_control{};

    // Solver settings
    int max_iterations{5};           // Limited iterations for real-time
    float step_size{0.1f};           // Gradient descent step size
    float tolerance{1e-2f};          // Convergence tolerance
};

} // namespace wheel_loader
