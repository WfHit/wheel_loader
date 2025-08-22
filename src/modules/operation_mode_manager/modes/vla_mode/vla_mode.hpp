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

#include "../operation_mode_base.hpp"
#include "vla_trajectory_decomposer.hpp"

namespace wheel_loader
{

class VlaMode : public OperationModeBase
{
public:
	VlaMode() = default;
	~VlaMode() = default;

	bool init() override;
	bool activate() override;
	void deactivate() override;
	void update(float dt) override;
	ChassisTrajectorySetpoint get_chassis_setpoint() const override;
	BucketTrajectorySetpoint get_bucket_setpoint() const override;

	/**
	 * Set new VLA trajectory point
	 */
	void set_vla_trajectory(const VlaTrajectoryPoint &trajectory);

	/**
	 * Update current vehicle state
	 */
	void update_vehicle_state(const Vector3f &position, float yaw);

	/**
	 * Check if VLA trajectory is still valid
	 */
	bool is_trajectory_valid() const;

private:
	/**
	 * Decompose VLA trajectory into chassis and bucket components
	 */
	bool decompose_trajectory();

	// Components
	VlaTrajectoryDecomposer trajectory_decomposer;

	// Current VLA trajectory
	VlaTrajectoryPoint vla_trajectory{};

	// Current vehicle state
	Vector3f current_position{};
	float current_yaw{0.0f};

	// Timing
	hrt_abstime last_trajectory_update{0};
	static constexpr hrt_abstime TRAJECTORY_TIMEOUT{500_ms};
};

} // namespace wheel_loader
