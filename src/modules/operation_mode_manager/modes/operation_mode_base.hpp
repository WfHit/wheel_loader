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

#include "../common/trajectory_types.hpp"

namespace wheel_loader
{

class OperationModeBase
{
public:
	OperationModeBase() = default;
	virtual ~OperationModeBase() = default;

	/**
	 * Initialize the mode
	 */
	virtual bool init() = 0;

	/**
	 * Activate the mode
	 */
	virtual bool activate() = 0;

	/**
	 * Deactivate the mode
	 */
	virtual void deactivate() = 0;

	/**
	 * Update the mode logic
	 * @param dt Time since last update (s)
	 */
	virtual void update(float dt) = 0;

	/**
	 * Get chassis trajectory setpoint
	 */
	virtual ChassisTrajectorySetpoint get_chassis_setpoint() const = 0;

	/**
	 * Get bucket trajectory setpoint
	 */
	virtual BucketTrajectorySetpoint get_bucket_setpoint() const = 0;

	/**
	 * Check if mode is active
	 */
	bool is_active() const { return active; }

protected:
	bool active{false};
	ChassisTrajectorySetpoint chassis_setpoint{};
	BucketTrajectorySetpoint bucket_setpoint{};
};

} // namespace wheel_loader
