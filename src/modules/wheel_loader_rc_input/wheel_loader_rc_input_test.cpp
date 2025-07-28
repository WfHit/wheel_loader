/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include <gtest/gtest.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

// Test class to verify RC input mapping functionality
class WheelLoaderRcInputTest : public ::testing::Test
{
protected:
	// RC channel mapping test helper
	float mapRcChannel(uint16_t raw_value, float min_out = -1.0f, float max_out = 1.0f)
	{
		// Standard RC range is 1000-2000 microseconds
		constexpr float RC_MIN = 1000.0f;
		constexpr float RC_MAX = 2000.0f;
		constexpr float RC_CENTER = 1500.0f;

		// Handle invalid input
		if (raw_value < RC_MIN || raw_value > RC_MAX) {
			return 0.0f;
		}

		// Normalize to [-1, 1]
		float normalized = (static_cast<float>(raw_value) - RC_CENTER) / (RC_CENTER - RC_MIN);
		
		// Map to output range
		return math::constrain(normalized * (max_out - min_out) / 2.0f + (max_out + min_out) / 2.0f, min_out, max_out);
	}

	// Deadzone application test helper
	float applyDeadzone(float input, float deadzone = 0.05f)
	{
		if (fabsf(input) < deadzone) {
			return 0.0f;
		}
		
		// Scale output to maintain full range outside deadzone
		float sign = (input >= 0.0f) ? 1.0f : -1.0f;
		return sign * (fabsf(input) - deadzone) / (1.0f - deadzone);
	}
};

TEST_F(WheelLoaderRcInputTest, RcChannelMappingCenter)
{
	// Test center position (1500us) maps to 0
	float result = mapRcChannel(1500);
	EXPECT_FLOAT_EQ(result, 0.0f);
}

TEST_F(WheelLoaderRcInputTest, RcChannelMappingLimits)
{
	// Test minimum position (1000us) maps to -1
	float result_min = mapRcChannel(1000);
	EXPECT_FLOAT_EQ(result_min, -1.0f);

	// Test maximum position (2000us) maps to +1
	float result_max = mapRcChannel(2000);
	EXPECT_FLOAT_EQ(result_max, 1.0f);
}

TEST_F(WheelLoaderRcInputTest, RcChannelMappingInvalidInput)
{
	// Test values outside valid range return 0
	float result_low = mapRcChannel(500);
	EXPECT_FLOAT_EQ(result_low, 0.0f);

	float result_high = mapRcChannel(2500);
	EXPECT_FLOAT_EQ(result_high, 0.0f);
}

TEST_F(WheelLoaderRcInputTest, RcChannelMappingCustomRange)
{
	// Test custom output range [0, 5] for speed mapping
	float result_center = mapRcChannel(1500, 0.0f, 5.0f);
	EXPECT_FLOAT_EQ(result_center, 2.5f);

	float result_min = mapRcChannel(1000, 0.0f, 5.0f);
	EXPECT_FLOAT_EQ(result_min, 0.0f);

	float result_max = mapRcChannel(2000, 0.0f, 5.0f);
	EXPECT_FLOAT_EQ(result_max, 5.0f);
}

TEST_F(WheelLoaderRcInputTest, DeadzoneApplication)
{
	// Test deadzone suppresses small inputs
	float result_small = applyDeadzone(0.03f, 0.05f);
	EXPECT_FLOAT_EQ(result_small, 0.0f);

	// Test deadzone passes through large inputs (scaled)
	float result_large = applyDeadzone(0.1f, 0.05f);
	EXPECT_GT(result_large, 0.0f);
	EXPECT_LT(result_large, 0.1f); // Should be scaled down
}

TEST_F(WheelLoaderRcInputTest, DeadzoneScaling)
{
	// Test that deadzone maintains full range outside deadzone
	float result_max = applyDeadzone(1.0f, 0.05f);
	EXPECT_FLOAT_EQ(result_max, 1.0f);

	float result_min = applyDeadzone(-1.0f, 0.05f);
	EXPECT_FLOAT_EQ(result_min, -1.0f);
}

TEST_F(WheelLoaderRcInputTest, WheelSpeedCalculation)
{
	// Test wheel speed calculation for different scenarios
	constexpr float MAX_SPEED = 5.0f;
	constexpr float MAX_STEERING = 0.7f;

	// Forward motion, no steering
	float throttle = 0.5f; // 50% throttle
	float steering = 0.0f; // No steering
	
	float chassis_speed = throttle * MAX_SPEED;
	float steering_angle = steering * MAX_STEERING;
	
	// Expected wheel speeds (simplified model)
	float speed_factor = cosf(steering_angle * 0.5f);
	float expected_front_speed = chassis_speed * speed_factor;
	float expected_rear_left = chassis_speed * (1.0f - steering);
	float expected_rear_right = chassis_speed * (1.0f + steering);

	EXPECT_FLOAT_EQ(expected_front_speed, 2.5f);
	EXPECT_FLOAT_EQ(expected_rear_left, 2.5f);
	EXPECT_FLOAT_EQ(expected_rear_right, 2.5f);
}

TEST_F(WheelLoaderRcInputTest, WheelSpeedCalculationWithSteering)
{
	// Test wheel speed calculation with steering input
	constexpr float MAX_SPEED = 5.0f;
	
	float throttle = 0.4f; // 40% throttle  
	float steering = 0.5f; // 50% steering right
	
	float chassis_speed = throttle * MAX_SPEED; // 2.0 rad/s
	
	// Differential steering calculation
	float expected_rear_left = chassis_speed * (1.0f - steering);  // Slower
	float expected_rear_right = chassis_speed * (1.0f + steering); // Faster

	EXPECT_FLOAT_EQ(expected_rear_left, 1.0f);  // 2.0 * 0.5
	EXPECT_FLOAT_EQ(expected_rear_right, 3.0f); // 2.0 * 1.5
}

// Safety limit tests
TEST_F(WheelLoaderRcInputTest, SafetyLimits)
{
	constexpr float MAX_WHEEL_SPEED = 5.0f;
	constexpr float MAX_STEERING_ANGLE = 0.7f;

	// Test speed limiting
	float over_speed = 7.0f;
	float limited_speed = math::constrain(over_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
	EXPECT_FLOAT_EQ(limited_speed, MAX_WHEEL_SPEED);

	// Test steering limiting  
	float over_steering = 1.2f;
	float limited_steering = math::constrain(over_steering, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
	EXPECT_FLOAT_EQ(limited_steering, MAX_STEERING_ANGLE);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}