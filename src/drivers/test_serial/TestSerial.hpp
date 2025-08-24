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

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <drivers/drv_hrt.h>
#include <termios.h>
#include <fcntl.h>
#include <cstddef>
#include <cstdint>

/**
 * @file TestSerial.hpp
 * Serial port test driver for PX4
 */

// Using declarations
using namespace time_literals;

class TestSerial : public ModuleBase<TestSerial>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	TestSerial(const char *port);
	~TestSerial() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	void Run() override;

	int init();

	void stop();

private:
	enum class TestType {
		BASIC_COMM,
		LOOPBACK,
		DATA_INTEGRITY,
		BAUD_RATE,
		STRESS_TEST,
		BUFFER_TEST,
		ALL_TESTS
	};

	enum class TestResult {
		NOT_RUN,
		PASSED,
		FAILED,
		RUNNING
	};

private:

	struct TestPattern {
		const char *name;
		const uint8_t *pattern;
		size_t length;
		uint32_t expected_crc;
	};

	struct TestStats {
		uint32_t bytes_sent{0};
		uint32_t bytes_received{0};
		uint32_t packets_sent{0};
		uint32_t packets_received{0};
		uint32_t errors{0};
		hrt_abstime start_time{0};
		hrt_abstime end_time{0};
	};

	// Serial port management
	int openSerialPort(const char *device, int baudrate);
	void closeSerialPort();
	bool configureSerialPort(int fd, int baudrate);

	// Test execution methods
	TestResult runBasicCommunicationTest();
	TestResult runLoopbackTest();
	TestResult runDataIntegrityTest();
	TestResult runBaudRateTest();
	TestResult runStressTest();
	TestResult runBufferTest();
	void runAllTests();

	// Helper methods
	bool sendTestData(const uint8_t *data, size_t length);
	bool receiveTestData(uint8_t *buffer, size_t expected_length, uint32_t timeout_ms = 1000);
	uint32_t calculateCRC(const uint8_t *data, size_t length);
	void printTestResults();
	void resetStats();

	// Test patterns
	static const TestPattern _test_patterns[];
	static const size_t _num_test_patterns;

	// Configuration
	char _port[32]{};
	int _serial_fd{-1};
	int _baudrate{115200};
	TestType _current_test{TestType::BASIC_COMM};
	bool _should_exit{false};
	bool _test_completed{false};

	// Test state
	TestResult _test_results[static_cast<int>(TestType::ALL_TESTS)]{};
	TestStats _current_stats{};
	uint32_t _test_iteration{0};

	// Timing
	static constexpr uint32_t SCHEDULE_INTERVAL_US = 100_ms;
	static constexpr uint32_t TEST_TIMEOUT_MS = 5000;
};

// Test pattern definitions
namespace test_patterns
{
	// Simple alternating pattern
	static const uint8_t PATTERN_ALTERNATING[] = {0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55, 0xAA, 0x55};

	// Incremental pattern
	static const uint8_t PATTERN_INCREMENTAL[] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
	                                               0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

	// All zeros
	static const uint8_t PATTERN_ZEROS[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

	// All ones
	static const uint8_t PATTERN_ONES[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

	// Random-like pattern
	static const uint8_t PATTERN_RANDOM[] = {0x5A, 0x3C, 0xF1, 0x8E, 0x42, 0x97, 0x6B, 0xD4};
}
