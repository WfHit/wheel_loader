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

#include "TestSerial.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/WorkQueueManager.hpp>
#include <drivers/drv_hrt.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <cstring>
#include <cstddef>
#include <fcntl.h>
#include <termios.h>

using namespace test_patterns;

// Test pattern definitions
const TestSerial::TestPattern TestSerial::_test_patterns[] = {
	{"Alternating", PATTERN_ALTERNATING, sizeof(PATTERN_ALTERNATING), 0},
	{"Incremental", PATTERN_INCREMENTAL, sizeof(PATTERN_INCREMENTAL), 0},
	{"Zeros", PATTERN_ZEROS, sizeof(PATTERN_ZEROS), 0},
	{"Ones", PATTERN_ONES, sizeof(PATTERN_ONES), 0},
	{"Random", PATTERN_RANDOM, sizeof(PATTERN_RANDOM), 0}
};

const size_t TestSerial::_num_test_patterns = sizeof(_test_patterns) / sizeof(_test_patterns[0]);

TestSerial::TestSerial(const char *port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
	strncpy(_port, port, sizeof(_port) - 1);
	_port[sizeof(_port) - 1] = '\0';

	// Initialize test results
	for (int i = 0; i < static_cast<int>(TestType::ALL_TESTS); i++) {
		_test_results[i] = TestResult::NOT_RUN;
	}
}

TestSerial::~TestSerial()
{
	stop();
}

int TestSerial::init()
{
	// Use the port passed in constructor
	PX4_INFO("Serial test driver initialized with device: %s", _port);

	// Run all tests once automatically
	_current_test = TestType::ALL_TESTS;
	_test_completed = true;
	ScheduleOnInterval(SCHEDULE_INTERVAL_US);

	return PX4_OK;
}

void TestSerial::Run()
{
	if (_should_exit) {
		ScheduleClear();
		return;
	}

	// Run all tests automatically once when started
	if (_current_test == TestType::ALL_TESTS && !_test_completed) {
		runAllTests();
		_test_completed = true;
		// Stop automatic execution after running tests
	}

	// Keep scheduled but do nothing - waiting for manual commands
	// This keeps the driver alive for status/stop commands
}

int TestSerial::openSerialPort(const char *device, int baudrate)
{
	int fd = open(device, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (fd < 0) {
		PX4_ERR("Failed to open serial port %s: %s", device, strerror(errno));
		return -1;
	}

	if (!configureSerialPort(fd, baudrate)) {
		close(fd);
		return -1;
	}

	return fd;
}

bool TestSerial::configureSerialPort(int fd, int baudrate)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) != 0) {
		PX4_ERR("Error getting serial port attributes: %s", strerror(errno));
		return false;
	}

	// Set baud rate
	speed_t speed;
	switch (baudrate) {
	case 9600:    speed = B9600; break;
	case 19200:   speed = B19200; break;
	case 38400:   speed = B38400; break;
	case 57600:   speed = B57600; break;
	case 115200:  speed = B115200; break;
	case 230400:  speed = B230400; break;
	case 460800:  speed = B460800; break;
	case 921600:  speed = B921600; break;
	default:
		PX4_ERR("Unsupported baud rate: %d", baudrate);
		return false;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	// Configure port settings
	tty.c_cflag &= ~PARENB;  // No parity
	tty.c_cflag &= ~CSTOPB;  // One stop bit
	tty.c_cflag &= ~CSIZE;   // Clear size bits
	tty.c_cflag |= CS8;      // 8 data bits
	tty.c_cflag &= ~CRTSCTS; // No hardware flow control
	tty.c_cflag |= CREAD | CLOCAL; // Enable reading and ignore modem control lines

	tty.c_lflag &= ~ICANON;  // Non-canonical mode
	tty.c_lflag &= ~ECHO;    // No echo
	tty.c_lflag &= ~ECHOE;   // No echo erase
	tty.c_lflag &= ~ECHONL;  // No echo newline
	tty.c_lflag &= ~ISIG;    // No signal processing

	tty.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
	tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	tty.c_oflag &= ~OPOST;   // No output processing
	tty.c_oflag &= ~ONLCR;   // No CR to NL translation

	// Set timeouts
	tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds)
	tty.c_cc[VMIN] = 0;      // No minimum number of characters

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		PX4_ERR("Error setting serial port attributes: %s", strerror(errno));
		return false;
	}

	return true;
}

void TestSerial::closeSerialPort()
{
	if (_serial_fd >= 0) {
		close(_serial_fd);
		_serial_fd = -1;
	}
}

TestSerial::TestResult TestSerial::runBasicCommunicationTest()
{
	PX4_INFO("Running basic communication test...");

	_serial_fd = openSerialPort(_port, _baudrate);
	if (_serial_fd < 0) {
		return TestResult::FAILED;
	}

	resetStats();
	_current_stats.start_time = hrt_absolute_time();

	// Send and receive a simple test message
	const char *test_msg = "PX4_SERIAL_TEST";
	size_t msg_len = strlen(test_msg);

	if (!sendTestData(reinterpret_cast<const uint8_t *>(test_msg), msg_len)) {
		closeSerialPort();
		return TestResult::FAILED;
	}

	uint8_t rx_buffer[64];
	if (!receiveTestData(rx_buffer, msg_len, 1000)) {
		closeSerialPort();
		return TestResult::FAILED;
	}

	_current_stats.end_time = hrt_absolute_time();
	closeSerialPort();

	// Verify received data
	if (memcmp(test_msg, rx_buffer, msg_len) == 0) {
		PX4_INFO("Basic communication test: PASSED");
		return TestResult::PASSED;
	} else {
		PX4_ERR("Basic communication test: FAILED - Data mismatch");
		return TestResult::FAILED;
	}
}

TestSerial::TestResult TestSerial::runLoopbackTest()
{
	PX4_INFO("Running loopback test...");

	_serial_fd = openSerialPort(_port, _baudrate);
	if (_serial_fd < 0) {
		return TestResult::FAILED;
	}

	resetStats();
	_current_stats.start_time = hrt_absolute_time();

	// Test each pattern
	for (size_t i = 0; i < _num_test_patterns; i++) {
		const TestPattern &pattern = _test_patterns[i];

		if (!sendTestData(pattern.pattern, pattern.length)) {
			closeSerialPort();
			return TestResult::FAILED;
		}

		uint8_t rx_buffer[64];
		if (!receiveTestData(rx_buffer, pattern.length, 1000)) {
			closeSerialPort();
			return TestResult::FAILED;
		}

		if (memcmp(pattern.pattern, rx_buffer, pattern.length) != 0) {
			PX4_ERR("Loopback test failed for pattern: %s", pattern.name);
			closeSerialPort();
			return TestResult::FAILED;
		}

		PX4_INFO("Loopback test passed for pattern: %s", pattern.name);
	}

	_current_stats.end_time = hrt_absolute_time();
	closeSerialPort();

	PX4_INFO("Loopback test: PASSED");
	return TestResult::PASSED;
}

TestSerial::TestResult TestSerial::runDataIntegrityTest()
{
	PX4_INFO("Running data integrity test...");

	_serial_fd = openSerialPort(_port, _baudrate);
	if (_serial_fd < 0) {
		return TestResult::FAILED;
	}

	resetStats();
	_current_stats.start_time = hrt_absolute_time();

	// Generate a larger test pattern with CRC
	uint8_t test_data[256];
	for (int i = 0; i < 256; i++) {
		test_data[i] = i & 0xFF;
	}

	uint32_t expected_crc = calculateCRC(test_data, sizeof(test_data));

	if (!sendTestData(test_data, sizeof(test_data))) {
		closeSerialPort();
		return TestResult::FAILED;
	}

	uint8_t rx_buffer[256];
	if (!receiveTestData(rx_buffer, sizeof(rx_buffer), 2000)) {
		closeSerialPort();
		return TestResult::FAILED;
	}

	uint32_t received_crc = calculateCRC(rx_buffer, sizeof(rx_buffer));

	_current_stats.end_time = hrt_absolute_time();
	closeSerialPort();

	if (expected_crc == received_crc) {
		PX4_INFO("Data integrity test: PASSED (CRC: 0x%08lX)", (unsigned long)expected_crc);
		return TestResult::PASSED;
	} else {
		PX4_ERR("Data integrity test: FAILED (Expected CRC: 0x%08lX, Received: 0x%08lX)",
		        (unsigned long)expected_crc, (unsigned long)received_crc);
		return TestResult::FAILED;
	}
}

TestSerial::TestResult TestSerial::runBaudRateTest()
{
	PX4_INFO("Running baud rate test...");

	const int test_baudrates[] = {9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600};
	const size_t num_baudrates = sizeof(test_baudrates) / sizeof(test_baudrates[0]);

	const char *test_msg = "BAUD_TEST";
	size_t msg_len = strlen(test_msg);

	for (size_t i = 0; i < num_baudrates; i++) {
		int baudrate = test_baudrates[i];
		PX4_INFO("Testing baud rate: %d", baudrate);

		_serial_fd = openSerialPort(_port, baudrate);
		if (_serial_fd < 0) {
			continue; // Skip this baud rate
		}

		bool test_passed = false;
		if (sendTestData(reinterpret_cast<const uint8_t *>(test_msg), msg_len)) {
			uint8_t rx_buffer[32];
			if (receiveTestData(rx_buffer, msg_len, 1000)) {
				if (memcmp(test_msg, rx_buffer, msg_len) == 0) {
					test_passed = true;
				}
			}
		}

		closeSerialPort();

		if (test_passed) {
			PX4_INFO("Baud rate %d: PASSED", baudrate);
		} else {
			PX4_WARN("Baud rate %d: FAILED", baudrate);
		}
	}

	PX4_INFO("Baud rate test: COMPLETED");
	return TestResult::PASSED;
}

TestSerial::TestResult TestSerial::runStressTest()
{
	PX4_INFO("Running stress test...");

	_serial_fd = openSerialPort(_port, _baudrate);
	if (_serial_fd < 0) {
		return TestResult::FAILED;
	}

	resetStats();
	_current_stats.start_time = hrt_absolute_time();

	const int num_iterations = 1000;
	const uint8_t test_byte = 0x5A;

	for (int i = 0; i < num_iterations; i++) {
		if (!sendTestData(&test_byte, 1)) {
			closeSerialPort();
			return TestResult::FAILED;
		}

		uint8_t rx_byte;
		if (!receiveTestData(&rx_byte, 1, 100)) {
			_current_stats.errors++;
			continue;
		}

		if (rx_byte != test_byte) {
			_current_stats.errors++;
		}

		_current_stats.packets_sent++;
		_current_stats.packets_received++;
	}

	_current_stats.end_time = hrt_absolute_time();
	closeSerialPort();

	float error_rate = (float)_current_stats.errors / (float)num_iterations * 100.0f;
	PX4_INFO("Stress test completed. Error rate: %.2f%% (%u/%d)",
	         (double)error_rate, (unsigned int)_current_stats.errors, num_iterations);

	return (error_rate < 5.0f) ? TestResult::PASSED : TestResult::FAILED;
}

TestSerial::TestResult TestSerial::runBufferTest()
{
	PX4_INFO("Running buffer test...");

	_serial_fd = openSerialPort(_port, _baudrate);
	if (_serial_fd < 0) {
		return TestResult::FAILED;
	}

	resetStats();
	_current_stats.start_time = hrt_absolute_time();

	// Test with different buffer sizes
	const size_t buffer_sizes[] = {1, 8, 64, 256, 1024};
	const size_t num_sizes = sizeof(buffer_sizes) / sizeof(buffer_sizes[0]);

	for (size_t i = 0; i < num_sizes; i++) {
		size_t buffer_size = buffer_sizes[i];
		PX4_INFO("Testing buffer size: %zu bytes", buffer_size);

		uint8_t *test_buffer = new uint8_t[buffer_size];
		uint8_t *rx_buffer = new uint8_t[buffer_size];

		// Fill test buffer with pattern
		for (size_t j = 0; j < buffer_size; j++) {
			test_buffer[j] = j & 0xFF;
		}

		bool test_passed = false;
		if (sendTestData(test_buffer, buffer_size)) {
			if (receiveTestData(rx_buffer, buffer_size, 2000)) {
				if (memcmp(test_buffer, rx_buffer, buffer_size) == 0) {
					test_passed = true;
				}
			}
		}

		delete[] test_buffer;
		delete[] rx_buffer;

		if (!test_passed) {
			PX4_ERR("Buffer test failed for size: %zu", buffer_size);
			closeSerialPort();
			return TestResult::FAILED;
		}

		PX4_INFO("Buffer size %zu: PASSED", buffer_size);
	}

	_current_stats.end_time = hrt_absolute_time();
	closeSerialPort();

	PX4_INFO("Buffer test: PASSED");
	return TestResult::PASSED;
}

void TestSerial::runAllTests()
{
	PX4_INFO("Running all serial tests...");

	_test_results[static_cast<int>(TestType::BASIC_COMM)] = runBasicCommunicationTest();
	_test_results[static_cast<int>(TestType::LOOPBACK)] = runLoopbackTest();
	_test_results[static_cast<int>(TestType::DATA_INTEGRITY)] = runDataIntegrityTest();
	_test_results[static_cast<int>(TestType::BAUD_RATE)] = runBaudRateTest();
	_test_results[static_cast<int>(TestType::STRESS_TEST)] = runStressTest();
	_test_results[static_cast<int>(TestType::BUFFER_TEST)] = runBufferTest();

	printTestResults();
}

bool TestSerial::sendTestData(const uint8_t *data, size_t length)
{
	if (_serial_fd < 0 || !data || length == 0) {
		return false;
	}

	ssize_t bytes_written = write(_serial_fd, data, length);
	if (bytes_written != static_cast<ssize_t>(length)) {
		PX4_ERR("Failed to send test data: %s", strerror(errno));
		return false;
	}

	_current_stats.bytes_sent += bytes_written;
	return true;
}

bool TestSerial::receiveTestData(uint8_t *buffer, size_t expected_length, uint32_t timeout_ms)
{
	if (_serial_fd < 0 || !buffer || expected_length == 0) {
		return false;
	}

	hrt_abstime start_time = hrt_absolute_time();
	size_t bytes_received = 0;

	while (bytes_received < expected_length) {
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			PX4_ERR("Receive timeout after %lu ms", timeout_ms);
			return false;
		}

		ssize_t bytes_read = read(_serial_fd, buffer + bytes_received, expected_length - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
			_current_stats.bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Failed to receive test data: %s", strerror(errno));
			return false;
		}

		// Small delay to prevent CPU spinning
		usleep(1000);
	}

	return true;
}

uint32_t TestSerial::calculateCRC(const uint8_t *data, size_t length)
{
	// Simple CRC32 implementation
	uint32_t crc = 0xFFFFFFFF;

	for (size_t i = 0; i < length; i++) {
		crc ^= data[i];
		for (int j = 0; j < 8; j++) {
			if (crc & 1) {
				crc = (crc >> 1) ^ 0xEDB88320;
			} else {
				crc >>= 1;
			}
		}
	}

	return ~crc;
}

void TestSerial::printTestResults()
{
	PX4_INFO("=== Serial Test Results ===");

	const char *test_names[] = {
		"Basic Communication",
		"Loopback",
		"Data Integrity",
		"Baud Rate",
		"Stress Test",
		"Buffer Test"
	};

	for (int i = 0; i < static_cast<int>(TestType::ALL_TESTS); i++) {
		const char *result_str;
		switch (_test_results[i]) {
		case TestResult::PASSED:  result_str = "PASSED"; break;
		case TestResult::FAILED:  result_str = "FAILED"; break;
		case TestResult::RUNNING: result_str = "RUNNING"; break;
		default:                  result_str = "NOT RUN"; break;
		}

		PX4_INFO("%s: %s", test_names[i], result_str);
	}

	PX4_INFO("=========================");
}

void TestSerial::resetStats()
{
	_current_stats = {};
}

void TestSerial::stop()
{
	_should_exit = true;
	closeSerialPort();
}

int TestSerial::task_spawn(int argc, char *argv[])
{
	const char *port = "/dev/ttyS0";
	int ch;
	const char *myoptarg = nullptr;
	int myoptind = 1;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		if (ch == 'd') port = myoptarg;
	}

	TestSerial *instance = new TestSerial(port);
	if (!instance) return PX4_ERROR;

	_object.store(instance);
	_task_id = task_id_is_work_queue;

	if (instance->init() != PX4_OK) {
		delete instance;
		_object.store(nullptr);
		return PX4_ERROR;
	}
	return PX4_OK;
}

int TestSerial::custom_command(int argc, char *argv[])
{
	if (argc > 0 && strcmp(argv[0], "stop") == 0) {
		// Handle stop command even if not running
		return PX4_OK;
	}

	if (!is_running()) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	TestSerial *instance = get_instance();
	if (!instance) {
		return PX4_ERROR;
	}

	if (argc > 0 && strcmp(argv[0], "status") == 0) {
		PX4_INFO("Port: %s", instance->_port);
		instance->printTestResults();
		return PX4_OK;
	}

	if (argc > 0 && strcmp(argv[0], "test") == 0) {
		if (argc == 1) {
			// No specific test specified, run all tests
			PX4_INFO("Manually executing all tests...");
			instance->runAllTests();
		} else if (argc == 2) {
			// Specific test requested
			const char *test_name = argv[1];
			TestResult result = TestResult::NOT_RUN;

			if (strcmp(test_name, "basic") == 0) {
				PX4_INFO("Running basic communication test...");
				result = instance->runBasicCommunicationTest();
				instance->_test_results[static_cast<int>(TestType::BASIC_COMM)] = result;
			} else if (strcmp(test_name, "loopback") == 0) {
				PX4_INFO("Running loopback test...");
				result = instance->runLoopbackTest();
				instance->_test_results[static_cast<int>(TestType::LOOPBACK)] = result;
			} else if (strcmp(test_name, "integrity") == 0) {
				PX4_INFO("Running data integrity test...");
				result = instance->runDataIntegrityTest();
				instance->_test_results[static_cast<int>(TestType::DATA_INTEGRITY)] = result;
			} else if (strcmp(test_name, "baudrate") == 0) {
				PX4_INFO("Running baud rate test...");
				result = instance->runBaudRateTest();
				instance->_test_results[static_cast<int>(TestType::BAUD_RATE)] = result;
			} else if (strcmp(test_name, "stress") == 0) {
				PX4_INFO("Running stress test...");
				result = instance->runStressTest();
				instance->_test_results[static_cast<int>(TestType::STRESS_TEST)] = result;
			} else if (strcmp(test_name, "buffer") == 0) {
				PX4_INFO("Running buffer test...");
				result = instance->runBufferTest();
				instance->_test_results[static_cast<int>(TestType::BUFFER_TEST)] = result;
			} else if (strcmp(test_name, "all") == 0) {
				PX4_INFO("Running all tests...");
				instance->runAllTests();
				return PX4_OK;
			} else {
				PX4_ERR("Unknown test: %s", test_name);
				PX4_INFO("Available tests: basic, loopback, integrity, baudrate, stress, buffer, all");
				return PX4_ERROR;
			}

			// Print result for individual test
			const char *result_str = (result == TestResult::PASSED) ? "PASSED" :
			                        (result == TestResult::FAILED) ? "FAILED" : "NOT RUN";
			PX4_INFO("Test result: %s", result_str);
		} else {
			return print_usage("too many arguments for test command");
		}
		return PX4_OK;
	}

	return print_usage("unknown command");
}

int TestSerial::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PX4_INFO("Usage: test_serial start -d <device>");
	PX4_INFO("       test_serial stop");
	PX4_INFO("       test_serial status");
	PX4_INFO("       test_serial test [test_name]");
	PX4_INFO("");
	PX4_INFO("  -d <device>    Serial device path (default: /dev/ttyS0)");
	PX4_INFO("");
	PX4_INFO("Commands:");
	PX4_INFO("  status         Show test results and driver status");
	PX4_INFO("  test           Execute all tests");
	PX4_INFO("  test <n>    Execute specific test");
	PX4_INFO("");
	PX4_INFO("Available tests:");
	PX4_INFO("  basic          Basic communication test");
	PX4_INFO("  loopback       Loopback test with patterns");
	PX4_INFO("  integrity      Data integrity test with CRC");
	PX4_INFO("  baudrate       Baud rate compatibility test");
	PX4_INFO("  stress         Stress test with multiple iterations");
	PX4_INFO("  buffer         Buffer size test");
	PX4_INFO("  all            Run all tests (default)");

	return 0;
}
