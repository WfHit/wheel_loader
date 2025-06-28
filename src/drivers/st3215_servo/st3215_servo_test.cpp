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

/**
 * @file st3215_servo_test.cpp
 * @author PX4 Development Team
 *
 * Test utilities for ST3215 smart servo driver
 */

#include "st3215_servo.hpp"

#include <px4_platform_common/log.h>
#include <cstring>
#include <cerrno>
#include <termios.h>

bool ST3215Servo::uart_test(const char *port1, const char *port2, int baud_rate)
{
	PX4_INFO("=== UART Test ===");
	PX4_INFO("Port 1: %s", port1);
	PX4_INFO("Port 2: %s", port2);
	PX4_INFO("Baud rate: %d", baud_rate);

	int fd1 = open_uart_port(port1, baud_rate);
	if (fd1 < 0) {
		PX4_ERR("Failed to open port 1: %s", port1);
		return false;
	}

	int fd2 = open_uart_port(port2, baud_rate);
	if (fd2 < 0) {
		PX4_ERR("Failed to open port 2: %s", port2);
		close_uart_port(fd1);
		return false;
	}

	PX4_INFO("Both ports opened successfully");

	// Run communication tests
	bool test_passed = true;

	test_passed &= test_uart_loopback(fd1, fd2);
	test_passed &= test_uart_bidirectional(fd1, fd2);
	test_passed &= test_uart_performance(fd1, fd2);

	// Cleanup
	close_uart_port(fd1);
	close_uart_port(fd2);

	if (test_passed) {
		PX4_INFO("=== UART Test PASSED ===");
		return true;
	} else {
		PX4_ERR("=== UART Test FAILED ===");
		return false;
	}
}

int ST3215Servo::open_uart_port(const char *port, int baud_rate)
{
	PX4_INFO("Opening %s at %d baud", port, baud_rate);

	int fd = ::open(port, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (fd < 0) {
		PX4_ERR("Failed to open %s: %s", port, strerror(errno));
		return -1;
	}

	// Configure port
	struct termios tty;
	if (tcgetattr(fd, &tty) != 0) {
		PX4_ERR("Failed to get attributes for %s: %s", port, strerror(errno));
		::close(fd);
		return -1;
	}

	// Set baud rate
	speed_t speed;
	switch (baud_rate) {
	case 9600:   speed = B9600; break;
	case 19200:  speed = B19200; break;
	case 38400:  speed = B38400; break;
	case 57600:  speed = B57600; break;
	case 115200: speed = B115200; break;
	case 1000000: speed = B1000000; break;
	default:
		PX4_ERR("Unsupported baud rate: %d", baud_rate);
		::close(fd);
		return -1;
	}

	cfsetospeed(&tty, speed);
	cfsetispeed(&tty, speed);

	// 8N1
	tty.c_cflag &= ~PARENB;
	tty.c_cflag &= ~CSTOPB;
	tty.c_cflag &= ~CSIZE;
	tty.c_cflag |= CS8;

	// No flow control
	tty.c_cflag &= ~CRTSCTS;
	tty.c_cflag |= CREAD | CLOCAL;

	// Raw mode
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK);
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);
	tty.c_oflag &= ~OPOST;

	// Timeouts
	tty.c_cc[VMIN] = 0;
	tty.c_cc[VTIME] = 1; // 100ms timeout

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		PX4_ERR("Failed to set attributes for %s: %s", port, strerror(errno));
		::close(fd);
		return -1;
	}

	// Flush buffers
	tcflush(fd, TCIOFLUSH);

	PX4_INFO("Successfully opened %s (fd: %d)", port, fd);
	return fd;
}

void ST3215Servo::close_uart_port(int fd)
{
	if (fd >= 0) {
		::close(fd);
	}
}

bool ST3215Servo::test_uart_loopback(int fd1, int fd2)
{
	PX4_INFO("--- Testing UART Loopback ---");

	const char test_data[] = "Hello UART Test!";
	const int test_len = strlen(test_data);
	char rx_buffer[64];

	// Send from port 1 to port 2
	PX4_INFO("Sending from port 1 to port 2: '%s'", test_data);

	ssize_t bytes_sent = ::write(fd1, test_data, test_len);
	if (bytes_sent != test_len) {
		PX4_ERR("Send failed: expected %d, sent %d", test_len, (int)bytes_sent);
		return false;
	}

	// Force transmission
	tcdrain(fd1);
	px4_usleep(10000); // 10ms delay

	// Read from port 2
	memset(rx_buffer, 0, sizeof(rx_buffer));
	ssize_t bytes_received = ::read(fd2, rx_buffer, sizeof(rx_buffer) - 1);

	if (bytes_received <= 0) {
		PX4_ERR("Receive failed: %d bytes received", (int)bytes_received);
		return false;
	}

	rx_buffer[bytes_received] = '\0';
	PX4_INFO("Received on port 2: '%s' (%d bytes)", rx_buffer, (int)bytes_received);

	if (bytes_received != test_len || strcmp(test_data, rx_buffer) != 0) {
		PX4_ERR("Data mismatch!");
		return false;
	}

	PX4_INFO("Loopback test PASSED");
	return true;
}

bool ST3215Servo::test_uart_bidirectional(int fd1, int fd2)
{
	PX4_INFO("--- Testing Bidirectional Communication ---");

	const char data1[] = "Port1->Port2";
	const char data2[] = "Port2->Port1";
	char rx_buffer[64];

	// Test 1: Port 1 to Port 2
	PX4_INFO("Testing Port1 -> Port2");
	::write(fd1, data1, strlen(data1));
	tcdrain(fd1);
	px4_usleep(5000);

	memset(rx_buffer, 0, sizeof(rx_buffer));
	ssize_t bytes_rx = ::read(fd2, rx_buffer, sizeof(rx_buffer) - 1);
	if (bytes_rx <= 0 || strcmp(data1, rx_buffer) != 0) {
		PX4_ERR("Port1->Port2 failed");
		return false;
	}
	PX4_INFO("Port1->Port2: OK");

	// Test 2: Port 2 to Port 1
	PX4_INFO("Testing Port2 -> Port1");
	::write(fd2, data2, strlen(data2));
	tcdrain(fd2);
	px4_usleep(5000);

	memset(rx_buffer, 0, sizeof(rx_buffer));
	bytes_rx = ::read(fd1, rx_buffer, sizeof(rx_buffer) - 1);
	if (bytes_rx <= 0 || strcmp(data2, rx_buffer) != 0) {
		PX4_ERR("Port2->Port1 failed");
		return false;
	}
	PX4_INFO("Port2->Port1: OK");

	PX4_INFO("Bidirectional test PASSED");
	return true;
}

bool ST3215Servo::test_uart_performance(int fd1, int fd2)
{
	PX4_INFO("--- Testing UART Performance ---");

	const int test_size = 1024;
	char *test_data = new char[test_size];
	char *rx_buffer = new char[test_size];

	// Generate test data
	for (int i = 0; i < test_size; i++) {
		test_data[i] = (char)(i % 256);
	}

	// Performance test: send large data block
	PX4_INFO("Sending %d bytes for performance test", test_size);

	ssize_t bytes_sent = ::write(fd1, test_data, test_size);
	if (bytes_sent != test_size) {
		PX4_ERR("Performance test send failed: %d bytes sent", (int)bytes_sent);
		delete[] test_data;
		delete[] rx_buffer;
		return false;
	}

	tcdrain(fd1);
	px4_usleep(50000); // 50ms delay for large data

	// Receive data
	memset(rx_buffer, 0, test_size);
	ssize_t bytes_received = ::read(fd2, rx_buffer, test_size);

	if (bytes_received != test_size) {
		PX4_ERR("Performance test receive failed: %d bytes received", (int)bytes_received);
		delete[] test_data;
		delete[] rx_buffer;
		return false;
	}

	// Verify data integrity
	bool data_ok = true;
	for (int i = 0; i < test_size; i++) {
		if (test_data[i] != rx_buffer[i]) {
			PX4_ERR("Data corruption at position %d: expected 0x%02X, got 0x%02X",
				i, (unsigned char)test_data[i], (unsigned char)rx_buffer[i]);
			data_ok = false;
			break;
		}
	}

	delete[] test_data;
	delete[] rx_buffer;

	if (data_ok) {
		PX4_INFO("Performance test PASSED");
		return true;
	} else {
		PX4_ERR("Performance test FAILED - data corruption detected");
		return false;
	}
}
