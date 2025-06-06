#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>

#include <lib/distributed_uorb/uart_protocol/uart_protocol.hpp>

#include <termios.h>
#include <fcntl.h>
#include <poll.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

namespace distributed_uorb {

class UartTransport
{
public:
	UartTransport();
	~UartTransport();

	/**
	 * Initialize UART transport
	 * @param device_path UART device path (e.g., "/dev/ttyS3")
	 * @param baudrate Baudrate (e.g., B921600)
	 * @return 0 on success, -1 on error
	 */
	int init(const char *device_path, speed_t baudrate);

	/**
	 * Send a frame over UART
	 * @param frame Frame to send
	 * @return bytes sent, -1 on error
	 */
	int sendFrame(const UartFrame &frame);

	/**
	 * Receive a frame from UART (non-blocking)
	 * @param frame Buffer to store received frame
	 * @param timeout_ms Timeout in milliseconds
	 * @return 1 if frame received, 0 if no data, -1 on error
	 */
	int receiveFrame(UartFrame &frame, int timeout_ms = 10);

	/**
	 * Check if transport is ready
	 */
	bool isReady() const { return _fd >= 0; }

	/**
	 * Get file descriptor for polling
	 */
	int getFd() const { return _fd; }

	/**
	 * Close transport
	 */
	void close();

private:
	int _fd{-1};
	uint8_t _rx_buffer[UART_FRAME_MAX_SIZE];
	size_t _rx_buffer_pos{0};

	/**
	 * Parse received bytes for complete frames
	 * @param frame Output frame if found
	 * @return 1 if complete frame found, 0 if incomplete, -1 on error
	 */
	int parseRxBuffer(UartFrame &frame);

	/**
	 * Find sync pattern in buffer
	 * @return position of sync pattern, -1 if not found
	 */
	int findSyncPattern();

	/**
	 * Validate frame checksum
	 * @param frame Frame to validate
	 * @return true if valid
	 */
	bool validateFrame(const UartFrame &frame);
};

} // namespace distributed_uorb
