
#include <lib/distributed_uorb/uart_transport/uart_transport.hpp>

#include <errno.h>
#include <string.h>

namespace distributed_uorb {

UartTransport::UartTransport()
{
}

UartTransport::~UartTransport()
{
	close();
}

int UartTransport::init(const char *device_path, speed_t baudrate)
{
	if (!device_path) {
		PX4_ERR("Invalid device path");
		return -1;
	}

	// Open UART device
	_fd = px4_open(device_path, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_fd < 0) {
		PX4_ERR("Failed to open UART device %s: %s", device_path, strerror(errno));
		return -1;
	}

	// Configure UART
	struct termios uart_config;

	if (tcgetattr(_fd, &uart_config) < 0) {
		PX4_ERR("Failed to get UART config: %s", strerror(errno));
		close();
		return -1;
	}

	// Clear ONLCR flag (don't send '\r' after '\n')
	uart_config.c_oflag &= ~ONLCR;

	// No parity, one stop bit
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Set baudrate
	if (cfsetispeed(&uart_config, baudrate) < 0 || cfsetospeed(&uart_config, baudrate) < 0) {
		PX4_ERR("Failed to set UART baudrate");
		close();
		return -1;
	}

	if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
		PX4_ERR("Failed to set UART config: %s", strerror(errno));
		close();
		return -1;
	}

	// Clear buffers
	_rx_buffer_pos = 0;
	memset(_rx_buffer, 0, sizeof(_rx_buffer));

	PX4_INFO("UART transport initialized on %s", device_path);
	return 0;
}

int UartTransport::sendFrame(const UartFrame &frame)
{
	if (_fd < 0) {
		return -1;
	}

	// Calculate total frame size
	size_t frame_size = sizeof(frame.header) + frame.header.length + sizeof(frame.checksum);

	// Send frame
	ssize_t bytes_sent = px4_write(_fd, &frame, frame_size);

	if (bytes_sent != (ssize_t)frame_size) {
		PX4_ERR("Failed to send complete frame: sent %d of %zu bytes", (int)bytes_sent, frame_size);
		return -1;
	}

	return bytes_sent;
}

int UartTransport::receiveFrame(UartFrame &frame, int timeout_ms)
{
	if (_fd < 0) {
		return -1;
	}

	// Poll for data
	struct pollfd fds[1];
	fds[0].fd = _fd;
	fds[0].events = POLLIN;

	int poll_ret = px4_poll(fds, 1, timeout_ms);

	if (poll_ret < 0) {
		return -1;
	}

	if (poll_ret == 0) {
		// Timeout - no data available
		return 0;
	}

	// Read available data
	ssize_t bytes_read = px4_read(_fd, &_rx_buffer[_rx_buffer_pos],
				      sizeof(_rx_buffer) - _rx_buffer_pos);

	if (bytes_read <= 0) {
		return (bytes_read == 0) ? 0 : -1;
	}

	_rx_buffer_pos += bytes_read;

	// Try to parse complete frame
	return parseRxBuffer(frame);
}

int UartTransport::parseRxBuffer(UartFrame &frame)
{
	// Need at least header size
	if (_rx_buffer_pos < sizeof(UartFrameHeader)) {
		return 0;
	}

	// Find sync pattern
	int sync_pos = findSyncPattern();

	if (sync_pos < 0) {
		// No sync pattern found, clear buffer
		_rx_buffer_pos = 0;
		return 0;
	}

	// Move sync pattern to beginning if not already there
	if (sync_pos > 0) {
		memmove(_rx_buffer, &_rx_buffer[sync_pos], _rx_buffer_pos - sync_pos);
		_rx_buffer_pos -= sync_pos;
	}

	// Check if we have enough data for header
	if (_rx_buffer_pos < sizeof(UartFrameHeader)) {
		return 0;
	}

	// Parse header
	UartFrameHeader *header = reinterpret_cast<UartFrameHeader *>(_rx_buffer);

	// Validate header
	if (header->sync1 != UART_SYNC1 || header->sync2 != UART_SYNC2) {
		// Invalid sync, remove first byte and try again
		memmove(_rx_buffer, &_rx_buffer[1], _rx_buffer_pos - 1);
		_rx_buffer_pos--;
		return parseRxBuffer(frame);
	}

	// Check payload length
	if (header->length > UART_MAX_PAYLOAD_SIZE) {
		PX4_WARN("Invalid payload length: %u", header->length);
		// Skip this frame
		memmove(_rx_buffer, &_rx_buffer[1], _rx_buffer_pos - 1);
		_rx_buffer_pos--;
		return parseRxBuffer(frame);
	}

	// Calculate total frame size
	size_t total_frame_size = sizeof(UartFrameHeader) + header->length + sizeof(uint16_t);

	// Check if we have complete frame
	if (_rx_buffer_pos < total_frame_size) {
		return 0; // Need more data
	}

	// Copy frame
	memcpy(&frame, _rx_buffer, total_frame_size);

	// Validate checksum
	if (!validateFrame(frame)) {
		PX4_WARN("Invalid frame checksum");
		// Skip this frame
		memmove(_rx_buffer, &_rx_buffer[1], _rx_buffer_pos - 1);
		_rx_buffer_pos--;
		return parseRxBuffer(frame);
	}

	// Remove frame from buffer
	memmove(_rx_buffer, &_rx_buffer[total_frame_size], _rx_buffer_pos - total_frame_size);
	_rx_buffer_pos -= total_frame_size;

	return 1; // Frame received successfully
}

int UartTransport::findSyncPattern()
{
	for (size_t i = 0; i < _rx_buffer_pos - 1; i++) {
		if (_rx_buffer[i] == UART_SYNC1 && _rx_buffer[i + 1] == UART_SYNC2) {
			return i;
		}
	}

	return -1;
}

bool UartTransport::validateFrame(const UartFrame &frame)
{
	// Calculate expected checksum
	uint16_t calculated_crc = calculateCRC16(reinterpret_cast<const uint8_t *>(&frame.header),
						 sizeof(frame.header) + frame.header.length);

	// Get received checksum
	const uint8_t *frame_bytes = reinterpret_cast<const uint8_t *>(&frame);
	size_t checksum_offset = sizeof(frame.header) + frame.header.length;
	uint16_t received_crc = *reinterpret_cast<const uint16_t *>(&frame_bytes[checksum_offset]);

	return calculated_crc == received_crc;
}

void UartTransport::close()
{
	if (_fd >= 0) {
		px4_close(_fd);
		_fd = -1;
	}

	_rx_buffer_pos = 0;
}

} // namespace distributed_uorb
