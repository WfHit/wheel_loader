#include "uart_protocol.hpp"

namespace distributed_uorb {

uint16_t calculateCRC16(const uint8_t *data, size_t len)
{
	uint16_t crc = 0xFFFF;
	const uint16_t polynomial = 0x1021;

	for (size_t i = 0; i < len; i++) {
		crc ^= (uint16_t)data[i] << 8;

		for (int j = 0; j < 8; j++) {
			if (crc & 0x8000) {
				crc = (crc << 1) ^ polynomial;

			} else {
				crc <<= 1;
			}
		}
	}

	return crc;
}

} // namespace distributed_uorb
