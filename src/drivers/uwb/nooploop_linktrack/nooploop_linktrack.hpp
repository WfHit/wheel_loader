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
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_uwb.h>
#include <sys/select.h>

using namespace time_literals;

class NoopLoopLinkTrack : public ModuleBase<NoopLoopLinkTrack>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	NoopLoopLinkTrack(const char *port);
	~NoopLoopLinkTrack() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int init();

private:
	// Protocol constants
	static constexpr uint8_t NLINK_HEADER = 0x55;
	static constexpr uint8_t NLINK_NODE_FRAME3 = 0x05;
	static constexpr uint8_t NLINK_ROLE_TAG = 0x02;
	static constexpr uint8_t NLINK_ROLE_ANCHOR = 0x01;
	static constexpr int MAX_ANCHORS = 50;
	static constexpr int MAX_RANGES_PER_FRAME = 16; // Reserve space for 16 anchors as requested
	static constexpr size_t RX_BUFFER_SIZE = 2048;  // Larger buffer for big frames (16 anchors * ~8 bytes each + header + overhead)

	// Anchor configuration
	struct AnchorConfig {
		uint8_t id;
		float x, y, z;
		bool forbid; // true=forbidden/inactive, false=active
	};

	// Range measurement from Node_Frame3 - matches protocol specification exactly
	struct NodeFrame3Header {
		uint8_t frame_header;    // 0x55 (Index 0)
		uint8_t function_mark;   // 0x05 for Node_Frame3 (Index 1)
		uint16_t frame_length;   // Frame length in bytes (Index 2-3, little endian)
		uint8_t role;           // Local node role: 0x01=ANCHOR, 0x02=TAG (Index 4)
		uint8_t id;             // Local node ID (Index 5)
	} __attribute__((packed));

	// Payload data that follows the header
	struct NodeFrame3Payload {
		uint32_t local_time;    // Time of local node, unit: ms (Index 6-9)
		uint32_t system_time;   // Time of system, unit: ms (Index 10-13)
		uint8_t reserved[4];    // Reserved (Index 14-17)
		uint16_t voltage;       // Interface supply voltage of the local node, unit: V * 1000 (Index 18-19)
		uint8_t valid_quantity; // Total valid nodes (Index 20)
	} __attribute__((packed));

	struct AnchorData {
		uint8_t role;           // Role corresponding to this block: 0x01=ANCHOR (Block role)
		uint8_t id;             // ID corresponding to this block (Block ID)
		uint8_t distance[3];    // Distance from tag to corresponding anchor, unit: m * 1000 (3 bytes, little endian)
		uint8_t fp_rssi;        // First path power level, unit: dB (fp_rssi * (-2))
		uint8_t rx_rssi;        // Received power level, unit: dB (rx_rssi * (-2))
	} __attribute__((packed));

	// Complete frame structure (header + payload + data blocks + checksum)
	struct CompleteFrame {
		NodeFrame3Header header;
		NodeFrame3Payload payload;
		// Variable number of AnchorData blocks follows
		// uint8_t checksum;  // At the end (sum of all previous bytes)
	} __attribute__((packed));

	// Parser state machine (similar to MAVLink)
	enum class ParserState {
		UNINIT = 0,
		IDLE,
		GOT_HEADER,
		GOT_FUNCTION,
		GOT_LENGTH_LOW,
		GOT_PAYLOAD,
		GOT_CHECKSUM
	};

	void Run() override;

	// Core functions
	bool parse_char(uint8_t c);
	bool parse_frame(const uint8_t *data, size_t length);
	void process_ranges(uint8_t tag_id, uint8_t num_ranges, const AnchorData *ranges);
	void publish_range(uint8_t anchor_id, float distance, float rssi, float fp_rssi);
	bool load_anchors(const char *filename);

	// Serial port (similar to UWB SR150)
	char _port[32];
	int _fd{-1};
	fd_set _uart_set;
	struct timeval _uart_timeout{};

	// Anchors
	AnchorConfig _anchors[MAX_ANCHORS];
	uint8_t _num_anchors{0};

	// Parser
	ParserState _parser_state{ParserState::IDLE};
	uint8_t _rx_buffer[RX_BUFFER_SIZE];
	size_t _rx_buffer_pos{0};
	uint16_t _frame_length{0};
	size_t _packet_idx{0};
	uint8_t _frame_buffer[RX_BUFFER_SIZE]; // Static buffer for complete frames
	uint8_t _calculated_checksum{0};

	// Frame parsing statistics
	uint32_t _parse_errors{0};
	uint32_t _buffer_overruns{0};
	uint32_t _frames_received{0};

	// Publications
	uORB::Publication<sensor_uwb_s> _sensor_uwb_pub{ORB_ID(sensor_uwb)};

	// Performance counters
	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UWB_BAUD>) _param_baud_rate,
		(ParamInt<px4::params::UWB_TAG_ID>) _param_tag_id,
		(ParamInt<px4::params::UWB_EN>) _param_enable,
		(ParamInt<px4::params::UWB_UPDATE_RATE>) _param_update_rate
	)
};
