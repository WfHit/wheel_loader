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
	static constexpr uint8_t NLINK_FRAME_END = 0x77;
	static constexpr uint8_t NLINK_NODE_FRAME3 = 0x06;
	static constexpr int MAX_ANCHORS = 50;
	static constexpr int MAX_RANGES_PER_FRAME = 20;

	// Anchor configuration
	struct AnchorConfig {
		uint8_t id;
		float x, y, z;
		bool active;
	};

	// Range measurement from Node_Frame3
	struct RangeData {
		uint8_t anchor_id;
		uint32_t distance_mm;
		int8_t rssi;
		uint16_t los_confidence;
	} __attribute__((packed));

	// Parser state
	enum class ParserState {
		WAIT_HEADER,
		WAIT_LENGTH,
		WAIT_TYPE,
		WAIT_DATA,
		WAIT_END
	};

	void Run() override;

	// Core functions
	bool parse_frame(const uint8_t *data, size_t length);
	void process_ranges(uint8_t tag_id, uint8_t num_ranges, const RangeData *ranges);
	void publish_range(uint8_t anchor_id, float distance, float accuracy);
	bool configure_device();
	bool load_anchors(const char *filename);
	uint8_t calculate_checksum(const uint8_t *data, size_t length);

	// Serial port (similar to UWB SR150)
	char _port[32];
	int _fd{-1};
	fd_set _uart_set;
	struct timeval _uart_timeout{};

	// Anchors
	AnchorConfig _anchors[MAX_ANCHORS];
	uint8_t _num_anchors{0};

	// Parser
	ParserState _parser_state{ParserState::WAIT_HEADER};
	uint8_t _rx_buffer[512];
	size_t _rx_buffer_pos{0};
	uint16_t _frame_length{0};

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
