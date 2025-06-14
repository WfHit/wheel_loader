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

// System includes
#include <cstring>

// PX4 platform includes
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

// Library includes
#include <lib/perf/perf_counter.h>

// uORB includes
#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_uwb.h>

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
	// LinkTrack protocol definitions
	static constexpr uint8_t HEADER = 0x55;
	static constexpr uint8_t FRAME_END = 0x77;
	static constexpr int MAX_ANCHORS = 16;
	static constexpr int MAX_MEASUREMENTS_PER_MSG = 8;

	enum class FrameType : uint8_t {
		POSITION_2D = 0x00,
		POSITION_3D = 0x01,
		RANGE_BATCH = 0x02,
		ANCHOR_POSITION = 0x03,
		TAG_POSITION = 0x04,
		SYSTEM_STATUS = 0x05,
		MULTI_RANGE_WITH_DIAGNOSTICS = 0x06
	};

	struct AnchorInfo {
		uint8_t id;
		float x;
		float y;
		float z;
		bool valid;
		char name[32];
	};

	struct RangeData {
		uint8_t anchor_id;
		uint8_t tag_id;
		uint32_t distance_mm;
		int8_t rssi;
		uint8_t los_confidence;
		uint16_t first_path_amp;
		uint16_t rx_power;
		uint8_t multipath_count;
	} __attribute__((packed));

	struct MultiRangePacket {
		uint8_t tag_id;
		uint8_t num_measurements;
		uint64_t timestamp_us;
		RangeData measurements[MAX_MEASUREMENTS_PER_MSG];
	} __attribute__((packed));

	void Run() override;
	bool parse_frame(uint8_t *buffer, size_t len);
	void process_multi_range(const uint8_t *data);
	void publish_uwb_batch(const MultiRangePacket &packet);
	bool configure_device();
	uint8_t calculate_checksum(const uint8_t *data, size_t len);
	bool load_anchor_positions(const char *filename);
	void save_anchor_positions_to_params();
	float estimate_range_bias(int8_t rssi, uint8_t los_confidence, uint8_t multipath_count);
	AnchorInfo *find_anchor(uint8_t id);

	// Serial port
	char _port[32];
	int _fd{-1};

	// Anchor configuration
	AnchorInfo _anchors[MAX_ANCHORS];
	uint8_t _num_anchors{0};
	char _anchor_file[256];

	// Parser state
	enum class ParserState {
		WAIT_HEADER,
		WAIT_LENGTH_LOW,
		WAIT_LENGTH_HIGH,
		WAIT_TYPE,
		WAIT_DATA,
		WAIT_CHECKSUM,
		WAIT_END
	};

	ParserState _parser_state{ParserState::WAIT_HEADER};
	uint8_t _rx_buffer[1024];
	size_t _rx_buffer_pos{0};
	uint16_t _frame_length{0};
	uint8_t _frame_type{0};

	// Publications
	uORB::Publication<sensor_uwb_s> _sensor_uwb_pub{ORB_ID(sensor_uwb)};

	// Performance counters
	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;
	perf_counter_t _buffer_overflows;
	perf_counter_t _range_batch_perf;

	// Statistics
	uint32_t _total_measurements{0};
	uint32_t _filtered_measurements{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::UWB_BAUD>) _param_baud_rate,
		(ParamInt<px4::params::UWB_TAG_ID>) _param_tag_id,
		(ParamFloat<px4::params::UWB_MIN_RSSI>) _param_min_rssi,
		(ParamInt<px4::params::UWB_PUB_ALL>) _param_publish_all_ranges,
		(ParamFloat<px4::params::UWB_X_OFF>) _param_offset_x,
		(ParamFloat<px4::params::UWB_Y_OFF>) _param_offset_y,
		(ParamFloat<px4::params::UWB_Z_OFF>) _param_offset_z,
		(ParamInt<px4::params::UWB_EN>) _param_enable,
		(ParamInt<px4::params::UWB_PORT>) _param_port
	)
};
