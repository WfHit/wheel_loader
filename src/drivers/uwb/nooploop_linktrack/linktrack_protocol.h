/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include <stdint.h>

/* LinkTrack Protocol Constants */
#define LINKTRACK_START_CHAR_UWB	0x55
#define LINKTRACK_HEADER_SIZE		6
#define LINKTRACK_MIN_MSG_SIZE		7  // Header + 1 byte checksum

/* Message IDs */
#define LINKTRACK_MSG_BATCH		0x30
#define LINKTRACK_MSG_SINGLE		0x31

/* Maximum number of anchors in a batch message */
#define LINKTRACK_MAX_ANCHORS		8

#pragma pack(push, 1)

/* Anchor data in batch message */
typedef struct {
	uint8_t id;
	int32_t x;		// mm
	int32_t y;		// mm
	int32_t z;		// mm
	uint32_t range;		// mm
	uint8_t rssi;		// Signal strength
} linktrack_anchor_t;

/* Batch measurement message */
typedef struct {
	uint32_t seq_num;
	uint8_t tag_id;
	uint8_t num_anchors;
	linktrack_anchor_t anchors[LINKTRACK_MAX_ANCHORS];
} linktrack_batch_t;

/* Single measurement message */
typedef struct {
	uint8_t tag_id;
	uint8_t anchor_id;
	uint32_t range;		// mm
	int32_t x;		// mm
	int32_t y;		// mm
	int32_t z;		// mm
	int16_t vx;		// mm/s
	int16_t vy;		// mm/s
	int16_t vz;		// mm/s
	uint8_t rssi;		// Signal strength
} linktrack_single_t;

#pragma pack(pop)
