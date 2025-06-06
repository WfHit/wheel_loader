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
 * @file module_constants.h
 * @brief Module identification and status constants for articulated chassis modules
 *
 * This header defines common constants used across articulated chassis modules
 * for status reporting and inter-module communication.
 *
 * @author PX4 Development Team
 */

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Module ID Constants
 *
 * These constants identify different modules in the articulated chassis system.
 * Used in ModuleStatus.module_id field.
 */

// Chassis Control Modules (0-19)
#define MODULE_ID_STEERING              0   ///< Steering controller module
#define MODULE_ID_WHEEL_CONTROLLER      1   ///< Individual wheel speed controller
#define MODULE_ID_SAFETY_MANAGER        2   ///< Safety monitoring and coordination
#define MODULE_ID_SLIP_ESTIMATOR        3   ///< Wheel slip detection and estimation
#define MODULE_ID_TRACTION_CONTROL      4   ///< Predictive traction control
#define MODULE_ID_TERRAIN_ADAPTATION    5   ///< Terrain adaptation module
#define MODULE_ID_LOAD_AWARE_TORQUE     6   ///< Load-aware torque distribution

// Vehicle Level Modules (20-39)
#define MODULE_ID_WHEEL_LOADER          20  ///< Main wheel loader controller
#define MODULE_ID_BOOM_CONTROL          21  ///< Boom hydraulic control
#define MODULE_ID_BUCKET_CONTROL        22  ///< Bucket hydraulic control

// Communication and Proxy Modules (40-59)
#define MODULE_ID_UORB_PROXY            40  ///< uORB UART proxy module
#define MODULE_ID_UORB_BRIDGE           41  ///< uORB UART bridge module

// Reserved for future expansion (60-99)
// User-defined modules (100-255)

/**
 * Module Health Status Constants
 *
 * These constants define the health status of modules.
 * Compatible with UAVCAN NodeStatus health values.
 * Used in ModuleStatus.health_status field.
 */
#define MODULE_HEALTH_OK                0   ///< Module is functioning normally
#define MODULE_HEALTH_WARNING           1   ///< Module has minor issues but is operational
#define MODULE_HEALTH_ERROR             2   ///< Module has significant issues, degraded operation
#define MODULE_HEALTH_CRITICAL          3   ///< Module has critical issues, may stop functioning

/**
 * Module Operational Status Constants
 *
 * These constants define the operational mode of modules.
 * Used in ModuleStatus.operational_status field.
 */
#define MODULE_OP_NORMAL                0   ///< Normal operation mode
#define MODULE_OP_INITIALIZATION        1   ///< Module is initializing
#define MODULE_OP_MAINTENANCE           2   ///< Module is in maintenance mode
#define MODULE_OP_EMERGENCY_STOP        3   ///< Emergency stop is active
#define MODULE_OP_DEGRADED              4   ///< Operating in degraded mode
#define MODULE_OP_OFFLINE               7   ///< Module is offline/not responding

/**
 * Module Priority Levels
 *
 * Priority levels for module operations and error handling.
 */
#define MODULE_PRIORITY_CRITICAL        0   ///< Critical modules (safety, steering)
#define MODULE_PRIORITY_HIGH            1   ///< High priority modules (traction, wheel control)
#define MODULE_PRIORITY_NORMAL          2   ///< Normal priority modules (comfort features)
#define MODULE_PRIORITY_LOW             3   ///< Low priority modules (diagnostics, logging)

/**
 * Module Communication Timeouts (microseconds)
 */
#define MODULE_WATCHDOG_TIMEOUT_US      1000000     ///< 1 second watchdog timeout
#define MODULE_STATUS_UPDATE_RATE_US    100000      ///< 100ms status update rate
#define MODULE_HEARTBEAT_RATE_US        500000      ///< 500ms heartbeat rate

/**
 * Module Data Array Sizes
 *
 * Standard sizes for module status data arrays.
 */
#define MODULE_STATUS_DATA_SIZE         8   ///< Number of float32 data fields in ModuleStatus
#define MODULE_NAME_MAX_LENGTH          16  ///< Maximum length of module name string

#ifdef __cplusplus
}
#endif
