############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
#############################################################################

option(EKF2_SYMFORCE_GEN "ekf2 generate symforce output" OFF)

# Symforce code generation TODO:fixme
execute_process(
    COMMAND ${PYTHON_EXECUTABLE} -m symforce.symbolic
    RESULT_VARIABLE PYTHON_SYMFORCE_EXIT_CODE
    OUTPUT_QUIET
    ERROR_QUIET
)

# for now only provide symforce target helper if derivation.py generation isn't default
if((NOT CONFIG_EKF2_MAGNETOMETER) OR (NOT CONFIG_EKF2_WIND))
	set(EKF2_SYMFORCE_GEN ON)
endif()

set(EKF_DERIVATION_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/EKF/python/ekf_derivation)

set(EKF_GENERATED_FILES ${EKF_DERIVATION_SRC_DIR}/generated/state.h)
set(EKF_GENERATED_DERIVATION_INCLUDE_PATH "${EKF_DERIVATION_SRC_DIR}/..")

if(EKF2_SYMFORCE_GEN AND (${PYTHON_SYMFORCE_EXIT_CODE} EQUAL 0))

	# regenerate default in tree
	add_custom_command(
		OUTPUT
			${EKF_DERIVATION_SRC_DIR}/generated/predict_covariance.h
			${EKF_DERIVATION_SRC_DIR}/generated/state.h
		COMMAND
			${PYTHON_EXECUTABLE} ${EKF_DERIVATION_SRC_DIR}/derivation.py
		DEPENDS
			${EKF_DERIVATION_SRC_DIR}/derivation.py
			${EKF_DERIVATION_SRC_DIR}/utils/derivation_utils.py

		WORKING_DIRECTORY ${EKF_DERIVATION_SRC_DIR}
		COMMENT "Symforce code generation (default)"
		USES_TERMINAL
	)

	# generate to build directory
	set(EKF_DERIVATION_DST_DIR ${CMAKE_CURRENT_BINARY_DIR}/ekf_derivation)
	file(MAKE_DIRECTORY ${EKF_DERIVATION_DST_DIR})

	set(EKF_GENERATED_FILES ${EKF_DERIVATION_DST_DIR}/generated/state.h)
	set(EKF_GENERATED_DERIVATION_INCLUDE_PATH ${CMAKE_CURRENT_BINARY_DIR})

	set(SYMFORCE_ARGS)

	if(NOT CONFIG_EKF2_MAGNETOMETER)
		message(STATUS "ekf2: symforce disabling mag")
		list(APPEND SYMFORCE_ARGS "--disable_mag")
	endif()

	if(NOT CONFIG_EKF2_WIND)
		message(STATUS "ekf2: symforce disabling wind")
		list(APPEND SYMFORCE_ARGS "--disable_wind")
	endif()

	add_custom_command(
		OUTPUT
			${EKF_DERIVATION_DST_DIR}/generated/predict_covariance.h
			${EKF_DERIVATION_DST_DIR}/generated/state.h
		COMMAND
			${PYTHON_EXECUTABLE} ${EKF_DERIVATION_SRC_DIR}/derivation.py ${SYMFORCE_ARGS}
		DEPENDS
			${EKF_DERIVATION_SRC_DIR}/derivation.py
			${EKF_DERIVATION_SRC_DIR}/utils/derivation_utils.py

		WORKING_DIRECTORY ${EKF_DERIVATION_DST_DIR}
		COMMENT "Symforce code generation"
		USES_TERMINAL
	)



	add_custom_target(ekf2_symforce_generate
		DEPENDS
			${EKF_DERIVATION_SRC_DIR}/generated/predict_covariance.h
			${EKF_DERIVATION_DST_DIR}/generated/predict_covariance.h
	)
endif()

set(EKF_MODULE_PARAMS)
set(EKF_LIBS)
set(EKF_SRCS)
list(APPEND EKF_SRCS
	EKF/control.cpp
	EKF/covariance.cpp
	EKF/ekf.cpp
	EKF/ekf_helper.cpp
	EKF/estimator_interface.cpp
	EKF/height_control.cpp
	EKF/velocity_fusion.cpp
	EKF/position_fusion.cpp
	EKF/yaw_fusion.cpp

	EKF/imu_down_sampler/imu_down_sampler.cpp

	EKF/aid_sources/fake_height_control.cpp
	EKF/aid_sources/fake_pos_control.cpp
	EKF/aid_sources/ZeroGyroUpdate.cpp
	EKF/aid_sources/ZeroVelocityUpdate.cpp
	EKF/aid_sources/zero_innovation_heading_update.cpp
)

if(CONFIG_EKF2_AIRSPEED)
	list(APPEND EKF_SRCS EKF/aid_sources/airspeed/airspeed_fusion.cpp)
	list(APPEND EKF_MODULE_PARAMS params_airspeed.yaml)
endif()

if(CONFIG_EKF2_AUX_GLOBAL_POSITION)
	list(APPEND EKF_SRCS EKF/aid_sources/aux_global_position/aux_global_position.cpp)
	list(APPEND EKF_MODULE_PARAMS params_aux_global_position.yaml)
endif()

if(CONFIG_EKF2_AUXVEL)
	list(APPEND EKF_SRCS EKF/aid_sources/auxvel/auxvel_fusion.cpp)
	list(APPEND EKF_MODULE_PARAMS params_aux_velocity.yaml)
endif()

if(CONFIG_EKF2_BAROMETER)
	list(APPEND EKF_SRCS EKF/aid_sources/barometer/baro_height_control.cpp)
	list(APPEND EKF_MODULE_PARAMS params_barometer.yaml)
endif()

if(CONFIG_EKF2_DRAG_FUSION)
	list(APPEND EKF_SRCS EKF/aid_sources/drag/drag_fusion.cpp)
	list(APPEND EKF_MODULE_PARAMS params_drag.yaml)
endif()

if(CONFIG_EKF2_EXTERNAL_VISION)
	list(APPEND EKF_SRCS
		EKF/aid_sources/external_vision/ev_control.cpp
		EKF/aid_sources/external_vision/ev_height_control.cpp
		EKF/aid_sources/external_vision/ev_pos_control.cpp
		EKF/aid_sources/external_vision/ev_vel_control.cpp
		EKF/aid_sources/external_vision/ev_yaw_control.cpp
	)
	list(APPEND EKF_MODULE_PARAMS params_external_vision.yaml)
endif()

if(CONFIG_EKF2_GNSS)
	list(APPEND EKF_SRCS
		EKF/aid_sources/gnss/gnss_checks.cpp
		EKF/aid_sources/gnss/gnss_height_control.cpp
		EKF/aid_sources/gnss/gps_control.cpp
	)

	if(CONFIG_EKF2_GNSS_YAW)
		list(APPEND EKF_SRCS EKF/aid_sources/gnss/gnss_yaw_control.cpp)
	endif()

	list(APPEND EKF_LIBS yaw_estimator)

	list(APPEND EKF_MODULE_PARAMS params_gnss.yaml)
endif()

if(CONFIG_EKF2_GRAVITY_FUSION)
	list(APPEND EKF_SRCS EKF/aid_sources/gravity/gravity_fusion.cpp)
	list(APPEND EKF_MODULE_PARAMS params_gravity.yaml)
endif()

if(CONFIG_EKF2_MAGNETOMETER)
	list(APPEND EKF_SRCS
		EKF/aid_sources/magnetometer/mag_control.cpp
		EKF/aid_sources/magnetometer/mag_fusion.cpp
	)
	list(APPEND EKF_MODULE_PARAMS params_magnetometer.yaml)
endif()

if(CONFIG_EKF2_OPTICAL_FLOW)
	list(APPEND EKF_SRCS
		EKF/aid_sources/optical_flow/optical_flow_control.cpp
		EKF/aid_sources/optical_flow/optical_flow_fusion.cpp
	)
	list(APPEND EKF_MODULE_PARAMS params_optical_flow.yaml)
endif()

if(CONFIG_EKF2_RANGE_FINDER)
	list(APPEND EKF_SRCS
		EKF/aid_sources/range_finder/range_finder_consistency_check.cpp
		EKF/aid_sources/range_finder/range_height_control.cpp
		EKF/aid_sources/range_finder/range_height_fusion.cpp
		EKF/aid_sources/range_finder/sensor_range_finder.cpp
	)
	list(APPEND EKF_MODULE_PARAMS params_range_finder.yaml)
endif()

if(CONFIG_EKF2_SIDESLIP)
	list(APPEND EKF_SRCS EKF/aid_sources/sideslip/sideslip_fusion.cpp)
	list(APPEND EKF_MODULE_PARAMS params_sideslip.yaml)
endif()

if(CONFIG_EKF2_TERRAIN)
	list(APPEND EKF_SRCS EKF/terrain_control.cpp)
	list(APPEND EKF_MODULE_PARAMS params_terrain.yaml)
endif()

if(CONFIG_EKF2_UWB)
	list(APPEND EKF_SRCS
		EKF/aid_sources/uwb/uwb_control.cpp
		EKF/aid_sources/uwb/uwb_checks.cpp
	)
	list(APPEND EKF_MODULE_PARAMS params_uwb.yaml)
endif()

if(CONFIG_EKF2_WIND)
	list(APPEND EKF_SRCS EKF/wind.cpp)
	list(APPEND EKF_MODULE_PARAMS params_wind.yaml)
endif ()

add_subdirectory(EKF)

px4_add_module(
	MODULE modules__ekf2
	MAIN ekf2
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
		#-DDEBUG_BUILD
		#-O0
	INCLUDES
		EKF
		EKF/aid_sources
		${EKF_GENERATED_DERIVATION_INCLUDE_PATH}
	PRIORITY
		"SCHED_PRIORITY_MAX - 18" # max priority below high priority WQ threads
	STACK_MAX
		3600
	SRCS
		${EKF_SRCS}

		EKF2.cpp
		EKF2.hpp
		EKF2Selector.cpp
		EKF2Selector.hpp

		${EKF_GENERATED_FILES}

	MODULE_CONFIG
		module.yaml
		params_gyro_bias.yaml
		params_accel_bias.yaml
		params_multi.yaml
		params_volatile.yaml
		params_selector.yaml
		${EKF_MODULE_PARAMS}

	DEPENDS
		geo
		hysteresis
		perf
		px4_work_queue
		world_magnetic_model

		${EKF_LIBS}
		lat_lon_alt
		bias_estimator
		output_predictor
	UNITY_BUILD
	)

if(BUILD_TESTING)
	add_subdirectory(test)
endif()
