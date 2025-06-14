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
############################################################################

add_subdirectory(bias_estimator)
add_subdirectory(output_predictor)

set(EKF_LIBS)
set(EKF_SRCS)
list(APPEND EKF_SRCS
	control.cpp
	covariance.cpp
	ekf.cpp
	ekf_helper.cpp
	estimator_interface.cpp
	height_control.cpp
	velocity_fusion.cpp
	position_fusion.cpp
	yaw_fusion.cpp

	imu_down_sampler/imu_down_sampler.cpp

	aid_sources/fake_height_control.cpp
	aid_sources/fake_pos_control.cpp
	aid_sources/ZeroGyroUpdate.cpp
	aid_sources/ZeroVelocityUpdate.cpp
	aid_sources/zero_innovation_heading_update.cpp
)

if(CONFIG_EKF2_AIRSPEED)
	list(APPEND EKF_SRCS aid_sources/airspeed/airspeed_fusion.cpp)
endif()

if(CONFIG_EKF2_AUX_GLOBAL_POSITION)
	list(APPEND EKF_SRCS aid_sources/aux_global_position/aux_global_position.cpp)
endif()

if(CONFIG_EKF2_AUXVEL)
	list(APPEND EKF_SRCS aid_sources/auxvel/auxvel_fusion.cpp)
endif()

if(CONFIG_EKF2_BAROMETER)
	list(APPEND EKF_SRCS
		aid_sources/barometer/baro_height_control.cpp
	)
endif()

if(CONFIG_EKF2_DRAG_FUSION)
	list(APPEND EKF_SRCS aid_sources/drag/drag_fusion.cpp)
endif()

if(CONFIG_EKF2_EXTERNAL_VISION)
	list(APPEND EKF_SRCS
		aid_sources/external_vision/ev_control.cpp
		aid_sources/external_vision/ev_height_control.cpp
		aid_sources/external_vision/ev_pos_control.cpp
		aid_sources/external_vision/ev_vel_control.cpp
		aid_sources/external_vision/ev_yaw_control.cpp
	)
endif()

if(CONFIG_EKF2_GNSS)
	list(APPEND EKF_SRCS
		aid_sources/gnss/gnss_checks.cpp
		aid_sources/gnss/gnss_height_control.cpp
		aid_sources/gnss/gps_control.cpp
	)

	if(CONFIG_EKF2_GNSS_YAW)
		list(APPEND EKF_SRCS aid_sources/gnss/gnss_yaw_control.cpp)
	endif()

	add_subdirectory(yaw_estimator)
	list(APPEND EKF_LIBS yaw_estimator)
endif()

if(CONFIG_EKF2_GRAVITY_FUSION)
	list(APPEND EKF_SRCS aid_sources/gravity/gravity_fusion.cpp)
endif()

if(CONFIG_EKF2_MAGNETOMETER)
	list(APPEND EKF_SRCS
		aid_sources/magnetometer/mag_control.cpp
		aid_sources/magnetometer/mag_fusion.cpp
	)
endif()

if(CONFIG_EKF2_OPTICAL_FLOW)
	list(APPEND EKF_SRCS
		aid_sources/optical_flow/optical_flow_control.cpp
		aid_sources/optical_flow/optical_flow_fusion.cpp
	)
endif()

if(CONFIG_EKF2_RANGE_FINDER)
	list(APPEND EKF_SRCS
		aid_sources/range_finder/range_finder_consistency_check.cpp
		aid_sources/range_finder/range_height_control.cpp
		aid_sources/range_finder/range_height_fusion.cpp
		aid_sources/range_finder/sensor_range_finder.cpp
	)
endif()

if(CONFIG_EKF2_SIDESLIP)
	list(APPEND EKF_SRCS aid_sources/sideslip/sideslip_fusion.cpp)
endif()

if(CONFIG_EKF2_UWB)
	list(APPEND EKF_SRCS
		aid_sources/uwb/uwb_control.cpp
		aid_sources/uwb/uwb_checks.cpp
	)
endif()

if(CONFIG_EKF2_TERRAIN)
	list(APPEND EKF_SRCS terrain_control.cpp)
endif()

if(CONFIG_EKF2_WIND)
	list(APPEND EKF_SRCS wind.cpp)
endif ()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
add_library(ecl_EKF
	${EKF_SRCS}
)

add_dependencies(ecl_EKF prebuild_targets)
target_include_directories(ecl_EKF PUBLIC ${EKF_GENERATED_DERIVATION_INCLUDE_PATH})

target_link_libraries(ecl_EKF
	PRIVATE
		bias_estimator
		geo
		lat_lon_alt
		output_predictor
		world_magnetic_model
		${EKF_LIBS}
)

target_compile_options(ecl_EKF PRIVATE -fno-associative-math)
