#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_command_ack.h>

#include <lib/mathlib/mathlib.h>
#include <lib/matrix/matrix/math.hpp>

#include <cstring>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>

// UART communication includes
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using namespace time_literals;

class VLAProxy : public ModuleBase<VLAProxy>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	VLAProxy();
	~VLAProxy() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::run() */
	void Run() override;

	bool init();

private:
	// VLA Command Types
	enum class VLACommand {
		START,
		STOP,
		PAUSE,
		RESUME
	};

	// Robot status structure to send to VLA
	struct RobotStatus {
		uint64_t timestamp;
		float position[3];      // x, y, z
		float velocity[3];      // vx, vy, vz
		float quaternion[4];    // w, x, y, z
		float angular_vel[3];   // roll_rate, pitch_rate, yaw_rate
		uint8_t armed;
		uint8_t flight_mode;
		float battery_percentage;
	};

	// VLA trajectory waypoint
	struct VLAWaypoint {
		float position[3];
		float velocity[3];
		float acceleration[3];
		float yaw;
		float yaw_rate;
		uint64_t timestamp_us;
	};

	// Methods
	void process_robot_status();
	void process_vla_trajectory();
	void send_status_to_vla(const RobotStatus &status);
	bool receive_trajectory_from_vla(std::deque<VLAWaypoint> &trajectory);
	void publish_trajectory_setpoint(const VLAWaypoint &waypoint);
	void handle_vla_command(VLACommand cmd);

	// Communication interface (can be replaced with actual implementation)
	bool init_vla_connection();
	void close_vla_connection();

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::VLA_PROXY_EN>) _param_vla_proxy_enabled,
		(ParamInt<px4::params::VLA_UART_EN>) _param_vla_uart_enabled,
		(ParamInt<px4::params::VLA_UART_BAUD>) _param_vla_uart_baud,
		(ParamInt<px4::params::VLA_UART_DEV>) _param_vla_uart_dev,
		(ParamFloat<px4::params::VLA_UPDATE_RATE>) _param_vla_update_rate,
		(ParamInt<px4::params::VLA_TIMEOUT_MS>) _param_vla_timeout_ms
	)

	// Subscriptions
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};

	// Publications
	uORB::Publication<trajectory_setpoint_s> _trajectory_setpoint_pub{ORB_ID(trajectory_setpoint)};
	uORB::Publication<vehicle_command_ack_s> _vehicle_command_ack_pub{ORB_ID(vehicle_command_ack)};

	// State
	bool _vla_connected{false};
	bool _vla_running{false};
	std::deque<VLAWaypoint> _trajectory_buffer;
	std::mutex _trajectory_mutex;

	// Communication socket (placeholder - implement based on your VLA interface)
	int _vla_uart_fd{-1};
	char _uart_device[20]{"/dev/ttyS3"};  // Default UART device

	// Timing
	hrt_abstime _last_status_sent{0};
	hrt_abstime _last_trajectory_received{0};

	static constexpr uint32_t SCHEDULE_INTERVAL{20_ms};  // 50Hz update rate
};
