#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/parameter_update.h>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "quadencoder_common.h"
#include "../lib/quad_encoder/quadencoder_ioctl.h"

class QuadEncoder : public ModuleBase<QuadEncoder>
{
public:
	QuadEncoder();
	~QuadEncoder() override = default;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void run() override;
	bool init();

private:
	// Publications
	uORB::PublicationMulti<sensor_quad_encoder_s> _quad_encoder_pub{ORB_ID(sensor_quad_encoder)};

	// Subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// Encoder instances
	struct quadencoder_dev_s *_encoder_dev{nullptr};
	int _encoder_fd{-1};
	int _instance{0};

	// Parameters
	param_t _param_pin_a;
	param_t _param_pin_b;
	param_t _param_pin_z;
	param_t _param_mode;
	param_t _param_ppr;
	param_t _param_wheel_radius;
	param_t _param_enabled;

	// Parameter values
	int32_t _pin_a{0};
	int32_t _pin_b{1};
	int32_t _pin_z{-1};
	int32_t _mode{4};
	int32_t _ppr{1024};
	float _wheel_radius{0.0f};
	int32_t _enabled{0};

	void update_params();
	void publish_encoder_data();
	int initialize_encoder();
};

QuadEncoder::QuadEncoder()
{
	// Initialize parameter handles
	_param_pin_a = param_find("QUADENC_PIN_A");
	_param_pin_b = param_find("QUADENC_PIN_B");
	_param_pin_z = param_find("QUADENC_PIN_Z");
	_param_mode = param_find("QUADENC_MODE");
	_param_ppr = param_find("QUADENC_PPR");
	_param_wheel_radius = param_find("QUADENC_WHEEL_R");
	_param_enabled = param_find("QUADENC_EN");
}

bool QuadEncoder::init()
{
	// Update parameters
	update_params();

	// Check if encoder is enabled
	if (_enabled == 0) {
		PX4_INFO("Quadrature encoder disabled (QUADENC_EN=0)");
		return false;
	}

	// Initialize encoder
	if (initialize_encoder() < 0) {
		return false;
	}

	return true;
}

int QuadEncoder::initialize_encoder()
{
	char dev_path[32];
	snprintf(dev_path, sizeof(dev_path), "/dev/qe%d", _instance);

	_encoder_fd = open(dev_path, O_RDONLY);

	if (_encoder_fd < 0) {
		PX4_ERR("Failed to open %s", dev_path);
		return -1;
	}

	// Configure encoder
	if (ioctl(_encoder_fd, QEIOC_SETMODE, _mode) < 0) {
		PX4_WARN("Failed to set encoder mode");
	}

	if (ioctl(_encoder_fd, QEIOC_SETRESOLUTION, _ppr) < 0) {
		PX4_WARN("Failed to set encoder resolution");
	}

	// Enable/disable index based on pin configuration
	if (_pin_z >= 0) {
		ioctl(_encoder_fd, QEIOC_ENABLEINDEX, 0);
	} else {
		ioctl(_encoder_fd, QEIOC_DISABLEINDEX, 0);
	}

	// Reset encoder position
	ioctl(_encoder_fd, QEIOC_POSITION, 0);

	PX4_INFO("Quadrature encoder %d initialized (pins A:%ld B:%ld Z:%ld, mode:X%ld, PPR:%ld)",
		 _instance, (long)_pin_a, (long)_pin_b, (long)_pin_z, (long)_mode, (long)_ppr);

	return 0;
}

void QuadEncoder::run()
{
	// Update parameters
	update_params();

	// Main loop
	while (!should_exit()) {
		// Check for parameter updates
		if (_parameter_update_sub.updated()) {
			parameter_update_s update;
			_parameter_update_sub.copy(&update);
			update_params();
		}

		// Read encoder and publish
		publish_encoder_data();

		// Run at 200Hz
		px4_usleep(5000);
	}

	if (_encoder_fd >= 0) {
		close(_encoder_fd);
	}
}

void QuadEncoder::publish_encoder_data()
{
	if (_encoder_fd < 0) {
		return;
	}

	int32_t position = 0;

	// Read encoder position
	if (read(_encoder_fd, &position, sizeof(position)) == sizeof(position)) {
		sensor_quad_encoder_s encoder_data{};

		encoder_data.timestamp = hrt_absolute_time();
		encoder_data.device_id = _instance;
		encoder_data.position = position;

		// Get velocity via ioctl
		int32_t velocity = 0;
		ioctl(_encoder_fd, QEIOC_GETVELOCITY, &velocity);
		encoder_data.velocity = velocity;

		// Calculate angle
		encoder_data.angle = (float)position * 2.0f * M_PI_F / (float)_ppr;
		encoder_data.angular_velocity = (float)velocity * 2.0f * M_PI_F / (float)_ppr;

		// Calculate linear values if wheel radius is set
		if (_wheel_radius > 0.0f) {
			encoder_data.distance = encoder_data.angle * _wheel_radius;
			encoder_data.speed = encoder_data.angular_velocity * _wheel_radius;
		} else {
			encoder_data.distance = 0.0f;
			encoder_data.speed = 0.0f;
		}

		// Set configuration
		encoder_data.pulses_per_rev = _ppr;
		encoder_data.mode = _mode;
		encoder_data.status = 0;
		encoder_data.error_count = 0;
		encoder_data.index_detected = false;

		// Publish
		_quad_encoder_pub.publish(encoder_data);
	}
}

void QuadEncoder::update_params()
{
	// Update all parameters
	param_get(_param_pin_a, &_pin_a);
	param_get(_param_pin_b, &_pin_b);
	param_get(_param_pin_z, &_pin_z);
	param_get(_param_mode, &_mode);
	param_get(_param_ppr, &_ppr);
	param_get(_param_wheel_radius, &_wheel_radius);
	param_get(_param_enabled, &_enabled);

	// Apply new configuration to encoder
	if (_encoder_fd >= 0) {
		ioctl(_encoder_fd, QEIOC_SETMODE, _mode);
		ioctl(_encoder_fd, QEIOC_SETRESOLUTION, _ppr);
	}
}

int QuadEncoder::task_spawn(int argc, char *argv[])
{
	int instance = 0;
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "i:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'i':
			instance = atoi(myoptarg);
			break;

		default:
			print_usage();
			return -1;
		}
	}

	QuadEncoder *instance_ptr = new QuadEncoder();

	if (instance_ptr) {
		_object.store(instance_ptr);
		instance_ptr->_instance = instance;
		_task_id = task_id_is_work_queue;

		if (instance_ptr->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance_ptr;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int QuadEncoder::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int QuadEncoder::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Quadrature encoder driver supporting multiple encoder instances.
Publishes encoder data to the 'sensor_quad_encoder' uORB topic.

### Examples
Start encoder instance 0:
$ quad_encoder start -i 0

Start encoder instance 1:
$ quad_encoder start -i 1
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("quad_encoder", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_INT('i', 0, 0, 3, "Instance number", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int quad_encoder_main(int argc, char *argv[])
{
	return QuadEncoder::main(argc, argv);
}
