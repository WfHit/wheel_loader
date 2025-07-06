#include <px4_platform_common/encoder_hw.h>
#include <px4_platform_common/board_encoder_interface.h>
#include <board_config.h>

// Board encoder configurations
const encoder_hw_config_t g_board_encoder_configs[BOARD_NUM_QUADRATURE_ENCODERS] = {
	{
		.gpio_a = QENCODER_A_GPIO,
		.gpio_b = QENCODER_B_GPIO,
		.gpio_index = 0,  // No index pin
		.irq_priority = 5,  // Standard interrupt priority
		.pulses_per_revolution = QENCODER_DEFAULT_PPR,
		.mode = ENCODER_MODE_RELATIVE,
		.swap_channels = false,
		.enable_index = false,
	}
};

// Board encoder names
const char *g_board_encoder_names[BOARD_NUM_QUADRATURE_ENCODERS] = {
	"Rear Wheel Motor"
};
