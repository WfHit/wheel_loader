#include <px4_arch/quadrature_encoder.h>
#include <px4_arch/board_encoder_interface.h>
#include <board_config.h>

// Board encoder configurations
const quadrature_encoder_config_t g_board_encoder_configs[BOARD_NUM_QUADRATURE_ENCODERS] = {
	{
		.gpio_a = QENCODER1_A_GPIO_RAW,
		.gpio_b = QENCODER1_B_GPIO_RAW,
		.gpio_index = 0,  // No index pin
		.irq_priority = 5,  // Standard interrupt priority
		.pulses_per_revolution = QENCODER_DEFAULT_PPR,
		.mode = ENCODER_MODE_RELATIVE,
		.swap_channels = false,
		.enable_index = false,
	},
	{
		.gpio_a = QENCODER2_A_GPIO_RAW,
		.gpio_b = QENCODER2_B_GPIO_RAW,
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
	"Front Wheel Motor",
	"Bucket Motor"
};
