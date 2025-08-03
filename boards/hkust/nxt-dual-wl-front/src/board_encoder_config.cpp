#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>
#include <board_config.h>

// Board encoder configurations
const quad_encoder_config_t g_board_encoder_configs[BOARD_NUM_QUADRATURE_ENCODERS] = {
	{
		.gpio_a = QENCODER1_A_GPIO_RAW,
		.gpio_b = QENCODER1_B_GPIO_RAW,
		.overflow_count = 0,  // 0 = no auto-reset
	},
	{
		.gpio_a = QENCODER2_A_GPIO_RAW,
		.gpio_b = QENCODER2_B_GPIO_RAW,
		.overflow_count = 0,  // 0 = no auto-reset
	}
};

// Board encoder names
const char *g_board_encoder_names[BOARD_NUM_QUADRATURE_ENCODERS] = {
	"Front Wheel Motor",
	"Bucket Motor"
};
