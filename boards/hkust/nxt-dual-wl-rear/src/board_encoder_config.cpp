#include <px4_arch/quad_encoder.h>
#include <px4_arch/board_encoder.h>
#include <board_config.h>

// Board encoder configurations
const quad_encoder_config_t g_board_encoder_configs[BOARD_NUM_QUADRATURE_ENCODERS] = {
	{
		.gpio_a = QENCODER_A_GPIO,
		.gpio_b = QENCODER_B_GPIO,
		.pulses_per_revolution = QENCODER_DEFAULT_PPR,
	}
};

// Board encoder names
const char *g_board_encoder_names[BOARD_NUM_QUADRATURE_ENCODERS] = {
	"Rear Wheel Motor"
};
