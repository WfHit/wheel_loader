px4_add_module(
	MODULE drivers__quad_encoder
	MAIN quad_encoder
	COMPILE_FLAGS
	SRCS
		quad_encoder_main.cpp
		quadencoder_common.c
		quadencoder_gpio.c
		quadencoder_timer.c
		quadencoder_params.c
	DEPENDS
		platforms__common
	)
