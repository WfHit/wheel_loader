px4_add_module(
	MODULE drivers__quad_encoder
	MAIN quad_encoder
	COMPILE_FLAGS
	SRCS
		quad_encoder_main.cpp
		quad_encoder_common.c
		quad_encoder_gpio.c
		quad_encoder_timer.c
	DEPENDS
		platforms__common
	)
