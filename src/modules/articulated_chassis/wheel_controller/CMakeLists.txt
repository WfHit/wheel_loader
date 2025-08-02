px4_add_module(
	MODULE modules__articulated_chassis__wheel_controller
	MAIN wheel_controller
	STACK_MAIN 2048
	SRCS
		wheel_controller.cpp
		wheel_controller_main.cpp
	DEPENDS
		mathlib
		px4_work_queue
		PID
	MODULE_CONFIG
		module.yaml
)
