px4_add_module(
	MODULE modules__chassis_control
	MAIN chassis_control
	STACK_MAIN 2048
	SRCS
		chassis_control.cpp
		chassis_control_main.cpp
	DEPENDS
		modules__articulated_chassis__slip_estimator
		modules__articulated_chassis__load_aware_torque
	)
