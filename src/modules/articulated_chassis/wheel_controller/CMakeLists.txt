px4_add_module(
    MODULE modules__articulated_chassis__wheel_controller
    MAIN wheel_controller
    COMPILE_FLAGS
        -Wno-unused-parameter
    SRCS
        WheelController.cpp
        wheel_controller_main.cpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
        pid
)
