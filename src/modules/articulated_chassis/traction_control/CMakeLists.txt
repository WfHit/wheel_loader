px4_add_module(
    MODULE modules__traction_control
    MAIN traction_control
    SRCS
        traction_control.cpp
        traction_control.hpp
    MODULE_CONFIG
        module.yaml
    DEPENDS
        px4_work_queue
        PID
        mathlib
        perf
)
