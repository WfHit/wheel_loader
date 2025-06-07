px4_add_module(
    MODULE modules__limit_sensor
    MAIN limit_sensor
    COMPILE_FLAGS
    SRCS
        LimitSensor.cpp
        LimitSensor.hpp
    DEPENDS
        parameters
        perf
        px4_work_queue
)
