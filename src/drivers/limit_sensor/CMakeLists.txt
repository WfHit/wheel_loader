px4_add_module(
    MODULE drivers__limit_sensor
    MAIN limit_sensor
    COMPILE_FLAGS
    SRCS
        LimitSensor.cpp
        LimitSensor.hpp
    DEPENDS
        perf
        px4_work_queue
)
