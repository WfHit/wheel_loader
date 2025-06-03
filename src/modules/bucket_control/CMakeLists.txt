px4_add_module(
    MODULE modules__bucket_control
    MAIN bucket_control
    SRCS
        bucket_control.cpp
    DEPENDS
        px4_work_queue
        pid
        mathlib
)
