# Alternative Implementation: Low-Level uORB API

If you need explicit instance control (like assigning motor 0 to instance 0, motor 1 to instance 1), use this approach:

## Header Changes (bucket_control.hpp)

```cpp
// Replace PublicationMulti with low-level handle
// uORB::PublicationMulti<hbridge_command_s> _hbridge_command_pub{ORB_ID(hbridge_command)};
orb_advert_t _hbridge_command_adv{nullptr};
```

## Implementation Changes (bucket_control.cpp)

```cpp
void BucketControl::setMotorCommand(float command)
{
    hbridge_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.instance = _motor_index;
    cmd.duty_cycle = command;
    cmd.enable = true;

    if (_hbridge_command_adv == nullptr) {
        // Try to get the specific instance we want
        int desired_instance = _motor_index;
        _hbridge_command_adv = orb_advertise_multi(ORB_ID(hbridge_command), &cmd, &desired_instance);

        if (_hbridge_command_adv != nullptr) {
            PX4_INFO("Bucket motor %d got uORB instance %d", _motor_index, desired_instance);

            // Check if we got the instance we wanted
            if (desired_instance != _motor_index) {
                PX4_WARN("Bucket motor %d wanted instance %d but got %d",
                         _motor_index, _motor_index, desired_instance);
            }
        }
    } else {
        orb_publish(ORB_ID(hbridge_command), _hbridge_command_adv, &cmd);
    }
}
```

## Constructor Changes

```cpp
BucketControl::BucketControl() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _hbridge_command_adv(nullptr)  // Initialize to nullptr
{
    // existing constructor code...
}
```

## Destructor Changes

```cpp
BucketControl::~BucketControl()
{
    if (_hbridge_command_adv != nullptr) {
        orb_unadvertise(_hbridge_command_adv);
    }
}
```

## Key Points:

1. `orb_advertise_multi` updates the `desired_instance` parameter with the actually assigned instance
2. You should check if the assigned instance matches what you requested
3. If multiple motors request the same instance, only the first one gets it
4. This approach gives you explicit control but requires more error handling
