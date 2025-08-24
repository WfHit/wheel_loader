# EKF2-Style SubscriptionMultiArray Usage Example

If you need to subscribe to multiple instances of a topic (like HBridge status from multiple motors), here's how EKF2 does it:

## Header Declaration (following EKF2 pattern)

```cpp
// Subscribe to all hbridge_status instances
uORB::SubscriptionMultiArray<hbridge_status_s> _hbridge_status_subs{ORB_ID::hbridge_status};
hrt_abstime _last_hbridge_status_update{0};
int _hbridge_status_selected{-1}; // Track which instance we're using
```

## Implementation (following EKF2 distance_sensor pattern)

```cpp
void BucketControl::updateHBridgeStatus()
{
    hbridge_status_s hbridge_status;

    // If no specific instance selected, find our motor's instance
    if (_hbridge_status_selected < 0) {
        const hrt_abstime timestamp_stale = math::max(hrt_absolute_time(), 100_ms) - 100_ms;

        if (_hbridge_status_subs.advertised()) {
            for (unsigned i = 0; i < _hbridge_status_subs.size(); i++) {
                if (_hbridge_status_subs[i].update(&hbridge_status)) {
                    // Check if this is our motor's status
                    if ((hbridge_status.timestamp != 0) &&
                        (hbridge_status.timestamp > timestamp_stale) &&
                        (hbridge_status.instance == _motor_index)) {

                        int nstatus = orb_group_count(ORB_ID(hbridge_status));
                        if (nstatus > 1) {
                            PX4_INFO("Bucket control selected hbridge_status:%d (motor %d, %d advertised)",
                                     i, _motor_index, nstatus);
                        }

                        _hbridge_status_selected = i;
                        _last_hbridge_status_update = hbridge_status.timestamp;
                        break;
                    }
                }
            }
        }
    }

    // Use the selected instance
    if (_hbridge_status_selected >= 0 &&
        _hbridge_status_subs[_hbridge_status_selected].update(&hbridge_status)) {

        if (hbridge_status.instance == _motor_index) {
            // Process our motor's status
            _last_hbridge_status_update = hbridge_status.timestamp;

            // Example: update motor state based on status
            if (hbridge_status.fault) {
                PX4_ERR("HBridge motor %d fault detected", _motor_index);
                // Handle fault...
            }

            // Update internal state
            _motor_current = hbridge_status.current;
            _motor_voltage = hbridge_status.voltage;
        }
    }
}
```

## Key EKF2 Patterns Applied:

1. **Dynamic Instance Selection**: Find the right instance at runtime
2. **Stale Data Check**: Ignore old data using timestamp comparison
3. **Array-like Access**: `_hbridge_status_subs[i]` to access specific instances
4. **Size Checking**: `_hbridge_status_subs.size()` to iterate through all instances
5. **Advertised Check**: `_hbridge_status_subs.advertised()` before accessing
6. **Instance Matching**: Use message content to identify the right instance

## Usage in Run() Method:

```cpp
void BucketControl::Run()
{
    // Update HBridge status from all motors
    updateHBridgeStatus();

    // Rest of control logic...
}
```

This pattern allows you to:
- Subscribe to all HBridge status messages
- Automatically find your motor's status messages
- Handle multiple motors in the same system
- Robust handling of instance changes/restarts
