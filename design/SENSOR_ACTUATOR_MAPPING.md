# Sensor and Actuator Message Mapping

This document describes the correct sensor and actuator message mapping for the wheel loader control system.

## Sensor Messages

### Magnetic Rotary Encoder
- **Message**: `sensor_mag_encoder`
- **Usage**: Absolute angle measurement for boom position
- **Hardware**: AS5600-compatible magnetic rotary encoders
- **Controllers using this**:
  - BoomControl (for boom angle measurement)

### Quadrature Encoder
- **Message**: `sensor_quad_encoder`
- **Usage**: Wheel speed and linear actuator position measurement
- **Controllers using this**:
  - WheelController (for wheel speed feedback)
  - BucketControl (for bucket actuator position)

### Limit Switches
- **Message**: `limit_sensor`
- **Usage**: End-stop detection for actuators
- **Controllers using this**:
  - BucketControl (for bucket travel limits)

## Actuator Messages

### HBridge Driver (DRV8701)
- **Message**: `hbridge_cmd`
- **Usage**: Motor control for wheels and hydraulic valves
- **Controllers using this**:
  - WheelController (for drive motor control)
  - BoomControl (for boom hydraulic valve control)
  - BucketControl (for bucket hydraulic valve control)

### ST3125 Servo
- **Message**: `robotic_servo_command`
- **Usage**: Smart servo control with internal PID and position feedback
- **Controllers using this**:
  - SteeringController (for steering servo control with internal angle feedback)

## Updated Controller Mappings

### 1. WheelController
**Before:**
```cpp
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/actuator_motors.h>
uORB::Subscription _wheel_encoders_sub{ORB_ID(wheel_encoders)};
uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
```

**After:**
```cpp
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/hbridge_cmd.h>
uORB::Subscription _sensor_quad_encoder_sub{ORB_ID(sensor_quad_encoder)};
uORB::Publication<hbridge_cmd_s> _hbridge_cmd_pub{ORB_ID(hbridge_cmd)};
```

### 2. SteeringController
**Uses**: `robotic_servo_command` ✓ (servo provides internal position feedback)
**Note**: No external sensor needed - the ST3125 servo returns its own angle position

### 3. BoomControl
**Before:**
```cpp
#include <uORB/topics/sensor_mag.h>
uORB::SubscriptionData<sensor_mag_s> _as5600_sub{ORB_ID(sensor_mag)};
```

**After:**
```cpp
#include <uORB/topics/sensor_mag_encoder.h>
uORB::SubscriptionData<sensor_mag_encoder_s> _mag_encoder_sub{ORB_ID(sensor_mag_encoder)};
```
**Already using**: `hbridge_cmd` ✓

### 4. BucketControl
**Before:**
```cpp
#include <uORB/topics/wheel_encoders.h>
#include <uORB/topics/actuator_motors.h>
uORB::Subscription _wheel_encoders_sub{ORB_ID(wheel_encoders)};
uORB::Publication<actuator_motors_s> _actuator_motors_pub{ORB_ID(actuator_motors)};
```

**After:**
```cpp
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/hbridge_cmd.h>
uORB::Subscription _sensor_quad_encoder_sub{ORB_ID(sensor_quad_encoder)};
uORB::Publication<hbridge_cmd_s> _hbridge_cmd_pub{ORB_ID(hbridge_cmd)};
```

## Implementation Notes

### HBridge Command Format
```cpp
void setMotorCommand(float command) {
    hbridge_cmd_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.channel = _param_motor_index.get();  // 0 or 1
    cmd.speed = math::constrain(command, -1.0f, 1.0f);
    cmd.direction = (command >= 0.0f) ? 0 : 1;  // 0=forward, 1=reverse
    cmd.enable_request = true;
    cmd.control_mode = 1;  // NORMAL mode
    _hbridge_cmd_pub.publish(cmd);
}
```

### Magnetic Encoder Reading
```cpp
void readMagneticEncoder() {
    sensor_mag_encoder_s encoder_data;
    if (_mag_encoder_sub.update(&encoder_data)) {
        _current_angle_rad = encoder_data.angle;
        _sensor_valid = (encoder_data.magnet_detected == 1) &&
                       (encoder_data.magnet_too_strong == 0) &&
                       (encoder_data.magnet_too_weak == 0);
    }
}
```

### Robotic Servo Reading (ST3125 internal feedback)
```cpp
void processServoFeedback() {
    servo_feedback_s feedback{};
    if (_servo_feedback_sub.update(&feedback)) {
        if (feedback.id == ST3125_SERVO_ID) {
            _current_angle_rad = feedback.position;
            _current_velocity_rad_s = feedback.velocity;
            _current_current_a = feedback.current;
            _servo_feedback_valid = true;
        }
    }
}
```
```cpp
void readQuadEncoder() {
    sensor_quad_encoder_s encoder_data;
    if (_sensor_quad_encoder_sub.update(&encoder_data)) {
        // Find the correct encoder instance
        for (int i = 0; i < encoder_data.count; i++) {
            if (encoder_data.valid[i]) {
                _encoder_position = encoder_data.position[i];
                _encoder_velocity = encoder_data.velocity[i];
                break;
            }
        }
    }
}
```

## Next Steps

1. ✅ Update the implementation files (.cpp) to match these header changes
2. ✅ Test compilation with the new message types
3. ✅ Verify parameter mappings for sensor/actuator indices
4. ✅ Update any remaining references to old message types
5. Update steering controller to remove AS5600 sensor usage (ST3125 provides internal feedback)
6. Test hardware integration with correct sensor/actuator assignments
7. Validate control performance with new message mappings
