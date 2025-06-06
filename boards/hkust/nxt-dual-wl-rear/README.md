# NXT-Dual-WL-Rear Board

## Overview

The NXT-Dual-WL-Rear is a specialized controller board for wheel loader applications, designed to control:
- **Rear wheel** - Propulsion control for the rear axle and differential
- **Boom** - Hydraulic boom control for lifting and positioning operations

This board is based on the nxt-dual-wl platform and includes specialized modules and configurations for rear-end wheel loader operations.

## Key Features

- Dual wheel quadrature encoder support for precise rear wheel control
- AS5600 magnetic encoder for boom position feedback
- H-bridge motor drivers (DRV8701) for boom hydraulic control
- BMI088 IMU for attitude and motion sensing
- SPL06 barometric pressure sensor
- CAN bus communication for coordination with front controller
- PWM outputs for servo and motor control

## Modules Enabled

- **boom_control** - Dedicated boom position and hydraulic control
- **rear_wheel_control** - Rear axle drive and differential control
- Standard PX4 navigation and control modules

## Pin Configuration

### PWM Outputs
- PWM1-2: Rear wheel motor control
- PWM3-4: Boom hydraulic valve control
- PWM5-8: Auxiliary outputs

### I2C Bus
- I2C1: AS5600 magnetic encoder (boom position)
- I2C2: External sensor expansion

### CAN Bus
- CAN1: Inter-controller communication (rear ↔ front)
- CAN2: External device communication

## Parameters

Key parameters for rear wheel loader control:
- `BOOM_CTRL_EN`: Enable boom control module
- `WL_REAR_WHEEL_EN`: Enable rear wheel control
- `WL_REAR_WHEEL_KP/KI/KD`: Rear wheel PID parameters

## Usage

This board should be mounted in the rear section of the wheel loader and connected to:
1. Rear wheel drive motors
2. Boom hydraulic valves
3. Rear wheel position encoders
4. Boom position sensor
5. CAN bus to front controller
