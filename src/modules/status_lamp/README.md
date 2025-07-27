# Status Lamp Controller Module

This module controls a status lamp with 8 sequential states by receiving uORB commands and outputting trigger pulses on a GPIO pin.

## Features

- 8 lamp states: OFF, GREEN, YELLOW, RED, BLUE, WHITE, PURPLE, ORANGE
- Sequential state changes only (cycles through states in order)
- Configurable pulse width and logic level
- GPIO trigger output on PH9 pin (X7Plus-WL board)
- Board compatibility checking (graceful handling of unsupported boards)

## Board Support

The module checks for `BOARD_HAS_STATUS_LAMP_TRIGGER` definition in the board configuration:
- **Supported boards**: X7Plus-WL (GPIO PH9)
- **Unsupported boards**: Module will start but GPIO operations are disabled

### Adding Support for New Boards

To add support for a new board, add these definitions to the board's `board_config.h`:

```cpp
/* STATUS LAMP TRIGGER */
#define GPIO_STATUS_LAMP_TRIGGER /* YOUR_PIN */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTYOUR|GPIO_PINYOUR)
#define BOARD_HAS_STATUS_LAMP_TRIGGER 1
```

And add the GPIO to the board's initialization list:
```cpp
#define PX4_GPIO_INIT_LIST { \
    /* ...existing GPIOs... */ \
    GPIO_STATUS_LAMP_TRIGGER, \
    /* ...more GPIOs... */ \
}
```

## Parameters

- `SLAMP_PULSE_MS`: Trigger pulse width in milliseconds (10-1000ms, default 100ms)
- `SLAMP_INVERT`: Invert logic level (0=active high, 1=active low, default 0)
- `SLAMP_INTERVAL_MS`: Interval between consecutive pulses (50-2000ms, default 200ms)

## Usage

### Start the module
```bash
status_lamp start
```

### Check status
```bash
status_lamp status
```

### Manual state control
```bash
# Set state to GREEN (state 1)
status_lamp set 1

# Set state to RED (state 3)
status_lamp set 3

# Set state to OFF (state 0)
status_lamp set 0
```

### Send state change commands via uORB
```bash
# Set state to GREEN (state 1)
uorb publish status_lamp_command '{"timestamp":0,"target_state":1}'

# Set state to RED (state 3)
uorb publish status_lamp_command '{"timestamp":0,"target_state":3}'
```

## Hardware Connection

Connect your status lamp trigger input to PWM channel 14 (PE6 pin) on the X7Plus-WL board.

## GPIO Configuration

The GPIO is defined in the board configuration as:
```cpp
#define GPIO_STATUS_LAMP_TRIGGER /* PE6 */ (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR|GPIO_PORTE|GPIO_PIN6)
```

## State Sequence

The lamp can only change states sequentially. If you request a non-adjacent state, the module will output multiple pulses to reach the target state:

- From OFF (0) to RED (3): outputs 3 pulses
- From RED (3) to GREEN (1): outputs 6 pulses (wraps around: 3→4→5→6→7→0→1)
