# Line Follower Robot

A high-performance line follower robot implementation for Raspberry Pi Pico using SVK IR sensor array with multiplexer-based ADC reading.

## Features

1. **Line Following with PID Control** - Precise navigation using proportional-integral-derivative control algorithm
2. **Intersection Detection and Handling** - Advanced pattern recognition for various intersection types (T, cross, Y, turns)
3. **Multicore Support** - Dual-core processing for concurrent sensor reading and motor control
4. **Multiplexer-Based Sensor Reading** - 8 IR sensors connected through analog multiplexer to single ADC channel
5. **Adaptive Speed Control** - Dynamic speed adjustment based on path complexity and turn requirements
6. **Smart Line Recovery** - Systematic search patterns when line is lost
7. **Real-time State Management** - Interrupt-based button handling and state transitions

## Hardware Requirements

### Sensors
- SVK IR Sensor Array (8 sensors)
- Analog multiplexer (CD4051 or equivalent)
- Raspberry Pi Pico ADC (GPIO 26)

### Motor Driver
- TB6612FNG motor driver
- Two DC motors with encoders (optional)

### Pin Configuration
- **Sensor Select Pins**: GPIO 10 (S0), GPIO 11 (S1), GPIO 12 (S2)
- **ADC Input**: GPIO 26 (ADC0)
- **Motor Control**: GPIO 2-8 (direction and PWM pins)
- **Buttons**: GPIO 14 (calibration), GPIO 15 (start/stop)

## Software Architecture

### Core Components
- **main.c**: Entry point and Core 0 motor control loop
- **line_follower.c**: Sensor processing, PID control, and state management
- **line_follower.h**: Type definitions, constants, and function declarations

### Key Functions
- `read_line_sensors()`: Multiplexer-based sensor reading with 10μs settling time
- `calculate_pid()`: PID control with anti-windup protection
- `detect_line_pattern()`: Advanced pattern recognition for intersections
- `handle_intersection()`: Configurable intersection handling

## Building and Flashing

```bash
mkdir build
cd build
cmake ..
make
picotool load line_follower_robot.elf -f
```

## Operation

1. Power on the robot
2. Press calibration button to calibrate sensors
3. Press start/stop button to begin line following
4. Robot will automatically detect and navigate intersections

## Dependencies

- Raspberry Pi Pico SDK
- Hardware libraries: GPIO, PWM, ADC, multicore
