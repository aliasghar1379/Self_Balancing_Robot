# Self-Balancing Robot

A two-wheeled self-balancing robot using PID control system, MPU6050 gyroscope/accelerometer, and Arduino microcontroller.

![Robot Overview](https://github.com/aliasghar1379/Self_Balancing_Robot/assets/59472710/32a254ef-d3ab-4f3a-ac32-216e2b400deb)

![Robot in Action](https://github.com/aliasghar1379/Self_Balancing_Robot/assets/59472710/31d34679-16f4-4db8-aaa6-b23660d67195)

## Project Overview

This project implements a self-balancing robot that maintains its equilibrium on two wheels using PID (Proportional-Integral-Derivative) control. The system utilizes MPU6050's gyroscope and accelerometer data to measure orientation and implement real-time balance adjustments.

## Features

- **PID Control System**
  - Real-time balance adjustment
  - Tunable PID parameters for optimal performance
  - Smooth motion control

- **Sensor Integration**
  - MPU6050 6-axis gyroscope and accelerometer
  - Real-time orientation tracking
  - X, Y, Z axis measurement

- **Motor Control**
  - Dual motor driver implementation
  - Precise speed and direction control
  - Quick response to balance adjustments

## Hardware Components

- Arduino Microcontroller
- MPU6050 Gyroscope/Accelerometer Module
- L298N Motor Driver (or equivalent)
- 2x DC Motors with wheels
- Battery Pack
- Chassis Structure
- Connecting wires
- (List any additional components)

## Circuit Connections

### MPU6050 to Arduino
- VCC → 5V
- GND → GND
- SDA → A4
- SCL → A5

### Motor Driver to Arduino
- IN1 → Digital Pin (specify)
- IN2 → Digital Pin (specify)
- IN3 → Digital Pin (specify)
- IN4 → Digital Pin (specify)
- ENA → PWM Pin (specify)
- ENB → PWM Pin (specify)

## Software Requirements

- Arduino IDE
- Required Libraries:
  - Wire.h
  - MPU6050.h

## PID Implementation

The robot uses a PID control algorithm:
```cpp
output = Kp * error + Ki * integral + Kd * derivative
```

Where:
- Kp: Proportional gain
- Ki: Integral gain
- Kd: Derivative gain
- error: Difference between desired and current angle
- integral: Sum of all errors over time
- derivative: Rate of change of error

## Setup Instructions

1. Assemble the hardware according to the circuit diagram
2. Install required libraries in Arduino IDE
3. Upload the code to Arduino
4. Calibrate the MPU6050 sensor
5. Tune PID parameters for optimal performance

## Tuning Guide

To achieve optimal balance:
1. Start with Kp to get basic response
2. Add Kd to reduce oscillations
3. Finally, add Ki to eliminate steady-state error
4. Fine-tune all parameters until desired performance is achieved

## Contributing

Contributions to improve the robot's performance or documentation are welcome! Please feel free to submit a Pull Request or open an issue for discussion.


## Acknowledgments

- Thanks to all contributors

## Contact

- GitHub: [@aliasghar1379](https://github.com/aliasghar1379)
