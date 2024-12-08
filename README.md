## DrawBot

### Overview
Two-MCU drawing robot system consisting of a touch display interface and a differential drive robot. The robot follows drawing paths input through the touch display, with real-time motion control and pen up/down capabilities.

### Hardware
- Touch Interface Unit:
  - STM32L4 MCU
  - HX8357 TFT Display w/ resistive touch
  - XBee wireless module
  
- Robot Unit:
  - STM32F4 MCU
  - L3GD20H gyroscope
  - LSM303D IMU
  - 2x DC motors with H-bridge
  - Servo for pen control
  - XBee wireless module

### Software Architecture
#### Display Unit (`/touch_display/`)
- Touch input handling with calibration
- Display graphics and UI management
- Coordinate transformation and path processing
- XBee communication protocol implementation

#### Robot Unit (`/robot/`)
- Motor control with PID feedback
- Gyroscope-based heading control
- IMU data fusion using Kalman filter
- Path execution and pen control
- Real-time motion planning

### Key Features
- Calibrated touch-to-robot coordinate mapping
- Closed-loop heading control
- Motion smoothing and path optimization
- Real-time wireless command streaming
- Basic UI with clear/toggle controls

### Building and Flashing
Standard STM32CubeIDE project structure. Build and flash separately for each unit:
1. Open respective project in STM32CubeIDE
2. Build project
3. Flash to corresponding MCU using ST-Link

### Authors
Madhav Sharma and Lucas Biondo-Savin

EECS 373 - Embedded Systems Design

Fall 2024
