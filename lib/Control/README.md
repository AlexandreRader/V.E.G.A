# VEGA SC317 - Control Systems

This folder contains the hardware control systems for the VEGA SC317 autonomous robot.

## Files

- `HardwareControl.h` - Actuator management (servos and stepper motors)
  - PCA9685 servo driver for 4-wheel steering
  - TMC2208 stepper drivers for 6-wheel propulsion
  - Motor enable/disable and safety systems

## Dependencies

- Adafruit_PWMServoDriver (for servo control)
- TMCStepper (for stepper motor control)

## Usage

```cpp
#include "Control/HardwareControl.h"

ActuatorManager actuators;
actuators.begin();

// Set servo angles (radians)
actuators.setServoAngles(0.1, -0.1, 0.05, -0.05);

// Set stepper speeds (Hz)
actuators.setStepperSpeeds(100, 100, 100, 100, 100, 100);
```