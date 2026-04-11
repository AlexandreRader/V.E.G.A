# VEGA SC317 - Sensor Systems

This folder contains all sensor management systems for obstacle detection and precision navigation.

## Files

- `Detection.h` - ToF sensor management for obstacle detection
  - 4x VL53L3CX Time-of-Flight sensors
  - Multi-address I2C configuration
  - Obstacle detection and emergency stop logic

- `IRSensors.h` - IR docking sensor for precision target approach
  - 38kHz modulated IR sensor
  - Distance measurement and target detection
  - Precision docking guidance

## Dependencies

- VL53L3CX (for ToF sensors)

## Usage

```cpp
#include "Sensors/Detection.h"
#include "Sensors/IRSensors.h"

ToFManager tof_sensors;
IRDockingSensor ir_sensor(PIN_IR_DOCKING);

tof_sensors.begin();
ir_sensor.begin();

// Check for obstacles
if (tof_sensors.hasObstacleFront()) {
    // Emergency stop logic
}

// Check docking status
if (ir_sensor.isDocked()) {
    // Mission complete
}
```