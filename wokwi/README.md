# VEGA SC317 - Wokwi Simulation

This directory contains the Wokwi simulation setup for testing the VEGA SC317 autonomous robot kinematics and path following algorithms.

## 🎯 What This Simulation Tests

- **Path Following Algorithm**: A* waypoint navigation
- **ICR Kinematics**: 4-wheel steering calculations
- **Actuator Control**: Simulated servo and stepper motor control
- **Mission Execution**: Complete autonomous navigation from start to goal

## 📁 Files

- `diagram.json` - Wokwi circuit diagram (ESP32 + LEDs)
- `project.json` - Wokwi project configuration
- `main_wokwi.cpp` - Simulation code (in src/ folder)

## 🔧 Hardware Simulation

Since Wokwi doesn't support real motors/sensors, we simulate:

### Servo Motors (Steering)
- **Red LED (GPIO 12)**: Front Left steering angle
- **Green LED (GPIO 13)**: Front Right steering angle
- **Blue LED (GPIO 14)**: Rear Left steering angle
- **Yellow LED (GPIO 15)**: Rear Right steering angle
- **Brightness** = steering angle intensity

### Stepper Motors (Propulsion)
- **Red LED (GPIO 16)**: Front Left wheel speed
- **Green LED (GPIO 17)**: Front Right wheel speed
- **Blue LED (GPIO 18)**: Middle Left wheel speed
- **Yellow LED (GPIO 19)**: Middle Right wheel speed
- **Purple LED (GPIO 21)**: Rear Left wheel speed
- **Orange LED (GPIO 22)**: Rear Right wheel speed
- **Blinking rate** = motor speed (faster = more power)

## 🚀 How to Run

1. **Open Wokwi**: Go to [wokwi.com](https://wokwi.com)
2. **Import Project**: Upload the `wokwi/` folder contents
3. **Upload Code**: Use `src/main_wokwi.cpp` as main file
4. **Start Simulation**

## 📡 Serial Commands

Open the Serial Monitor and type:

- `start` - Begin the mission
- `stop` - Stop the mission
- `status` - Show current status
- `help` - Show available commands

## 📊 What You'll See

### Serial Output
```
POS: [1.23, 4.56] | θ:45.2° | V:0.30 W:0.15 | WP:3/8 | Time:12.5s
SERVO ANGLES -> FL:15.2°, FR:-8.7°, RL:12.1°, RR:-5.3°
STEPPER SPEEDS -> FL:245Hz, FR:238Hz, ML:250Hz, MR:252Hz, RL:243Hz, RR:240Hz
```

### LED Behavior
- **Servo LEDs**: Brightness shows steering angle
- **Motor LEDs**: Blinking shows wheel speed and direction
- **Mission Progress**: Watch position approach waypoints

## 🎯 Mission Configuration

The simulation uses the mission data from `station_sol/mission_export.h`:

- **Start**: (START_X, START_Y) @ START_THETA
- **Goal**: (GOAL_X, GOAL_Y)
- **Waypoints**: PATH_SIZE points in MISSION_PATH array

## 🔍 Testing Scenarios

### 1. Straight Line Navigation
- Mission with waypoints in straight line
- Expect: All steering angles ≈ 0°, constant motor speeds

### 2. Curved Path Following
- Mission with curved waypoints
- Expect: Varying steering angles, coordinated motor speeds

### 3. Point Turn Test
- Very tight turn waypoints
- Expect: Opposite steering angles, differential motor speeds

### 4. Complete Mission
- Full start-to-goal navigation
- Expect: Smooth transitions between waypoints

## 🐛 Debugging

### Common Issues

1. **No LED activity**: Check GPIO pin assignments
2. **Erratic behavior**: Verify mission_export.h data
3. **No serial output**: Check baud rate (115200)

### Expected Behavior

- **Mission start**: LEDs begin responding to path calculations
- **Waypoint approach**: Steering adjusts, speed may reduce
- **Mission complete**: All LEDs stabilize, "MISSION COMPLETE" message

## 📈 Performance Metrics

Monitor these in the serial output:

- **Position accuracy**: How well robot follows waypoints
- **Speed control**: V (linear) and W (angular) velocity
- **Kinematics**: Steering angles and motor coordination
- **Timing**: Mission completion time

## 🔄 Real Hardware Translation

This simulation directly translates to real hardware:

- **LED brightness** → **Servo PWM pulses**
- **LED blinking** → **Stepper step pulses**
- **Serial commands** → **NRF24L01 messages**
- **Position output** → **Telemetry data**

The exact same algorithms run on both simulation and real robot!