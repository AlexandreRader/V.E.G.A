/*
 * ==========================================
 * VEGA SC317 - WOKWI SIMULATION TEST
 * ==========================================
 *
 * This version is adapted for Wokwi simulation:
 * - No real hardware (NRF24L01, ToF sensors, etc.)
 * - Simulated actuators using LEDs and serial output
 * - Mission data from mission_export.h
 * - Focus on kinematics and path following logic
 */

#include <Arduino.h>
#ifdef WOKWI_SIMULATION
#include "../wokwi/mission_export.h" // Test mission for simulation
#else
#include "mission_export.h" // Real mission from ground station
#endif
#include "../lib/Pathfinding/PathFollower.h"
#include "../lib/Pathfinding/Kinematics.h"


// ==========================================
// SIMULATED ACTUATORS (Wokwi-compatible)
// ==========================================

class SimulatedActuators {
private:
    // Status LED indicator (GPIO 2) - blinks to show firmware is running
    const uint8_t LED_STATUS = 2;
    
    // LED pins for simulation (instead of real servos/steppers)
    const uint8_t LED_SERVO_FL = 12;
    const uint8_t LED_SERVO_FR = 13;
    const uint8_t LED_SERVO_RL = 14;
    const uint8_t LED_SERVO_RR = 15;

    const uint8_t LED_MOTOR_FL = 16;
    const uint8_t LED_MOTOR_FR = 17;
    const uint8_t LED_MOTOR_ML = 18;
    const uint8_t LED_MOTOR_MR = 19;
    const uint8_t LED_MOTOR_RL = 21;
    const uint8_t LED_MOTOR_RR = 22;

public:
    void begin() {
        // Initialize status LED
        pinMode(LED_STATUS, OUTPUT);
        digitalWrite(LED_STATUS, LOW);
        
        // Initialize LED pins
        pinMode(LED_SERVO_FL, OUTPUT);
        pinMode(LED_SERVO_FR, OUTPUT);
        pinMode(LED_SERVO_RL, OUTPUT);
        pinMode(LED_SERVO_RR, OUTPUT);

        pinMode(LED_MOTOR_FL, OUTPUT);
        pinMode(LED_MOTOR_FR, OUTPUT);
        pinMode(LED_MOTOR_ML, OUTPUT);
        pinMode(LED_MOTOR_MR, OUTPUT);
        pinMode(LED_MOTOR_RL, OUTPUT);
        pinMode(LED_MOTOR_RR, OUTPUT);

        Serial.println("Simulated actuators initialized");
    }

    // Blink status LED to show firmware is running
    void blinkStatus() {
        static unsigned long last_blink = 0;
        static bool status_state = false;
        
        if (millis() - last_blink > 500) {
            status_state = !status_state;
            digitalWrite(LED_STATUS, status_state ? HIGH : LOW);
            last_blink = millis();
        }
    }

    // Simulate servo control with LED brightness
    void setServoAngles(float fl, float fr, float rl, float rr) {
        // Convert angles to LED brightness (0-255)
        int bright_fl = map(abs(fl) * 180/PI, 0, 45, 0, 255);
        int bright_fr = map(abs(fr) * 180/PI, 0, 45, 0, 255);
        int bright_rl = map(abs(rl) * 180/PI, 0, 45, 0, 255);
        int bright_rr = map(abs(rr) * 180/PI, 0, 45, 0, 255);

        analogWrite(LED_SERVO_FL, bright_fl);
        analogWrite(LED_SERVO_FR, bright_fr);
        analogWrite(LED_SERVO_RL, bright_rl);
        analogWrite(LED_SERVO_RR, bright_rr);

        Serial.printf("SERVO ANGLES -> FL:%.1f°, FR:%.1f°, RL:%.1f°, RR:%.1f°\n",
                      fl * 180/PI, fr * 180/PI, rl * 180/PI, rr * 180/PI);
    }

    // Simulate stepper control with LED blinking
    void setStepperSpeeds(float fl, float fr, float ml, float mr, float rl, float rr) {
        // Convert Hz to blink delay (higher speed = faster blink)
        int delay_fl = fl > 0 ? max(50, 1000/(int)fl) : 1000;
        int delay_fr = fr > 0 ? max(50, 1000/(int)fr) : 1000;
        int delay_ml = ml > 0 ? max(50, 1000/(int)ml) : 1000;
        int delay_mr = mr > 0 ? max(50, 1000/(int)mr) : 1000;
        int delay_rl = rl > 0 ? max(50, 1000/(int)rl) : 1000;
        int delay_rr = rr > 0 ? max(50, 1000/(int)rr) : 1000;

        // Simple LED state toggle for simulation
        static bool state = false;
        static unsigned long last_toggle = 0;

        if (millis() - last_toggle > 200) {
            state = !state;
            digitalWrite(LED_MOTOR_FL, state && fl > 0);
            digitalWrite(LED_MOTOR_FR, state && fr > 0);
            digitalWrite(LED_MOTOR_ML, state && ml > 0);
            digitalWrite(LED_MOTOR_MR, state && mr > 0);
            digitalWrite(LED_MOTOR_RL, state && rl > 0);
            digitalWrite(LED_MOTOR_RR, state && rr > 0);
            last_toggle = millis();
        }

        Serial.printf("STEPPER SPEEDS -> FL:%.0fHz, FR:%.0fHz, ML:%.0fHz, MR:%.0fHz, RL:%.0fHz, RR:%.0fHz\n",
                      fl, fr, ml, mr, rl, rr);
    }
};

// ==========================================
// SIMULATED IMU (for testing)
// ==========================================

class SimulatedIMU {
private:
    float simulated_heading;
    unsigned long last_update;

public:
    bool begin() {
        simulated_heading = START_THETA;
        last_update = millis();
        Serial.println("Simulated IMU initialized");
        return true;
    }

    float getCurrentTheta() {
        // Simple heading simulation with small drift
        unsigned long now = millis();
        float dt = (now - last_update) / 1000.0;
        last_update = now;

        // Add small random drift to simulate real IMU
        simulated_heading += (random(-10, 10) / 1000.0) * dt;

        return simulated_heading;
    }
};

// ==========================================
// MAIN TEST PROGRAM
// ==========================================

PathFollower follower;
Kinematics kinematics;
SimulatedActuators actuators;
SimulatedIMU imu;

// Robot state
float robot_x = START_X;
float robot_y = START_Y;
float robot_theta = START_THETA;
float current_v = 0.0;
float current_w = 0.0;

// Timing
unsigned long last_update = 0;
unsigned long mission_start_time = 0;
bool mission_started = false;

// Forward declarations
void startMission();
void stopMission();
void runSimulationStep();
void printStatus();

void setup() {
    Serial.begin(115200);
    
    delay(1000);

    Serial.println("\n🚀 VEGA SC317 - WOKWI SIMULATION TEST");
    Serial.println("=====================================");
    Serial.printf("Mission: %.2f, %.2f -> %.2f, %.2f\n", START_X, START_Y, GOAL_X, GOAL_Y);
    Serial.printf("Path points: %d\n", PATH_SIZE);

    // Initialize simulated components
    actuators.begin();
    if (imu.begin()) {
        Serial.println("✅ Simulated IMU: OK");
    }

    // Display mission waypoints
    Serial.println("\nWAYPOINTS:");
    for (int i = 0; i < PATH_SIZE; i++) {
        Serial.printf("  %d: (%.2f, %.2f)\n", i, MISSION_PATH[i].x, MISSION_PATH[i].y);
    }

    Serial.println("\n=== READY - Type 'start' to begin simulation ===");
}

void loop() {
    // Status LED indicator (blinking confirms code is running)
    actuators.blinkStatus();
    
    // Check for serial commands
    if (Serial.available()) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();
        cmd.toLowerCase();

        if (cmd == "start") {
            startMission();
        } else if (cmd == "stop") {
            stopMission();
        } else if (cmd == "status") {
            printStatus();
        } else if (cmd == "help") {
            Serial.println("Commands: start, stop, status, help");
        }
    }

    // Run simulation if mission is active
    if (mission_started && !follower.isDone()) {
        runSimulationStep();
    } else if (mission_started && follower.isDone()) {
        Serial.println("\n🎯 MISSION COMPLETE!");
        mission_started = false;
    }

    delay(100); // 10Hz simulation
}

void startMission() {
    if (!mission_started) {
        follower.resetMission();
        mission_started = true;
        mission_start_time = millis();
        robot_x = START_X;
        robot_y = START_Y;
        robot_theta = START_THETA;

        Serial.println("\n▶️  MISSION STARTED");
        Serial.printf("Starting position: (%.2f, %.2f) @ %.1f°\n",
                      robot_x, robot_y, robot_theta * 180/PI);
    }
}

void stopMission() {
    mission_started = false;
    current_v = 0;
    current_w = 0;
    actuators.setServoAngles(0, 0, 0, 0);
    actuators.setStepperSpeeds(0, 0, 0, 0, 0, 0);
    Serial.println("\n⏹️  MISSION STOPPED");
}

void runSimulationStep() {
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0;
    last_update = now;

    // Update robot position (simulated IMU)
    robot_theta = imu.getCurrentTheta();

    // Simulate odometry (perfect for testing)
    robot_x += current_v * cos(robot_theta) * dt;
    robot_y += current_v * sin(robot_theta) * dt;

    // Path following
    VelocityCommand cmd = follower.update(robot_x, robot_y, robot_theta);
    current_v = cmd.linear_v;
    current_w = cmd.angular_w;

    // Kinematics calculation
    MotorCommands motor_cmds = kinematics.calculateDrive(current_v, current_w);

    // Actuator control (simulated)
    actuators.setServoAngles(
        motor_cmds.angle_FL, motor_cmds.angle_FR,
        motor_cmds.angle_RL, motor_cmds.angle_RR
    );

    // Convert speeds to Hz
    float hz_fl = kinematics.speedToStepsHz(motor_cmds.speed_FL);
    float hz_fr = kinematics.speedToStepsHz(motor_cmds.speed_FR);
    float hz_ml = kinematics.speedToStepsHz(motor_cmds.speed_ML);
    float hz_mr = kinematics.speedToStepsHz(motor_cmds.speed_MR);
    float hz_rl = kinematics.speedToStepsHz(motor_cmds.speed_RL);
    float hz_rr = kinematics.speedToStepsHz(motor_cmds.speed_RR);

    actuators.setStepperSpeeds(hz_fl, hz_fr, hz_ml, hz_mr, hz_rl, hz_rr);

    // Status output (every 500ms)
    static unsigned long last_status = 0;
    if (now - last_status > 500) {
        int current_waypoint = follower.isDone() ? PATH_SIZE - 1 : 0;
        for (int i = 0; i < PATH_SIZE - 1; i++) {
            float dx = MISSION_PATH[i+1].x - MISSION_PATH[i].x;
            float dy = MISSION_PATH[i+1].y - MISSION_PATH[i].y;
            float dist_to_start = sqrt(pow(robot_x - MISSION_PATH[i].x, 2) + pow(robot_y - MISSION_PATH[i].y, 2));
            float segment_length = sqrt(dx*dx + dy*dy);
            if (dist_to_start < segment_length) {
                current_waypoint = i;
                break;
            }
        }

        Serial.printf("POS: [%.2f, %.2f] | θ:%.1f° | V:%.2f W:%.2f | WP:%d/%d | Time:%.1fs\n",
                      robot_x, robot_y, robot_theta * 180/PI,
                      current_v, current_w, current_waypoint + 1, PATH_SIZE,
                      (now - mission_start_time) / 1000.0);
        last_status = now;
    }
}

void printStatus() {
    Serial.println("\n=== SIMULATION STATUS ===");
    Serial.printf("Mission active: %s\n", mission_started ? "YES" : "NO");
    Serial.printf("Position: (%.2f, %.2f) @ %.1f°\n", robot_x, robot_y, robot_theta * 180/PI);
    Serial.printf("Velocity: V=%.2f m/s, W=%.2f rad/s\n", current_v, current_w);
    Serial.printf("Mission progress: %s\n", follower.isDone() ? "COMPLETE" : "IN PROGRESS");
    Serial.printf("Elapsed time: %.1f seconds\n", (millis() - mission_start_time) / 1000.0);
    Serial.println("========================");
}