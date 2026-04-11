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
#include <ESP32Servo.h>
#ifdef WOKWI_SIMULATION
#include "/home/wankeur/Documents/Code/Github/V.E.G.A/mission_export.h" // Test mission for simulation
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
    
    // Servo motors
    Servo servoFL, servoFR, servoRL, servoRR;
    
    // Stepper motor pins (STEP, DIR) - matching pins.h
    const uint8_t STEPPER_FL_STEP = 15;  // M1 Front Left
    const uint8_t STEPPER_FL_DIR = 16;
    const uint8_t STEPPER_FR_STEP = 17;  // M4 Front Right  
    const uint8_t STEPPER_FR_DIR = 18;
    const uint8_t STEPPER_ML_STEP = 42;  // M2 Middle Left
    const uint8_t STEPPER_ML_DIR = 41;
    const uint8_t STEPPER_MR_STEP = 40;  // M5 Middle Right
    const uint8_t STEPPER_MR_DIR = 39;
    const uint8_t STEPPER_RL_STEP = 38;  // M3 Rear Left
    const uint8_t STEPPER_RL_DIR = 37;
    const uint8_t STEPPER_RR_STEP = 36;  // M6 Rear Right
    const uint8_t STEPPER_RR_DIR = 35;

public:
    void begin() {
        // Initialize status LED
        pinMode(LED_STATUS, OUTPUT);
        digitalWrite(LED_STATUS, LOW);
        
        // Attach servo motors
        servoFL.attach(12);
        servoFR.attach(13);
        servoRL.attach(14);
        servoRR.attach(11);
        
        // Initialize stepper motor pins
        pinMode(STEPPER_FL_STEP, OUTPUT);
        pinMode(STEPPER_FL_DIR, OUTPUT);
        pinMode(STEPPER_FR_STEP, OUTPUT);
        pinMode(STEPPER_FR_DIR, OUTPUT);
        pinMode(STEPPER_ML_STEP, OUTPUT);
        pinMode(STEPPER_ML_DIR, OUTPUT);
        pinMode(STEPPER_MR_STEP, OUTPUT);
        pinMode(STEPPER_MR_DIR, OUTPUT);
        pinMode(STEPPER_RL_STEP, OUTPUT);
        pinMode(STEPPER_RL_DIR, OUTPUT);
        pinMode(STEPPER_RR_STEP, OUTPUT);
        pinMode(STEPPER_RR_DIR, OUTPUT);

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

    // Control servo motors
    void setServoAngles(float fl, float fr, float rl, float rr) {
        // Convert radians to degrees and center at 90° (neutral position)
        int angle_fl = 90 + (fl * 180/PI);
        int angle_fr = 90 + (fr * 180/PI);
        int angle_rl = 90 + (rl * 180/PI);
        int angle_rr = 90 + (rr * 180/PI);
        
        // Constrain to servo range (0-180°)
        angle_fl = constrain(angle_fl, 0, 180);
        angle_fr = constrain(angle_fr, 0, 180);
        angle_rl = constrain(angle_rl, 0, 180);
        angle_rr = constrain(angle_rr, 0, 180);
        
        servoFL.write(angle_fl);
        servoFR.write(angle_fr);
        servoRL.write(angle_rl);
        servoRR.write(angle_rr);

        Serial.print("SERVO ANGLES FL:");
        Serial.print(fl * 180/PI, 1);
        Serial.print("° FR:");
        Serial.print(fr * 180/PI, 1);
        Serial.print("° RL:");
        Serial.print(rl * 180/PI, 1);
        Serial.print("° RR:");
        Serial.print(rr * 180/PI, 1);
        Serial.println("°");
    }

    // Control stepper motors with A4988 drivers
    void setStepperSpeeds(float fl, float fr, float ml, float mr, float rl, float rr) {
        // Set directions (positive speed = forward)
        digitalWrite(STEPPER_FL_DIR, fl >= 0 ? HIGH : LOW);
        digitalWrite(STEPPER_FR_DIR, fr >= 0 ? HIGH : LOW);
        digitalWrite(STEPPER_ML_DIR, ml >= 0 ? HIGH : LOW);
        digitalWrite(STEPPER_MR_DIR, mr >= 0 ? HIGH : LOW);
        digitalWrite(STEPPER_RL_DIR, rl >= 0 ? HIGH : LOW);
        digitalWrite(STEPPER_RR_DIR, rr >= 0 ? HIGH : LOW);
        
        // Generate step pulses based on frequency
        generateStepPulse(STEPPER_FL_STEP, abs(fl));
        generateStepPulse(STEPPER_FR_STEP, abs(fr));
        generateStepPulse(STEPPER_ML_STEP, abs(ml));
        generateStepPulse(STEPPER_MR_STEP, abs(mr));
        generateStepPulse(STEPPER_RL_STEP, abs(rl));
        generateStepPulse(STEPPER_RR_STEP, abs(rr));

        Serial.printf("STEPPER SPEEDS -> FL:%.0fHz, FR:%.0fHz, ML:%.0fHz, MR:%.0fHz, RL:%.0fHz, RR:%.0fHz\n",
                      fl, fr, ml, mr, rl, rr);
    }
    
private:
    // Generate step pulses for A4988 stepper driver
    void generateStepPulse(uint8_t stepPin, float frequency) {
        if (frequency <= 0) return;
        
        static unsigned long lastSteps[6] = {0, 0, 0, 0, 0, 0};
        static int stepIndex = 0;
        
        // Map pins to indices
        int idx = -1;
        if (stepPin == STEPPER_FL_STEP) idx = 0;
        else if (stepPin == STEPPER_FR_STEP) idx = 1;
        else if (stepPin == STEPPER_ML_STEP) idx = 2;
        else if (stepPin == STEPPER_MR_STEP) idx = 3;
        else if (stepPin == STEPPER_RL_STEP) idx = 4;
        else if (stepPin == STEPPER_RR_STEP) idx = 5;
        
        if (idx == -1) return;
        
        unsigned long period_us = (unsigned long)(1000000.0 / frequency);
        if (micros() - lastSteps[idx] >= period_us) {
            digitalWrite(stepPin, HIGH);
            delayMicroseconds(1); // Very short pulse
            digitalWrite(stepPin, LOW);
            lastSteps[idx] = micros();
        }
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

PathFollower* follower = nullptr;
Kinematics* kinematics = nullptr;
//EKFManager *ekf = nullptr; // Si tu l'utilises
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

    Serial.println("\n--- TEST CONNEXION VEGA ---");
    Serial.println("Si tu vois ce message, le pont TCP fonctionne !");
    
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 3000); 

    Serial.println("\n🚀 VEGA SC317 - INITIALISATION...");

    // Initialisation dynamique (sur le Tas/Heap)
    follower = new PathFollower();
    kinematics = new Kinematics();
    

    actuators.begin();
    imu.begin();

    Serial.println("✅ Système prêt. Tapez 'start' pour lancer la mission.");
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

    // AJOUT D'UNE SÉCURITÉ : On vérifie que les objets existent bien
    if (mission_started && follower != nullptr && kinematics != nullptr) {
        if (!follower->isDone()) {
            runSimulationStep();
        } else {
            Serial.println("🎯 MISSION COMPLETE!");
            mission_started = false;
        }
    }
    
    delay(100); // 10Hz simulation
}

void startMission() {
    if (!mission_started) {
        follower->resetMission();
        mission_started = true;
        mission_start_time = millis();
        last_update = millis(); // Initialize timing to avoid huge dt on first step
        robot_x = START_X;
        robot_y = START_Y;
        robot_theta = START_THETA;

        Serial.println("MISSION STARTED");
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
    Serial.println("MISSION STOPPED");
}

void runSimulationStep() {
    unsigned long now = millis();
    float dt = (now - last_update) / 1000.0;
    last_update = now;

    // Path following FIRST to get velocity commands
    VelocityCommand cmd = follower->update(robot_x, robot_y, robot_theta);
    current_v = cmd.linear_v;
    current_w = cmd.angular_w;

    // Update robot orientation by integrating angular velocity
    robot_theta += current_w * dt;

    // Simulate odometry (perfect for testing)
    robot_x += current_v * cos(robot_theta) * dt;
    robot_y += current_v * sin(robot_theta) * dt;

    // Kinematics calculation
    MotorCommands motor_cmds = kinematics->calculateDrive(current_v, current_w);

    // Actuator control (simulated)
    actuators.setServoAngles(
        motor_cmds.angle_FL, motor_cmds.angle_FR,
        motor_cmds.angle_RL, motor_cmds.angle_RR
    );

    // Convert speeds to Hz
    float hz_fr = kinematics->speedToStepsHz(motor_cmds.speed_FR);
    float hz_ml = kinematics->speedToStepsHz(motor_cmds.speed_ML);
    float hz_mr = kinematics->speedToStepsHz(motor_cmds.speed_MR);
    float hz_rl = kinematics->speedToStepsHz(motor_cmds.speed_RL);
    float hz_fl = kinematics->speedToStepsHz(motor_cmds.speed_FL);
    float hz_rr = kinematics->speedToStepsHz(motor_cmds.speed_RR);

    actuators.setStepperSpeeds(hz_fl, hz_fr, hz_ml, hz_mr, hz_rl, hz_rr);

    // Status output (every 500ms)
    static unsigned long last_status = 0;
    if (now - last_status > 500) {
        Serial.print("POS [X:");
        Serial.print(robot_x, 2);
        Serial.print(" Y:");
        Serial.print(robot_y, 2);
        Serial.print("] THETA:");
        Serial.print(robot_theta * 180/PI, 1);
        Serial.print("° V:");
        Serial.print(current_v, 2);
        Serial.print(" W:");
        Serial.print(current_w, 4);  // More precision for angular velocity
        Serial.print(" WP:");
        Serial.print(follower->getCurrentIndex() + 1);
        Serial.print("/");
        Serial.print(PATH_SIZE);
        Serial.print(" TIME:");
        Serial.print((now - mission_start_time) / 1000.0, 1);
        Serial.println("s");
        last_status = now;
    }
}

void printStatus() {
    Serial.println("\n=== SIMULATION STATUS ===");
    Serial.printf("Mission active: %s\n", mission_started ? "YES" : "NO");
    Serial.printf("Position: (%.2f, %.2f) @ %.1f°\n", robot_x, robot_y, robot_theta * 180/PI);
    Serial.printf("Velocity: V=%.2f m/s, W=%.2f rad/s\n", current_v, current_w);
    Serial.printf("Mission progress: %s\n", follower->isDone() ? "COMPLETE" : "IN PROGRESS");
    Serial.printf("Elapsed time: %.1f seconds\n", (millis() - mission_start_time) / 1000.0);
    Serial.println("========================");
}


