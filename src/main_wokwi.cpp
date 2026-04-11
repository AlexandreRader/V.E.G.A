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
    const uint8_t LED_STATUS = 2; // OK
    
    // On évite 18, 19 et 20 à tout prix
    const uint8_t LED_SERVO_FL = 12;
    const uint8_t LED_SERVO_FR = 13;
    const uint8_t LED_SERVO_RL = 14;
    const uint8_t LED_SERVO_RR = 15;

    const uint8_t LED_MOTOR_FL = 16;
    const uint8_t LED_MOTOR_FR = 17;
    
    // NOUVELLES BROCHES SÉCURISÉES POUR LE S3
    const uint8_t LED_MOTOR_ML = 1; 
    const uint8_t LED_MOTOR_MR = 3;
    const uint8_t LED_MOTOR_RL = 21;
    const uint8_t LED_MOTOR_RR = 42;

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

        Serial.print("STEPPER SPEEDS FL:");
        Serial.print(fl, 0);
        Serial.print("Hz FR:");
        Serial.print(fr, 0);
        Serial.print("Hz ML:");
        Serial.print(ml, 0);
        Serial.print("Hz MR:");
        Serial.print(mr, 0);
        Serial.print("Hz RL:");
        Serial.print(rl, 0);
        Serial.print("Hz RR:");
        Serial.print(rr, 0);
        Serial.println("Hz");
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
        int current_waypoint = follower->isDone() ? PATH_SIZE - 1 : 0;
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

        Serial.print("POS [X:");
        Serial.print(robot_x, 2);
        Serial.print(" Y:");
        Serial.print(robot_y, 2);
        Serial.print("] THETA:");
        Serial.print(robot_theta * 180/PI, 1);
        Serial.print("° V:");
        Serial.print(current_v, 2);
        Serial.print(" W:");
        Serial.print(current_w, 2);
        Serial.print(" WP:");
        Serial.print(current_waypoint + 1);
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



/*

#include <Arduino.h>

// On teste uniquement la LED blanche sur le GPIO 2
const uint8_t LED_TEST = 2;

void setup() {
    // Initialisation du Serial (indispensable pour l'USB du S3)
    Serial.begin(115200);
    
    // Attente pour que le port série se réveille sur le PC/Wokwi
    uint32_t start = millis();
    while (!Serial && (millis() - start) < 3000); 

    Serial.println("\n*********************************");
    Serial.println("VEGA SC317 - TEST MINIMAL LED");
    Serial.println("*********************************");

    pinMode(LED_TEST, OUTPUT);
    Serial.println("GPIO 2 configuré en SORTIE");
}

void loop() {
    Serial.println("LED ON...");
    digitalWrite(LED_TEST, HIGH);
    delay(500);

    Serial.println("LED OFF");
    digitalWrite(LED_TEST, LOW);
    delay(500);
}
*/