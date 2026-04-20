#pragma once
#include <Arduino.h>
#include "../Pathfinding/PathFollower.h"
#include "../Pathfinding/Kinematics.h"
#include "../EKF/EKFManager.h"
#include "../Control/HardwareControl.h"
#include "../Sensors/Detection.h"
#include "../Sensors/IRSensors.h"
#include "../Communication/NRF.h"

// ==========================================
// CONTRÔLEUR PRINCIPAL DE NAVIGATION - VEGA SC317
// ==========================================

enum RobotState {
    STATE_IDLE,
    STATE_NAVIGATING,
    STATE_OBSTACLE_AVOIDANCE,
    STATE_DOCKING,
    STATE_EMERGENCY_STOP
};

class NavigationController {
private:
    // Composants du système
    PathFollower path_follower;
    Kinematics kinematics;
    EKFManager ekf;
    ActuatorManager actuators;
    ToFManager tof_sensors;
    IRDockingSensor ir_sensor;
    

    // État du robot
    RobotState current_state;
    float robot_x, robot_y, robot_theta;
    float current_v, current_w;

    // Paramètres de navigation
    const float OBSTACLE_SLOWDOWN_DISTANCE = 0.8;  // 80cm
    const float OBSTACLE_STOP_DISTANCE = 0.3;      // 30cm
    const float DOCKING_DISTANCE_THRESHOLD = 0.5;  // 50cm du goal

    // Timers et compteurs
    unsigned long last_update;
    unsigned long last_telemetry;

public:
    NavigationController()
        : ir_sensor(PIN_IR_OUT), current_state(STATE_IDLE),
          robot_x(0), robot_y(0), robot_theta(0),
          current_v(0), current_w(0), last_update(0), last_telemetry(0) {}

    // Initialisation complète du système
    bool initialize() {
        Serial.println("=== INITIALISATION SYSTÈME VEGA SC317 ===");

        bool success = true;

        

        // 2. Capteurs ToF
        if (!tof_sensors.begin()) {
            Serial.println("❌ Capteurs ToF: ÉCHEC");
            success = false;
        } else {
            Serial.println("✅ Capteurs ToF: OK");
        }

        // 3. Capteur IR
        ir_sensor.begin();
        Serial.println("✅ Capteur IR: OK");

        // 4. Actionneurs
        if (!actuators.begin()) {
            Serial.println("❌ Actionneurs: ÉCHEC");
            success = false;
        } else {
            Serial.println("✅ Actionneurs: OK");
        }

        // 5. Position initiale (sera mise à jour par NRF)
        setInitialPosition(START_X, START_Y, START_THETA);

        if (success) {
            Serial.println("=== SYSTÈME INITIALISÉ AVEC SUCCÈS ===");
            current_state = STATE_IDLE;
        } else {
            Serial.println("=== ERREURS D'INITIALISATION DÉTECTÉES ===");
        }

        return success;
    }

    // Définition de la position initiale
    void setInitialPosition(float x, float y, float theta) {
        robot_x = x;
        robot_y = y;
        robot_theta = theta;

        // Reset EKF
        ekf.X(0) = x;
        ekf.X(1) = y;
        ekf.X(2) = theta;

        Serial.printf("Position initiale définie: X=%.2f Y=%.2f Theta=%.2f\n", x, y, theta);
    }

    // Boucle principale de navigation
    void update() {
        unsigned long now = millis();
        float dt = (now - last_update) / 1000.0;
        last_update = now;

        // 1. Mise à jour des capteurs
        updateSensors();

        

        // 3. Mise à jour de l'état du robot
        updateRobotState(dt);

        // 4. Calcul des commandes de mouvement
        computeMotionCommands();

        // 5. Application des commandes aux actionneurs
        applyActuatorCommands();

      
    }

    // Mise à jour des capteurs
    void updateSensors() {
        tof_sensors.update();
        ir_sensor.isTargetDetected();  // Mise à jour automatique
    }

  

    // Mise à jour de l'état du robot
    void updateRobotState(float dt) {
        // 1. Vérification des arrêts d'urgence
        if (tof_sensors.emergencyStopRequired()) {
            current_state = STATE_EMERGENCY_STOP;
            current_v = 0;
            current_w = 0;
            actuators.enableMotors(false);
            Serial.println("🚨 ARRÊT D'URGENCE - OBSTACLE CRITIQUE");
            return;
        }

        // 2. Logique d'état
        switch (current_state) {
            case STATE_IDLE:
                // Attente de commande de démarrage
                break;

            case STATE_NAVIGATING:
                // Navigation normale avec suivi de trajectoire
                if (path_follower.isDone()) {
                    // Vérification si on doit passer en mode docking
                    float dist_to_goal = sqrt(pow(robot_x - GOAL_X, 2) + pow(robot_y - GOAL_Y, 2));
                    if (dist_to_goal < DOCKING_DISTANCE_THRESHOLD) {
                        current_state = STATE_DOCKING;
                        Serial.println("🎯 Passage en mode DOCKING");
                    } else {
                        current_state = STATE_IDLE;
                        Serial.println("✅ Mission terminée");
                    }
                }

                // Vérification d'obstacles
                if (tof_sensors.hasObstacleFront()) {
                    current_state = STATE_OBSTACLE_AVOIDANCE;
                    Serial.println("⚠️  Obstacle détecté - Évitement");
                }
                break;

            case STATE_OBSTACLE_AVOIDANCE:
                // Logique d'évitement d'obstacle
                performObstacleAvoidance();
                break;

            case STATE_DOCKING:
                // Navigation de précision avec capteur IR
                performPrecisionDocking();
                break;

            case STATE_EMERGENCY_STOP:
                // Attente de reset manuel
                break;
        }

        // 3. Estimation de position avec EKF (simulation simple pour l'instant)
        // Dans la version complète, utiliser IMU et odométrie
        robot_x += current_v * cos(robot_theta) * dt;
        robot_y += current_v * sin(robot_theta) * dt;
        robot_theta += current_w * dt;
    }

    // Calcul des commandes de mouvement
    void computeMotionCommands() {
        switch (current_state) {
            case STATE_NAVIGATING: {
                // Utilisation du PathFollower
                VelocityCommand cmd = path_follower.update(robot_x, robot_y, robot_theta);

                // Adaptation selon obstacles
                float front_dist = tof_sensors.getMinFrontDistance();
                if (front_dist < OBSTACLE_SLOWDOWN_DISTANCE) {
                    cmd.linear_v *= 0.5;  // Ralentissement
                }
                if (front_dist < OBSTACLE_STOP_DISTANCE) {
                    cmd.linear_v = 0;  // Arrêt
                }

                current_v = cmd.linear_v;
                current_w = cmd.angular_w;
                break;
            }

            case STATE_OBSTACLE_AVOIDANCE:
                // Logique d'évitement (temporaire)
                current_v = 0.1;
                current_w = 0.5;  // Rotation pour éviter
                break;

            case STATE_DOCKING:
                // Utilisation du capteur IR pour asservissement fin
                performPrecisionDocking();
                break;

            default:
                current_v = 0;
                current_w = 0;
                break;
        }
    }

    // Application aux actionneurs
    void applyActuatorCommands() {
        if (current_state == STATE_EMERGENCY_STOP) {
            actuators.enableMotors(false);
            return;
        }

        // Calcul de la cinématique
        MotorCommands motor_cmds = kinematics.calculateDrive(current_v, current_w);

        // Application aux servos et moteurs
        actuators.setServoAngles(
            motor_cmds.angle_FL, motor_cmds.angle_FR,
            motor_cmds.angle_RL, motor_cmds.angle_RR
        );

        // Conversion vitesses -> fréquences
        float hz_fl = kinematics.speedToStepsHz(motor_cmds.speed_FL);
        float hz_fr = kinematics.speedToStepsHz(motor_cmds.speed_FR);
        float hz_ml = kinematics.speedToStepsHz(motor_cmds.speed_ML);
        float hz_mr = kinematics.speedToStepsHz(motor_cmds.speed_MR);
        float hz_rl = kinematics.speedToStepsHz(motor_cmds.speed_RL);
        float hz_rr = kinematics.speedToStepsHz(motor_cmds.speed_RR);

        actuators.setStepperSpeeds(hz_fl, hz_fr, hz_ml, hz_mr, hz_rl, hz_rr);
        actuators.enableMotors(true);
    }

    // Démarrage de mission
    void startMission() {
        if (current_state == STATE_IDLE) {
            path_follower.resetMission();
            current_state = STATE_NAVIGATING;
            Serial.println("🚀 Mission démarrée");
        }
    }

    // Arrêt de mission
    void stopMission() {
        current_state = STATE_IDLE;
        current_v = 0;
        current_w = 0;
        actuators.enableMotors(false);
        Serial.println("🛑 Mission arrêtée");
    }

    // Reset système
    void resetSystem() {
        stopMission();
        setInitialPosition(START_X, START_Y, START_THETA);
        Serial.println("🔄 Système remis à zéro");
    }

    // Évitement d'obstacle (implémentation basique)
    void performObstacleAvoidance() {
        // Logique simple: tourner jusqu'à dégagement
        if (!tof_sensors.hasObstacleFront()) {
            current_state = STATE_NAVIGATING;
            Serial.println("✅ Obstacle évité - Reprise navigation");
        } else {
            current_v = 0;
            current_w = 0.3;  // Rotation lente
        }
    }

    // Docking de précision
    void performPrecisionDocking() {
        if (ir_sensor.isDocked()) {
            current_state = STATE_IDLE;
            current_v = 0;
            current_w = 0;
            Serial.println("🎯 DOCKING RÉUSSI - CIBLE ATTEINTE");
            return;
        }

        // Asservissement simple vers la cible IR
        float error = ir_sensor.getPositionError();
        if (error > 10) {  // Trop loin
            current_v = 0.05;
            current_w = 0;
        } else if (error > 0) {  // Un peu loin
            current_v = 0.02;
            current_w = 0;
        } else {  // Trop près
            current_v = -0.01;
            current_w = 0;
        }
    }



    // Getters pour diagnostic
    RobotState getCurrentState() const { return current_state; }
    float getRobotX() const { return robot_x; }
    float getRobotY() const { return robot_y; }
    float getRobotTheta() const { return robot_theta; }

    // Diagnostic complet
    void printSystemStatus() {
        Serial.println("\n=== ÉTAT SYSTÈME VEGA SC317 ===");
        Serial.printf("État: ");
        switch (current_state) {
            case STATE_IDLE: Serial.println("INACTIF"); break;
            case STATE_NAVIGATING: Serial.println("NAVIGATION"); break;
            case STATE_OBSTACLE_AVOIDANCE: Serial.println("ÉVITEMENT"); break;
            case STATE_DOCKING: Serial.println("DOCKING"); break;
            case STATE_EMERGENCY_STOP: Serial.println("ARRÊT URGENCE"); break;
        }

        Serial.printf("Position: X=%.2f Y=%.2f Theta=%.2f\n", robot_x, robot_y, robot_theta);
        Serial.printf("Vitesse: V=%.2f W=%.2f\n", current_v, current_w);

        tof_sensors.printStatus();
        ir_sensor.printStatus();
        actuators.printStatus();

        Serial.println("===============================\n");
    }
};