#pragma once
#include <Arduino.h>
#include "PCA9685.h" 
#include "pins.h"
#include "config.h"
#include "FastAccelStepper.h"

// ==========================================
// GESTION DES ACTIONNEURS - VEGA SC317
// ==========================================

class ActuatorManager {
private:
    ServoDriver pwm;
    
    // --- Le moteur matériel de l'ESP32 ---
    FastAccelStepperEngine engine;
    FastAccelStepper *steppers[6];

    bool motors_enabled;
    
    // --- RÉINTÉGRÉ : Pour mémoriser les vitesses et les afficher dans printStatus() ---
    float current_speeds_hz[6]; 

    const uint8_t PIN_STEP[6] = {PIN_STEP_M1, PIN_STEP_M2, PIN_STEP_M3, PIN_STEP_M4, PIN_STEP_M5, PIN_STEP_M6};
    const uint8_t PIN_DIR[6]  = {PIN_DIR_M1, PIN_DIR_M2, PIN_DIR_M3, PIN_DIR_M4, PIN_DIR_M5, PIN_DIR_M6};

public:
    ActuatorManager() : motors_enabled(false) {
        for(int i=0; i<6; i++) steppers[i] = nullptr;
        memset(current_speeds_hz, 0, sizeof(current_speeds_hz));
    }

    bool begin() {
        Serial.println("Initialisation des actionneurs...");
        pwm.init(0x7F); 

        pinMode(PIN_ENABLE_MOTORS, OUTPUT);
        digitalWrite(PIN_ENABLE_MOTORS, HIGH); 

        // Initialisation du moteur matériel de FastAccelStepper
        engine.init();

        for(int i=0; i<6; i++) {
            // Assigne une broche matérielle à la librairie
            steppers[i] = engine.stepperConnectToPin(PIN_STEP[i]);
            
            if (steppers[i]) {
                steppers[i]->setDirectionPin(PIN_DIR[i]);
                
                // On met une accélération très élevée car c'est ton EKF/PathFollower 
                // qui va dicter les variations de vitesse en douceur.
                steppers[i]->setAcceleration(50000); 
            }
        }

        Serial.println("Actionneurs initialisés (FastAccelStepper).");
        return true;
    }

    // --- NOUVEAU : Déplacement relatif (ex: faire X pas) ---
    void moveRelative(int motor_index, long steps, uint32_t speed_hz) {
        if (motor_index < 0 || motor_index > 5 || !steppers[motor_index]) return;
        
        steppers[motor_index]->setSpeedInHz(speed_hz);
        steppers[motor_index]->move(steps);
    }

    // --- NOUVEAU : Vérifier si un moteur tourne encore ---
    bool isMotorMoving(int motor_index) {
        if (motor_index < 0 || motor_index > 5 || !steppers[motor_index]) return false;
        return steppers[motor_index]->isRunning();
    }

    // --- Envoi des vitesses de croisière ---
    void setStepperSpeeds(float fl, float fr, float ml, float mr, float rl, float rr) {
        if (!motors_enabled) return;

        float speeds[6] = {fl, fr, ml, mr, rl, rr};

        for(int i=0; i<6; i++) {
            // Sauvegarde pour la fonction printStatus()
            current_speeds_hz[i] = speeds[i]; 

            if (steppers[i]) {
                long hz = abs(speeds[i]);
                
                if (hz < 1) { // Moins de 1 Hz = Arrêt total
                    steppers[i]->stopMove();
                } else {
                    // Limite de sécurité matérielle
                    hz = constrain(hz, 0, MAX_SPEED_HZ); 
                    
                    steppers[i]->setSpeedInHz(hz);
                    if (speeds[i] > 0) {
                        steppers[i]->runForward();
                    } else {
                        steppers[i]->runBackward();
                    }
                }
            }
        }
    }

    // --- Lecture directe de l'odométrie matérielle ---
    long getStepCount(int motor_index) {
        if (motor_index < 0 || motor_index > 5 || !steppers[motor_index]) return 0;
        // La librairie compte automatiquement les pas en positif et négatif !
        return steppers[motor_index]->getCurrentPosition(); 
    }

    void resetOdometry() {
        for(int i=0; i<6; i++) {
            if (steppers[i]) steppers[i]->setCurrentPosition(0);
        }
    }

    // Contrôle des servos (angles en radians)
    void setServoAngles(float fl, float fr, float rl, float rr) {
        int angle_fl = constrain((fl * 180.0 / M_PI) + 90, 0, 180);
        int angle_fr = constrain((fr * 180.0 / M_PI) + 90, 0, 180);
        int angle_rl = constrain((rl * 180.0 / M_PI) + 90, 0, 180);
        int angle_rr = constrain((rr * 180.0 / M_PI) + 90, 0, 180);

        pwm.setAngle(1, angle_fl);  
        pwm.setAngle(2, angle_fr);  
        pwm.setAngle(3, angle_rl);  
        pwm.setAngle(4, angle_rr);  
    }

    // Activation/Désactivation générale de la puissance des roues
    void enableMotors(bool enable) {
        motors_enabled = enable;
        digitalWrite(PIN_ENABLE_MOTORS, enable ? LOW : HIGH); // LOW = Activé sur les TMC2208

        // Si on désactive, on arrête les moteurs et on remet l'affichage à zéro
        if (!enable) {
            memset(current_speeds_hz, 0, sizeof(current_speeds_hz)); 
            for(int i=0; i<6; i++) {
                if (steppers[i]) steppers[i]->stopMove();
            }
        }
    }

    bool areMotorsEnabled() const {
        return motors_enabled;
    }

    void printStatus() {
        Serial.println("\n=== ÉTAT ACTIONNEURS ===");
        Serial.printf("Moteurs de traction : %s\n", motors_enabled ? "ACTIVÉS (LOW)" : "DÉSACTIVÉS (HIGH)");
        Serial.printf("Consignes (Hz) -> FL:%.0f FR:%.0f ML:%.0f MR:%.0f RL:%.0f RR:%.0f\n",
                      current_speeds_hz[0], current_speeds_hz[1], current_speeds_hz[2], 
                      current_speeds_hz[3], current_speeds_hz[4], current_speeds_hz[5]); 
        Serial.println("========================");
    }

};