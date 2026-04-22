#pragma once
#include <Arduino.h>
#include "PCA9685.h" // Bibliothèque Seeed Studio
#include "pins.h"
#include "config.h"

// ==========================================
// GESTION DES ACTIONNEURS - VEGA SC317
// ==========================================

class ActuatorManager {
private:
    // --- Drivers Servos (PCA9685) ---
    ServoDriver pwm;


    // État des moteurs
    bool motors_enabled;
    float current_speeds_hz[6]; // FL, FR, ML, MR, RL, RR en Hz
    unsigned long step_interval_us[6];   // Temps entre deux pas (en microsecondes)
    unsigned long last_step_time_us[6];  // Heure du dernier pas

    // --- NOUVEAU : Odométrie (Compteurs de pas) ---
    // volatile car ils pourraient plus tard être modifiés par une interruption
    volatile long step_counters[6];      

    // Tableaux de correspondance des pins pour simplifier le code
    const uint8_t PIN_STEP[6] = {PIN_STEP_M1, PIN_STEP_M2, PIN_STEP_M3, PIN_STEP_M4, PIN_STEP_M5, PIN_STEP_M6};
    const uint8_t PIN_DIR[6]  = {PIN_DIR_M1, PIN_DIR_M2, PIN_DIR_M3, PIN_DIR_M4, PIN_DIR_M5, PIN_DIR_M6};

public:
    ActuatorManager() : motors_enabled(false) {
        memset(current_speeds_hz, 0, sizeof(current_speeds_hz));
        memset(step_interval_us, 0, sizeof(step_interval_us));
        memset(last_step_time_us, 0, sizeof(last_step_time_us));
        memset((void*)step_counters, 0, sizeof(step_counters));
    }

    bool begin() {
        Serial.println("Initialisation des actionneurs...");
        pwm.init(0x7F); 
        delay(10);

        pinMode(PIN_ENABLE_MOTORS, OUTPUT);
        digitalWrite(PIN_ENABLE_MOTORS, HIGH); 
        
        // Initialisation de toutes les broches moteurs avec une boucle
        for(int i=0; i<6; i++) {
            pinMode(PIN_STEP[i], OUTPUT);
            pinMode(PIN_DIR[i], OUTPUT);
            digitalWrite(PIN_STEP[i], LOW);
        }

        Serial.println("Actionneurs initialisés.");
        return true;
    }

    // --- MISE À JOUR : Calcul des intervalles ---
    void setStepperSpeeds(float fl, float fr, float ml, float mr, float rl, float rr) {
        float speeds[6] = {fl, fr, ml, mr, rl, rr};

        for(int i=0; i<6; i++) {
            current_speeds_hz[i] = constrain(speeds[i], -MAX_SPEED_HZ, MAX_SPEED_HZ);
            
            if (abs(current_speeds_hz[i]) > 0.1) {
                // Direction
                digitalWrite(PIN_DIR[i], current_speeds_hz[i] >= 0 ? HIGH : LOW);
                // Calcul de l'intervalle en microsecondes (ex: 1000 Hz = 1000 us)
                step_interval_us[i] = 1000000.0 / abs(current_speeds_hz[i]);
            } else {
                step_interval_us[i] = 0; // Moteur à l'arrêt
            }
        }
    }

    // --- NOUVEAU : La fonction vitale à appeler dans le loop() ---
    void updateSteppers() {
        if (!motors_enabled) return;

        unsigned long now = micros();

        for(int i=0; i<6; i++) {
            // Si le moteur doit tourner ET que le délai est écoulé
            if (step_interval_us[i] > 0 && (now - last_step_time_us[i]) >= step_interval_us[i]) {
                
                // 1. Générer l'impulsion physique
                digitalWrite(PIN_STEP[i], HIGH);
                delayMicroseconds(2); // Les TMC2208 ont besoin d'environ 1 à 2µs à l'état haut
                digitalWrite(PIN_STEP[i], LOW);

                // 2. Mettre à jour l'horloge
                last_step_time_us[i] = now;

                // 3. Compter le pas pour l'odométrie
                if (current_speeds_hz[i] > 0) step_counters[i]++;
                else step_counters[i]--;
            }
        }
    }

    // --- NOUVEAU : Lecture de l'odométrie ---
    long getStepCount(int motor_index) {
        if (motor_index < 0 || motor_index > 5) return 0;
        return step_counters[motor_index];
    }

    void resetOdometry() {
        memset((void*)step_counters, 0, sizeof(step_counters));
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

        if (!enable) {
            memset(current_speeds_hz, 0, sizeof(current_speeds_hz)); // CORRIGÉ ICI
        }
    }

    bool areMotorsEnabled() const {
        return motors_enabled;
    }

    void printStatus() {
        Serial.println("\n=== ÉTAT ACTIONNEURS ===");
        Serial.printf("Moteurs de traction : %s\n", motors_enabled ? "ACTIVÉS (LOW)" : "DÉSACTIVÉS (HIGH)");
        Serial.printf("Consignes (Hz) -> FL:%.0f FR:%.0f ML:%.0f MR:%.0f RL:%.0f RR:%.0f\n",
                      current_speeds_hz[0], current_speeds_hz[1], current_speeds_hz[2], // CORRIGÉ ICI
                      current_speeds_hz[3], current_speeds_hz[4], current_speeds_hz[5]); // CORRIGÉ ICI
        Serial.println("========================");
    }
};