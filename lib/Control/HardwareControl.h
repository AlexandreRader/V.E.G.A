#pragma once
#include <Arduino.h>
#include "PCA9685.h" // Bibliothèque Seeed Studio
#include "pins.h"

// ==========================================
// GESTION DES ACTIONNEURS - VEGA SC317
// ==========================================

class ActuatorManager {
private:
    // --- Drivers Servos (PCA9685) ---
    ServoDriver pwm;

    // --- Paramètres Steppers ---
    const float MAX_SPEED_HZ = 2000.0; // Limite de sécurité pour les impulsions

    // État des moteurs
    bool motors_enabled;
    float current_speeds[6]; // FL, FR, ML, MR, RL, RR en Hz

public:
    ActuatorManager() : motors_enabled(false) {
        memset(current_speeds, 0, sizeof(current_speeds));
    }

    ~ActuatorManager() {
        // Plus rien à détruire ici puisque nous n'utilisons plus de pointeurs UART
    }

    // Initialisation complète
    bool begin() {
        Serial.println("Initialisation des actionneurs...");

        // 1. Initialisation du PCA9685 (Servos) via I2C
        // La fonction init() règle automatiquement la fréquence à 50Hz 
        pwm.init(0x7F); 
        delay(10);

        // 2. Configuration des broches GPIO pour les Steppers (Step/Dir)
        // Il est plus propre de déclarer les pinMode ici que dans le main.cpp
        pinMode(PIN_ENABLE_MOTORS, OUTPUT);
        digitalWrite(PIN_ENABLE_MOTORS, HIGH); // Moteurs désactivés par défaut (actif à l'état bas)
        
        // Moteur 1 (Exemple, à dupliquer pour les autres)
        pinMode(PIN_DIR_M1, OUTPUT);
        pinMode(PIN_STEP_M1, OUTPUT);
        
        /* // Décommente et ajoute les autres pins quand tu les auras définis dans pins.h
        pinMode(PIN_DIR_M2, OUTPUT); pinMode(PIN_STEP_M2, OUTPUT);
        pinMode(PIN_DIR_M3, OUTPUT); pinMode(PIN_STEP_M3, OUTPUT);
        pinMode(PIN_DIR_M4, OUTPUT); pinMode(PIN_STEP_M4, OUTPUT);
        pinMode(PIN_DIR_M5, OUTPUT); pinMode(PIN_STEP_M5, OUTPUT);
        pinMode(PIN_DIR_M6, OUTPUT); pinMode(PIN_STEP_M6, OUTPUT);
        */

        Serial.println("Actionneurs initialisés (PCA9685 I2C + Steppers GPIO)");
        return true;
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

    // Contrôle des directions (Les impulsions STEP sont gérées ailleurs)
    void setStepperSpeeds(float fl, float fr, float ml, float mr, float rl, float rr) {
        // Limitation logicielle
        fl = constrain(fl, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        fr = constrain(fr, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        ml = constrain(ml, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        mr = constrain(mr, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        rl = constrain(rl, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        rr = constrain(rr, -MAX_SPEED_HZ, MAX_SPEED_HZ);

        // Mise à jour des consignes
        current_speeds[0] = fl;
        current_speeds[1] = fr;
        current_speeds[2] = ml;
        current_speeds[3] = mr;
        current_speeds[4] = rl;
        current_speeds[5] = rr;

        // Mise à jour physique des broches de DIRECTION
        // Si la vitesse est positive, on met à HIGH, sinon LOW (ou l'inverse selon le câblage de tes moteurs)
        digitalWrite(PIN_DIR_M1, fl >= 0 ? HIGH : LOW);
        
        /* // Décommente quand tes pins seront définis
        digitalWrite(PIN_DIR_M2, fr >= 0 ? HIGH : LOW);
        digitalWrite(PIN_DIR_M3, ml >= 0 ? HIGH : LOW);
        digitalWrite(PIN_DIR_M4, mr >= 0 ? HIGH : LOW);
        digitalWrite(PIN_DIR_M5, rl >= 0 ? HIGH : LOW);
        digitalWrite(PIN_DIR_M6, rr >= 0 ? HIGH : LOW);
        */

        // ⚠️ RAPPEL : Cette fonction ne fait pas tourner les moteurs. 
        // Elle donne juste la consigne (Sens + Vitesse cible). 
        // Tu devras utiliser un Timer (Interrupt) ou une tâche FreeRTOS 
        // pour générer les impulsions sur les PIN_STEP_M... en fonction de current_speeds.
    }

    // Activation/Désactivation générale de la puissance des roues
    void enableMotors(bool enable) {
        motors_enabled = enable;
        digitalWrite(PIN_ENABLE_MOTORS, enable ? LOW : HIGH); // LOW = Activé sur les TMC2208

        if (!enable) {
            memset(current_speeds, 0, sizeof(current_speeds));
        }
    }

    bool areMotorsEnabled() const {
        return motors_enabled;
    }

    void printStatus() {
        Serial.println("\n=== ÉTAT ACTIONNEURS ===");
        Serial.printf("Moteurs de traction : %s\n", motors_enabled ? "ACTIVÉS (LOW)" : "DÉSACTIVÉS (HIGH)");
        Serial.printf("Consignes (Hz) -> FL:%.0f FR:%.0f ML:%.0f MR:%.0f RL:%.0f RR:%.0f\n",
                      current_speeds[0], current_speeds[1], current_speeds[2],
                      current_speeds[3], current_speeds[4], current_speeds[5]);
        Serial.println("========================");
    }
};