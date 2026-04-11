#pragma once
#include <Arduino.h>
#include <TMCStepper.h>
#include <Adafruit_PWMServoDriver.h>
#include "pins.h"

// ==========================================
// GESTION DES ACTIONNEURS - VEGA SC317
// ==========================================

class ActuatorManager {
private:
    // --- Drivers Servos (PCA9685) ---
    Adafruit_PWMServoDriver pwm;

    // --- Drivers Steppers (TMC2208) ---
    TMC2208Stepper* driver_FL;
    TMC2208Stepper* driver_FR;
    TMC2208Stepper* driver_ML;
    TMC2208Stepper* driver_MR;
    TMC2208Stepper* driver_RL;
    TMC2208Stepper* driver_RR;

    // --- Paramètres Servos ---
    const uint16_t SERVO_MIN = 150;  // Pulse min (us)
    const uint16_t SERVO_MAX = 600;  // Pulse max (us)
    const float SERVO_RANGE_DEG = 180.0; // Plage angulaire

    // --- Paramètres Steppers ---
    const float STEPS_PER_REV = 200.0;
    const int MICROSTEPS = 16;
    const float MAX_SPEED_HZ = 2000.0; // Limite sécurité

    // État des moteurs
    bool motors_enabled;
    float current_speeds[6]; // FL, FR, ML, MR, RL, RR

public:
    ActuatorManager() : 
        driver_FL(nullptr),
        driver_FR(nullptr),
        driver_ML(nullptr),
        driver_MR(nullptr),
        driver_RL(nullptr),
        driver_RR(nullptr),
        motors_enabled(false) {
        memset(current_speeds, 0, sizeof(current_speeds));
    }

    ~ActuatorManager() {
        delete driver_FL;
        delete driver_FR;
        delete driver_ML;
        delete driver_MR;
        delete driver_RL;
        delete driver_RR;
    }

    // Initialisation complète
    bool begin() {
        Serial.println("Initialisation des actionneurs...");

        // 1. Initialisation PCA9685 (Servos)
        pwm.begin();
        pwm.setPWMFreq(50); // 50Hz pour servos
        delay(10);

        // 2. Initialisation TMC2208 (Steppers)
        // Configuration UART pour chaque driver
        Serial2.begin(115200, SERIAL_8N1, PIN_UART_RX, PIN_UART_TX); // Port série partagé

        // Create TMC2208Stepper objects
        driver_FL = new TMC2208Stepper(&Serial2, 0.11f);
        driver_FR = new TMC2208Stepper(&Serial2, 0.11f);
        driver_ML = new TMC2208Stepper(&Serial2, 0.11f);
        driver_MR = new TMC2208Stepper(&Serial2, 0.11f);
        driver_RL = new TMC2208Stepper(&Serial2, 0.11f);
        driver_RR = new TMC2208Stepper(&Serial2, 0.11f);

        // Driver Front Left
        driver_FL->begin();
        driver_FL->toff(4);
        driver_FL->tbl(1);
        driver_FL->rms_current(400); // 400mA pour NEMA17 0.4A
        driver_FL->microsteps(MICROSTEPS);
        driver_FL->pwm_autoscale(true);

        // Driver Front Right
        driver_FR->begin();
        driver_FR->toff(4);
        driver_FR->tbl(1);
        driver_FR->rms_current(400);
        driver_FR->microsteps(MICROSTEPS);
        driver_FR->pwm_autoscale(true);

        // Driver Middle Left
        driver_ML->begin();
        driver_ML->toff(4);
        driver_ML->tbl(1);
        driver_ML->rms_current(400);
        driver_ML->microsteps(MICROSTEPS);
        driver_ML->pwm_autoscale(true);

        // Driver Middle Right
        driver_MR->begin();
        driver_MR->toff(4);
        driver_MR->tbl(1);
        driver_MR->rms_current(400);
        driver_MR->microsteps(MICROSTEPS);
        driver_MR->pwm_autoscale(true);

        // Driver Rear Left
        driver_RL->begin();
        driver_RL->toff(4);
        driver_RL->tbl(1);
        driver_RL->rms_current(400);
        driver_RL->microsteps(MICROSTEPS);
        driver_RL->pwm_autoscale(true);

        // Driver Rear Right
        driver_RR->begin();
        driver_RR->toff(4);
        driver_RR->tbl(1);
        driver_RR->rms_current(400);
        driver_RR->microsteps(MICROSTEPS);
        driver_RR->pwm_autoscale(true);

        Serial.println("Actionneurs initialisés");
        return true;
    }

    // Contrôle des servos (angles en radians)
    void setServoAngles(float fl, float fr, float rl, float rr) {
        // Conversion radians -> degrés
        int angle_fl = (fl * 180.0 / M_PI) + 90; // Centré à 90°
        int angle_fr = (fr * 180.0 / M_PI) + 90;
        int angle_rl = (rl * 180.0 / M_PI) + 90;
        int angle_rr = (rr * 180.0 / M_PI) + 90;

        // Limitation des angles
        angle_fl = constrain(angle_fl, 0, 180);
        angle_fr = constrain(angle_fr, 0, 180);
        angle_rl = constrain(angle_rl, 0, 180);
        angle_rr = constrain(angle_rr, 0, 180);

        // Conversion degrés -> pulse width
        uint16_t pulse_fl = map(angle_fl, 0, 180, SERVO_MIN, SERVO_MAX);
        uint16_t pulse_fr = map(angle_fr, 0, 180, SERVO_MIN, SERVO_MAX);
        uint16_t pulse_rl = map(angle_rl, 0, 180, SERVO_MIN, SERVO_MAX);
        uint16_t pulse_rr = map(angle_rr, 0, 180, SERVO_MIN, SERVO_MAX);

        // Envoi aux servos
        pwm.setPWM(0, 0, pulse_fl);  // Servo FL sur canal 0
        pwm.setPWM(1, 0, pulse_fr);  // Servo FR sur canal 1
        pwm.setPWM(2, 0, pulse_rl);  // Servo RL sur canal 2
        pwm.setPWM(3, 0, pulse_rr);  // Servo RR sur canal 3
    }

    // Contrôle des steppers (vitesses en Hz)
    void setStepperSpeeds(float fl, float fr, float ml, float mr, float rl, float rr) {
        // Limitation des vitesses
        fl = constrain(fl, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        fr = constrain(fr, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        ml = constrain(ml, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        mr = constrain(mr, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        rl = constrain(rl, -MAX_SPEED_HZ, MAX_SPEED_HZ);
        rr = constrain(rr, -MAX_SPEED_HZ, MAX_SPEED_HZ);

        // Stockage des vitesses actuelles
        current_speeds[0] = fl;
        current_speeds[1] = fr;
        current_speeds[2] = ml;
        current_speeds[3] = mr;
        current_speeds[4] = rl;
        current_speeds[5] = rr;

        // Configuration des directions et vitesses
        // (Utilise les pins DIR et STEP définis dans pins.h)
        digitalWrite(PIN_DIR_M1, fl >= 0 ? HIGH : LOW);
        digitalWrite(PIN_DIR_M2, fr >= 0 ? HIGH : LOW);
        digitalWrite(PIN_DIR_M3, ml >= 0 ? HIGH : LOW);
        // ... continuer pour les autres moteurs

        // Les impulsions STEP sont générées par timer interrupts
        // ou par une tâche RTOS séparée
    }

    // Activation/désactivation des moteurs
    void enableMotors(bool enable) {
        motors_enabled = enable;
        digitalWrite(PIN_ENABLE_MOTORS, enable ? LOW : HIGH); // Active low

        if (!enable) {
            // Arrêt d'urgence - toutes vitesses à zéro
            memset(current_speeds, 0, sizeof(current_speeds));
        }
    }

    // État des moteurs
    bool areMotorsEnabled() const {
        return motors_enabled;
    }

    // Diagnostic
    void printStatus() {
        Serial.println("=== ÉTAT ACTIONNEURS ===");
        Serial.printf("Moteurs: %s\n", motors_enabled ? "ACTIVÉS" : "DÉSACTIVÉS");
        Serial.printf("Vitesses (Hz): FL:%.0f FR:%.0f ML:%.0f MR:%.0f RL:%.0f RR:%.0f\n",
                      current_speeds[0], current_speeds[1], current_speeds[2],
                      current_speeds[3], current_speeds[4], current_speeds[5]);
    }
};