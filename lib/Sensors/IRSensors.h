#pragma once
#include <Arduino.h>

// ==========================================
// CAPTEUR IR 38kHz - DOCKING DE PRÉCISION
// ==========================================

class IRDockingSensor {
private:
    const uint8_t IR_PIN;
    const uint8_t LED_PIN;  // LED émettrice IR

    // Paramètres de modulation
    const uint16_t CARRIER_FREQ = 38000;  // 38kHz
    const uint16_t BURST_DURATION_US = 500;  // Durée d'émission

    // Seuils de détection
    const uint16_t DETECTION_THRESHOLD = 800;  // Valeur ADC pour détection
    const float TARGET_DISTANCE_CM = 5.0;     // Distance idéale de docking

    // État de détection
    bool target_detected;
    uint16_t last_reading;
    unsigned long last_measurement;

public:
    IRDockingSensor(uint8_t ir_pin, uint8_t led_pin = 255)
        : IR_PIN(ir_pin), LED_PIN(led_pin), target_detected(false),
          last_reading(0), last_measurement(0) {}

    // Initialisation
    void begin() {
        pinMode(IR_PIN, INPUT);

        if (LED_PIN != 255) {
            pinMode(LED_PIN, OUTPUT);
            digitalWrite(LED_PIN, LOW);
        }

        Serial.println("Capteur IR de docking initialisé");
    }

    // Mesure de distance (basé sur réflexion IR)
    uint16_t measureDistance() {
        if (LED_PIN != 255) {
            // Émission d'une burst IR modulée
            emitIRBurst();
        }

        // Lecture de la réflexion
        delayMicroseconds(100); // Attendre la réflexion
        last_reading = analogRead(IR_PIN);
        last_measurement = millis();

        return last_reading;
    }

    // Émission d'une burst IR 38kHz
    void emitIRBurst() {
        // Génération de la porteuse 38kHz
        unsigned long start_time = micros();
        while (micros() - start_time < BURST_DURATION_US) {
            digitalWrite(LED_PIN, HIGH);
            delayMicroseconds(13);  // Demi-période 38kHz ≈ 13.16us
            digitalWrite(LED_PIN, LOW);
            delayMicroseconds(13);
        }
    }

    // Vérification de la présence de cible
    bool isTargetDetected() {
        // Mesure automatique si nécessaire
        if (millis() - last_measurement > 100) {  // Toutes les 100ms
            measureDistance();
        }

        target_detected = (last_reading > DETECTION_THRESHOLD);
        return target_detected;
    }

    // Calcul de l'erreur de position (pour asservissement fin)
    float getPositionError() {
        if (!isTargetDetected()) return 999.0;  // Erreur maximale

        // Conversion ADC -> distance approximative
        // Cette calibration dépend de votre capteur spécifique
        float distance_cm = map(last_reading, 0, 1023, 50, 0);  // Calibration exemple

        // Erreur par rapport à la distance idéale
        return distance_cm - TARGET_DISTANCE_CM;
    }

    // État de docking réussi
    bool isDocked() {
        if (!isTargetDetected()) return false;

        float error = getPositionError();
        return abs(error) < 1.0;  // Tolérance de 1cm
    }

    // Diagnostic
    void printStatus() {
        Serial.println("=== CAPTEUR IR DOCKING ===");
        Serial.printf("Lecture ADC: %d\n", last_reading);
        Serial.printf("Cible détectée: %s\n", target_detected ? "OUI" : "NON");
        Serial.printf("Distance estimée: %.1f cm\n", map(last_reading, 0, 1023, 50, 0));
        Serial.printf("Erreur de position: %.1f cm\n", getPositionError());
        Serial.printf("Docking réussi: %s\n", isDocked() ? "OUI" : "NON");
    }
};