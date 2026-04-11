#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>
#include "pins.h"

// ==========================================
// GESTION CAPTEURS ToF - VEGA SC317
// ==========================================

class ToFManager {
private:
    // Capteurs ToF individuels
    VL53LX* tof_front_left;
    VL53LX* tof_front_right;
    VL53LX* tof_rear_left;
    VL53LX* tof_rear_right;

    // Données de distance (mm)
    uint16_t distances[4];
    bool data_ready[4];

    // Seuils de détection d'obstacle
    const uint16_t OBSTACLE_THRESHOLD_MM = 500;  // 50cm
    const uint16_t CRITICAL_DISTANCE_MM = 200;   // 20cm - arrêt d'urgence

public:
    ToFManager() : 
        tof_front_left(nullptr),
        tof_front_right(nullptr),
        tof_rear_left(nullptr),
        tof_rear_right(nullptr) {
        memset(distances, 0, sizeof(distances));
        memset(data_ready, 0, sizeof(data_ready));
    }

    ~ToFManager() {
        delete tof_front_left;
        delete tof_front_right;
        delete tof_rear_left;
        delete tof_rear_right;
    }

    // Initialisation des 4 capteurs ToF
    bool begin() {
        Serial.println("Initialisation capteurs ToF...");

        // Initialize I2C
        Wire.begin();

        // Create sensor objects
        tof_front_left = new VL53LX(&Wire, PIN_TOF_1_AVANT);
        tof_front_right = new VL53LX(&Wire, PIN_TOF_2_ARRIERE);
        tof_rear_left = new VL53LX(&Wire, PIN_TOF_3_GAUCHE);
        tof_rear_right = new VL53LX(&Wire, PIN_TOF_4_DROITE);

        // Initialize sensors with different addresses
        if (tof_front_left->InitSensor(0x29) != VL53LX_ERROR_NONE) {
            Serial.println("ERREUR: ToF Avant Gauche non détecté");
            return false;
        }
        tof_front_left->VL53LX_StartMeasurement();

        if (tof_front_right->InitSensor(0x2A) != VL53LX_ERROR_NONE) {
            Serial.println("ERREUR: ToF Avant Droit non détecté");
            return false;
        }
        tof_front_right->VL53LX_StartMeasurement();

        if (tof_rear_left->InitSensor(0x2B) != VL53LX_ERROR_NONE) {
            Serial.println("ERREUR: ToF Arrière Gauche non détecté");
            return false;
        }
        tof_rear_left->VL53LX_StartMeasurement();

        if (tof_rear_right->InitSensor(0x2C) != VL53LX_ERROR_NONE) {
            Serial.println("ERREUR: ToF Arrière Droit non détecté");
            return false;
        }
        tof_rear_right->VL53LX_StartMeasurement();

        Serial.println("Capteurs ToF initialisés avec succès");
        return true;
    }

    // Mise à jour des mesures
    void update() {
        VL53LX_MultiRangingData_t MultiRangingData;
        uint8_t NewDataReady = 0;

        // Front Left
        if (tof_front_left->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE && NewDataReady) {
            if (tof_front_left->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                if (MultiRangingData.NumberOfObjectsFound > 0) {
                    distances[0] = MultiRangingData.RangeData[0].RangeMilliMeter;
                    data_ready[0] = true;
                }
            }
            tof_front_left->VL53LX_ClearInterruptAndStartMeasurement();
        }

        // Front Right
        if (tof_front_right->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE && NewDataReady) {
            if (tof_front_right->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                if (MultiRangingData.NumberOfObjectsFound > 0) {
                    distances[1] = MultiRangingData.RangeData[0].RangeMilliMeter;
                    data_ready[1] = true;
                }
            }
            tof_front_right->VL53LX_ClearInterruptAndStartMeasurement();
        }

        // Rear Left
        if (tof_rear_left->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE && NewDataReady) {
            if (tof_rear_left->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                if (MultiRangingData.NumberOfObjectsFound > 0) {
                    distances[2] = MultiRangingData.RangeData[0].RangeMilliMeter;
                    data_ready[2] = true;
                }
            }
            tof_rear_left->VL53LX_ClearInterruptAndStartMeasurement();
        }

        // Rear Right
        if (tof_rear_right->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE && NewDataReady) {
            if (tof_rear_right->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                if (MultiRangingData.NumberOfObjectsFound > 0) {
                    distances[3] = MultiRangingData.RangeData[0].RangeMilliMeter;
                    data_ready[3] = true;
                }
            }
            tof_rear_right->VL53LX_ClearInterruptAndStartMeasurement();
        }
    }

    // Accesseurs pour les distances
    uint16_t getFrontLeftDistance() const { return distances[0]; }
    uint16_t getFrontRightDistance() const { return distances[1]; }
    uint16_t getRearLeftDistance() const { return distances[2]; }
    uint16_t getRearRightDistance() const { return distances[3]; }

    // Vérification d'obstacles
    bool hasObstacleFront() const {
        return (data_ready[0] && distances[0] < OBSTACLE_THRESHOLD_MM) ||
               (data_ready[1] && distances[1] < OBSTACLE_THRESHOLD_MM);
    }

    bool hasObstacleRear() const {
        return (data_ready[2] && distances[2] < OBSTACLE_THRESHOLD_MM) ||
               (data_ready[3] && distances[3] < OBSTACLE_THRESHOLD_MM);
    }

    bool hasObstacleLeft() const {
        return data_ready[2] && distances[2] < OBSTACLE_THRESHOLD_MM;
    }

    bool hasObstacleRight() const {
        return data_ready[3] && distances[3] < OBSTACLE_THRESHOLD_MM;
    }

    // Arrêt d'urgence (obstacle critique)
    bool emergencyStopRequired() const {
        for (int i = 0; i < 4; i++) {
            if (data_ready[i] && distances[i] < CRITICAL_DISTANCE_MM) {
                return true;
            }
        }
        return false;
    }

    // Calcul de la distance de sécurité (pour navigation)
    float getMinFrontDistance() const {
        if (!data_ready[0] && !data_ready[1]) return 999.0;
        float min_dist = 999.0;
        if (data_ready[0]) min_dist = std::min(min_dist, static_cast<float>(distances[0]) / 1000.0f);
        if (data_ready[1]) min_dist = std::min(min_dist, static_cast<float>(distances[1]) / 1000.0f);
        return min_dist;
    }

    // Diagnostic
    void printStatus() {
        Serial.println("=== ÉTAT CAPTEURS ToF ===");
        Serial.printf("Avant Gauche: %d mm (%s)\n", distances[0], data_ready[0] ? "OK" : "N/A");
        Serial.printf("Avant Droit:  %d mm (%s)\n", distances[1], data_ready[1] ? "OK" : "N/A");
        Serial.printf("Arrière Gauche: %d mm (%s)\n", distances[2], data_ready[2] ? "OK" : "N/A");
        Serial.printf("Arrière Droit:  %d mm (%s)\n", distances[3], data_ready[3] ? "OK" : "N/A");

        if (emergencyStopRequired()) {
            Serial.println("⚠️  ARRÊT D'URGENCE DÉTECTÉ!");
        }
    }
};