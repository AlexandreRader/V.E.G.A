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
    VL53LX* tof_front_left;
    VL53LX* tof_front_right;
    VL53LX* tof_rear_left;
    VL53LX* tof_rear_right;

    uint16_t distances[4];
    bool data_ready[4];
    bool active[4]; 
    uint8_t error_count[4]; 

    const uint16_t OBSTACLE_THRESHOLD_MM = 500;  
    const uint16_t CRITICAL_DISTANCE_MM = 200;   
    const uint16_t OUT_OF_RANGE_MM = 8190;

public:
    ToFManager() : 
        tof_front_left(nullptr), tof_front_right(nullptr),
        tof_rear_left(nullptr), tof_rear_right(nullptr) {
        memset(distances, 0, sizeof(distances));
        memset(data_ready, 0, sizeof(data_ready));
        memset(active, 0, sizeof(active));
        memset(error_count, 0, sizeof(error_count));
    }

    ~ToFManager() {
        delete tof_front_left; delete tof_front_right;
        delete tof_rear_left; delete tof_rear_right;
    }

    bool begin() {
        Serial.println("Initialisation capteurs ToF...");

        tof_front_left = new VL53LX(&Wire, PIN_TOF_1_AVANT);
        tof_front_right = new VL53LX(&Wire, PIN_TOF_2_ARRIERE);
        tof_rear_left = new VL53LX(&Wire, PIN_TOF_3_GAUCHE);
        tof_rear_right = new VL53LX(&Wire, PIN_TOF_4_DROITE);

        // EXTINCTION FORCÉE MANUELLE
        pinMode(PIN_TOF_1_AVANT, OUTPUT); digitalWrite(PIN_TOF_1_AVANT, LOW);
        pinMode(PIN_TOF_2_ARRIERE, OUTPUT); digitalWrite(PIN_TOF_2_ARRIERE, LOW);
        pinMode(PIN_TOF_3_GAUCHE, OUTPUT); digitalWrite(PIN_TOF_3_GAUCHE, LOW);
        pinMode(PIN_TOF_4_DROITE, OUTPUT); digitalWrite(PIN_TOF_4_DROITE, LOW);
        delay(100); 

        // --- Capteur 1 ---
        digitalWrite(PIN_TOF_1_AVANT, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            // 0x54 est l'adresse 8-bit ST (équivaut à 0x2A en 7-bit)
            if (tof_front_left->InitSensor(0x54) == VL53LX_ERROR_NONE) {
                if (tof_front_left->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    // KICKSTART : On purge l'interruption immédiatement pour forcer le laser !
                    tof_front_left->VL53LX_ClearInterruptAndStartMeasurement(); 
                    active[0] = true;
                    Serial.println(" - ToF Avant Gauche : ✅ OK (0x54)");
                }
            }
        }
        if (!active[0]) {
            digitalWrite(PIN_TOF_1_AVANT, LOW);
            Serial.println(" - ToF Avant Gauche : ❌ ECHEC");
        }

        // --- Capteur 2 ---
        digitalWrite(PIN_TOF_2_ARRIERE, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_front_right->InitSensor(0x56) == VL53LX_ERROR_NONE) {
                if (tof_front_right->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_front_right->VL53LX_ClearInterruptAndStartMeasurement();
                    active[1] = true;
                    Serial.println(" - ToF Avant Droit  : ✅ OK (0x56)");
                }
            }
        }
        if (!active[1]) {
            digitalWrite(PIN_TOF_2_ARRIERE, LOW);
            Serial.println(" - ToF Avant Droit  : ❌ ECHEC");
        }

        // --- Capteur 3 ---
        digitalWrite(PIN_TOF_3_GAUCHE, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_rear_left->InitSensor(0x58) == VL53LX_ERROR_NONE) {
                if (tof_rear_left->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_rear_left->VL53LX_ClearInterruptAndStartMeasurement();
                    active[2] = true;
                    Serial.println(" - ToF Arr Gauche   : ✅ OK (0x58)");
                }
            }
        }
        if (!active[2]) {
            digitalWrite(PIN_TOF_3_GAUCHE, LOW);
            Serial.println(" - ToF Arr Gauche   : ❌ ECHEC");
        }

        // --- Capteur 4 ---
        digitalWrite(PIN_TOF_4_DROITE, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_rear_right->InitSensor(0x5A) == VL53LX_ERROR_NONE) {
                if (tof_rear_right->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_rear_right->VL53LX_ClearInterruptAndStartMeasurement();
                    active[3] = true;
                    Serial.println(" - ToF Arr Droit    : ✅ OK (0x5A)");
                }
            }
        }
        if (!active[3]) {
            digitalWrite(PIN_TOF_4_DROITE, LOW);
            Serial.println(" - ToF Arr Droit    : ❌ ECHEC");
        }

        return (active[0] || active[1] || active[2] || active[3]);
    }

    void update() {
        VL53LX_MultiRangingData_t MultiRangingData;
        uint8_t NewDataReady = 0;

        // --- 1. Front Left ---
        if (active[0]) {
            if (tof_front_left->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[0] = 0; 
                if (NewDataReady != 0) { // Le capteur a bien levé le drapeau !
                    if (tof_front_left->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[0] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else {
                            distances[0] = OUT_OF_RANGE_MM; 
                        }
                        data_ready[0] = true;
                    }
                    tof_front_left->VL53LX_ClearInterruptAndStartMeasurement(); // Relance la mesure suivante
                }
            } else if (++error_count[0] > 5) {
                active[0] = false; data_ready[0] = false;
                Serial.println("\n⚠️ ALARME : ToF Avant Gauche I2C Perdu !");
            }
        }

        // --- 2. Front Right ---
        if (active[1]) {
            if (tof_front_right->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[1] = 0;
                if (NewDataReady != 0) {
                    if (tof_front_right->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[1] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else distances[1] = OUT_OF_RANGE_MM;
                        data_ready[1] = true;
                    }
                    tof_front_right->VL53LX_ClearInterruptAndStartMeasurement();
                }
            } else if (++error_count[1] > 5) {
                active[1] = false; data_ready[1] = false;
                Serial.println("\n⚠️ ALARME : ToF Avant Droit I2C Perdu !");
            }
        }

        // --- 3. Rear Left ---
        if (active[2]) {
            if (tof_rear_left->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[2] = 0;
                if (NewDataReady != 0) {
                    if (tof_rear_left->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[2] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else distances[2] = OUT_OF_RANGE_MM;
                        data_ready[2] = true;
                    }
                    tof_rear_left->VL53LX_ClearInterruptAndStartMeasurement();
                }
            } else if (++error_count[2] > 5) {
                active[2] = false; data_ready[2] = false;
                Serial.println("\n⚠️ ALARME : ToF Arr Gauche I2C Perdu !");
            }
        }

        // --- 4. Rear Right ---
        if (active[3]) {
            if (tof_rear_right->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[3] = 0;
                if (NewDataReady != 0) {
                    if (tof_rear_right->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[3] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else distances[3] = OUT_OF_RANGE_MM;
                        data_ready[3] = true;
                    }
                    tof_rear_right->VL53LX_ClearInterruptAndStartMeasurement();
                }
            } else if (++error_count[3] > 5) {
                active[3] = false; data_ready[3] = false;
                Serial.println("\n⚠️ ALARME : ToF Arr Droit I2C Perdu !");
            }
        }
    }

    void printStatus() {
        Serial.println("=== ÉTAT CAPTEURS ToF ===");
        Serial.printf("Avant Gauche: %d mm (%s)\n", distances[0], data_ready[0] ? "OK" : (active[0] ? "Wait" : "OFF"));
        Serial.printf("Avant Droit:  %d mm (%s)\n", distances[1], data_ready[1] ? "OK" : (active[1] ? "Wait" : "OFF"));
        Serial.printf("Arrière Gauche: %d mm (%s)\n", distances[2], data_ready[2] ? "OK" : (active[2] ? "Wait" : "OFF"));
        Serial.printf("Arrière Droit:  %d mm (%s)\n", distances[3], data_ready[3] ? "OK" : (active[3] ? "Wait" : "OFF"));
    }
};