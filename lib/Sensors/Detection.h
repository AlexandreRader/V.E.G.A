#pragma once
#include <Arduino.h>
#include <Wire.h>
#include <vl53lx_class.h>
#include "pins.h"

// 1. Définition des types d'obstacles
enum ObstacleSignature {
    OBSTACLE_NONE,       // Voie libre
    OBSTACLE_PASSABLE,   // Franchissable (ex: petit caillou)
    OBSTACLE_WALL,       // Mur infranchissable
    OBSTACLE_SLOPE       // Pente montante
};

// ==========================================
// GESTION CAPTEURS ToF - MATRICE AVANT 2X2
// ==========================================

class ToFManager {
private:
    VL53LX* tof_top_left;
    VL53LX* tof_top_right;
    VL53LX* tof_bottom_left;
    VL53LX* tof_bottom_right;

    uint16_t distances[4];
    bool data_ready[4];
    bool active[4]; 
    uint8_t error_count[4]; 

    const uint16_t OBSTACLE_THRESHOLD_MM = 500;  
    const uint16_t CRITICAL_DISTANCE_MM = 200;   
    const uint16_t OUT_OF_RANGE_MM = 8190;

public:
    ToFManager() : 
        tof_top_left(nullptr), tof_top_right(nullptr),
        tof_bottom_left(nullptr), tof_bottom_right(nullptr) {
        memset(distances, 0, sizeof(distances));
        memset(data_ready, 0, sizeof(data_ready));
        memset(active, 0, sizeof(active));
        memset(error_count, 0, sizeof(error_count));
    }

    ~ToFManager() {
        delete tof_top_left; delete tof_top_right;
        delete tof_bottom_left; delete tof_bottom_right;
    }

    bool begin() {
        Serial.println("Initialisation capteurs ToF (Face Avant)...");

        // Instanciation des objets avec les broches associées (Matrice 2x2)
        tof_top_left     = new VL53LX(&Wire, PIN_TOF_1_LT);
        tof_top_right    = new VL53LX(&Wire, PIN_TOF_2_RT);
        tof_bottom_left  = new VL53LX(&Wire, PIN_TOF_3_LB);
        tof_bottom_right = new VL53LX(&Wire, PIN_TOF_4_RB);

        // ==========================================
        // EXTINCTION FORCÉE GLOBALE (Reset initial)
        // ==========================================
        pinMode(PIN_TOF_1_LT, OUTPUT); digitalWrite(PIN_TOF_1_LT, LOW);
        pinMode(PIN_TOF_2_RT, OUTPUT); digitalWrite(PIN_TOF_2_RT, LOW);
        pinMode(PIN_TOF_3_LB, OUTPUT); digitalWrite(PIN_TOF_3_LB, LOW);
        pinMode(PIN_TOF_4_RB, OUTPUT); digitalWrite(PIN_TOF_4_RB, LOW);
        delay(100); 

        // ==========================================
        // INITIALISATION SÉQUENTIELLE PAR CAPTEUR
        // ==========================================

        // --- Capteur 1 : Haut Gauche (Top Left - 0x54) ---
        digitalWrite(PIN_TOF_1_LT, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_top_left->InitSensor(0x54) == VL53LX_ERROR_NONE) {
                if (tof_top_left->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_top_left->VL53LX_ClearInterruptAndStartMeasurement(); 
                    active[0] = true;
                    Serial.println(" - ToF Haut Gauche  : ✅ OK (0x54)");
                }
            }
        }
        if (!active[0]) {
            digitalWrite(PIN_TOF_1_LT, LOW);
            Serial.println(" - ToF Haut Gauche  : ❌ ECHEC");
        }

        // --- Capteur 2 : Haut Droit (Top Right - 0x56) ---
        digitalWrite(PIN_TOF_2_RT, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_top_right->InitSensor(0x56) == VL53LX_ERROR_NONE) {
                if (tof_top_right->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_top_right->VL53LX_ClearInterruptAndStartMeasurement();
                    active[1] = true;
                    Serial.println(" - ToF Haut Droit   : ✅ OK (0x56)");
                }
            }
        }
        if (!active[1]) {
            digitalWrite(PIN_TOF_2_RT, LOW);
            Serial.println(" - ToF Haut Droit   : ❌ ECHEC");
        }

        // --- Capteur 3 : Bas Gauche (Bottom Left - 0x58) ---
        digitalWrite(PIN_TOF_3_LB, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_bottom_left->InitSensor(0x58) == VL53LX_ERROR_NONE) {
                if (tof_bottom_left->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_bottom_left->VL53LX_ClearInterruptAndStartMeasurement();
                    active[2] = true;
                    Serial.println(" - ToF Bas Gauche   : ✅ OK (0x58)");
                }
            }
        }
        if (!active[2]) {
            digitalWrite(PIN_TOF_3_LB, LOW);
            Serial.println(" - ToF Bas Gauche   : ❌ ECHEC");
        }

        // --- Capteur 4 : Bas Droit (Bottom Right - 0x5A) ---
        digitalWrite(PIN_TOF_4_RB, HIGH); delay(20);
        Wire.beginTransmission(0x29);
        if (Wire.endTransmission() == 0) {
            if (tof_bottom_right->InitSensor(0x5A) == VL53LX_ERROR_NONE) {
                if (tof_bottom_right->VL53LX_StartMeasurement() == VL53LX_ERROR_NONE) {
                    tof_bottom_right->VL53LX_ClearInterruptAndStartMeasurement();
                    active[3] = true;
                    Serial.println(" - ToF Bas Droit    : ✅ OK (0x5A)");
                }
            }
        }
        if (!active[3]) {
            digitalWrite(PIN_TOF_4_RB, LOW);
            Serial.println(" - ToF Bas Droit    : ❌ ECHEC");
        }

        return (active[0] || active[1] || active[2] || active[3]);
    }

    void update() {
        VL53LX_MultiRangingData_t MultiRangingData;
        uint8_t NewDataReady = 0;

        // --- 1. Haut Gauche (Top Left) ---
        if (active[0]) {
            if (tof_top_left->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[0] = 0; 
                if (NewDataReady != 0) { 
                    if (tof_top_left->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[0] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else {
                            distances[0] = OUT_OF_RANGE_MM; 
                        }
                        data_ready[0] = true;
                    }
                    tof_top_left->VL53LX_ClearInterruptAndStartMeasurement(); 
                }
            } else if (++error_count[0] > 5) {
                active[0] = false; data_ready[0] = false;
                Serial.println("\n⚠️ ALARME : ToF Haut Gauche I2C Perdu !");
            }
        }

        // --- 2. Haut Droit (Top Right) ---
        if (active[1]) {
            if (tof_top_right->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[1] = 0;
                if (NewDataReady != 0) {
                    if (tof_top_right->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[1] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else distances[1] = OUT_OF_RANGE_MM;
                        data_ready[1] = true;
                    }
                    tof_top_right->VL53LX_ClearInterruptAndStartMeasurement();
                }
            } else if (++error_count[1] > 5) {
                active[1] = false; data_ready[1] = false;
                Serial.println("\n⚠️ ALARME : ToF Haut Droit I2C Perdu !");
            }
        }

        // --- 3. Bas Gauche (Bottom Left) ---
        if (active[2]) {
            if (tof_bottom_left->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[2] = 0;
                if (NewDataReady != 0) {
                    if (tof_bottom_left->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[2] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else distances[2] = OUT_OF_RANGE_MM;
                        data_ready[2] = true;
                    }
                    tof_bottom_left->VL53LX_ClearInterruptAndStartMeasurement();
                }
            } else if (++error_count[2] > 5) {
                active[2] = false; data_ready[2] = false;
                Serial.println("\n⚠️ ALARME : ToF Bas Gauche I2C Perdu !");
            }
        }

        // --- 4. Bas Droit (Bottom Right) ---
        if (active[3]) {
            if (tof_bottom_right->VL53LX_GetMeasurementDataReady(&NewDataReady) == VL53LX_ERROR_NONE) {
                error_count[3] = 0;
                if (NewDataReady != 0) {
                    if (tof_bottom_right->VL53LX_GetMultiRangingData(&MultiRangingData) == VL53LX_ERROR_NONE) {
                        if (MultiRangingData.NumberOfObjectsFound > 0) {
                            distances[3] = MultiRangingData.RangeData[0].RangeMilliMeter;
                        } else distances[3] = OUT_OF_RANGE_MM;
                        data_ready[3] = true;
                    }
                    tof_bottom_right->VL53LX_ClearInterruptAndStartMeasurement();
                }
            } else if (++error_count[3] > 5) {
                active[3] = false; data_ready[3] = false;
                Serial.println("\n⚠️ ALARME : ToF Bas Droit I2C Perdu !");
            }
        }
    }

    void printStatus() {
        Serial.println("=== ÉTAT NOMINAL CAPTEURS ToF ===");
        Serial.printf("Haut Gauche: %d mm (%s)\n", distances[0], data_ready[0] ? "OK" : (active[0] ? "Wait" : "OFF"));
        Serial.printf("Haut Droit:  %d mm (%s)\n", distances[1], data_ready[1] ? "OK" : (active[1] ? "Wait" : "OFF"));
        Serial.printf("Bas Gauche:  %d mm (%s)\n", distances[2], data_ready[2] ? "OK" : (active[2] ? "Wait" : "OFF"));
        Serial.printf("Bas Droit:   %d mm (%s)\n", distances[3], data_ready[3] ? "OK" : (active[3] ? "Wait" : "OFF"));
    }

    // Accesseurs mis à jour pour la clarté du code externe
    uint16_t getTopLeftDistance() const { return distances[0]; }
    uint16_t getTopRightDistance() const { return distances[1]; }
    uint16_t getBottomLeftDistance() const { return distances[2]; }
    uint16_t getBottomRightDistance() const { return distances[3]; }

    // Arrêt d'urgence si un obstacle est trop près
    bool emergencyStopRequired() const {
        for (int i = 0; i < 4; i++) {
            if (data_ready[i] && distances[i] < CRITICAL_DISTANCE_MM) {
                return true;
            }
        }
        return false;
    }

    // Le "Cerveau" d'analyse de forme de l'obstacle
    ObstacleSignature getObstacleSignature() {
        // Conversion immédiate en mètres pour les calculs de seuils
        float dist_TL = getTopLeftDistance() / 1000.0; 
        float dist_TR = getTopRightDistance() / 1000.0;
        float dist_BL = getBottomLeftDistance() / 1000.0;
        float dist_BR = getBottomRightDistance() / 1000.0;

        float avg_top = (dist_TL + dist_TR) / 2.0;
        float avg_bot = (dist_BL + dist_BR) / 2.0;

        // Voie libre (> 1m devant)
        if (avg_bot > 1.0 && avg_top > 1.0) return OBSTACLE_NONE;

        // Mur Infranchissable (L'écart haut/bas est très faible, ex: < 10cm)
        if (abs(avg_top - avg_bot) < 0.10 && avg_bot < 0.80) return OBSTACLE_WALL;

        // Franchissable (Le bas détecte quelque chose à < 30cm, le haut voit loin)
        if (avg_bot < 0.30 && avg_top > 0.80) return OBSTACLE_PASSABLE;

        // Pente (Le bas se rapproche, le haut aussi mais reste plus loin)
        if (avg_bot < 0.80 && avg_top < 1.0 && avg_top > avg_bot) return OBSTACLE_SLOPE;

        return OBSTACLE_NONE;
    }

    // Fonction de compatibilité pour Navigation.h (Distance min à l'avant)
    float getMinFrontDistance() {
        // Retourne la plus petite distance sur les capteurs du bas (en mètres)
        float dist_BL = getBottomLeftDistance() / 1000.0;
        float dist_BR = getBottomRightDistance() / 1000.0;
        return min(dist_BL, dist_BR);
    }

    void printGridStatus() {
        char tl[16], tr[16], bl[16], br[16];

        // On utilise les vraies variables locales lissées "distances" purgées par l'I2C
        if (active[0] && data_ready[0]) sprintf(tl, "%4d mm", distances[0]);
        else strcpy(tl, "  ❌ ECHEC ");
        
        if (active[1] && data_ready[1]) sprintf(tr, "%4d mm", distances[1]);
        else strcpy(tr, "  ❌ ECHEC ");

        if (active[2] && data_ready[2]) sprintf(bl, "%4d mm", distances[2]);
        else strcpy(bl, "  ❌ ECHEC ");
        
        if (active[3] && data_ready[3]) sprintf(br, "%4d mm", distances[3]);
        else strcpy(br, "  ❌ ECHEC ");

        // Affichage graphique style Dashboard matriciel
        Serial.println("\n=====================================");
        Serial.printf("  [HAUT]  Gau: %s  |  Dro: %s  \n", tl, tr);
        Serial.println("  ---------------------------------  ");
        Serial.printf("  [BAS ]  Gau: %s  |  Dro: %s  \n", bl, br);
        Serial.println("=====================================");
    }
};