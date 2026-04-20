#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "AK09918.h"   // TA bibliothèque réparée
#include "ICM20600.h"  // TA bibliothèque réparée

class IMUManager {
private:
    AK09918 ak09918;
    ICM20600 icm20600{true}; // Force l'adresse 0x69
    float headingOffset = 0.0;

public:
    // Variables PHYSIQUES prêtes à l'emploi
    float accX, accY, accZ;       // en m/s²
    float gyroX, gyroY, gyroZ;    // en °/s

    bool begin() {
        Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);

        // 1. Initialisation via la bibliothèque (Gère le bypass toute seule)
        icm20600.initialize();

        // 2. Initialisation de la boussole via la bibliothèque
        if (ak09918.initialize() != AK09918_ERR_OK) {
            return false;
        }
        ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

        return true;
    }

    // --- Lecture via la bibliothèque ---
    bool readMotion() {
        // On récupère les valeurs brutes directement avec les fonctions de Seeed Studio
        int16_t ax = icm20600.getAccelerationX();
        int16_t ay = icm20600.getAccelerationY();
        int16_t az = icm20600.getAccelerationZ();

        int16_t gx = icm20600.getGyroscopeX();
        int16_t gy = icm20600.getGyroscopeY();
        int16_t gz = icm20600.getGyroscopeZ();

        // Conversion avec les diviseurs de la bibliothèque (16G et 2000 dps)
        accX = (ax / 2048.0) * 9.81;
        accY = (ay / 2048.0) * 9.81;
        accZ = (az / 2048.0) * 9.81;
        
        gyroX = gx / 16.4;
        gyroY = gy / 16.4;
        gyroZ = gz / 16.4;

        return true;
    }

    // --- Boussole via la bibliothèque (Celle qui marchait !) ---
    float getRawHeading() {
        int32_t x, y, z;
        
        // La fonction isDataReady de la librairie est ultra fiable
        if (ak09918.isDataReady() == AK09918_ERR_OK) {
            ak09918.getData(&x, &y, &z);
            
            float angle = atan2((float)y, (float)x);
            if (angle < 0) angle += 2 * PI;
            return angle;
        }
        return -1.0; // Retourne -1 si la donnée n'est pas encore prête
    }

    void setStartHeading(float mission_start_theta) {
        float real_magnetic_heading = getRawHeading();
        if (real_magnetic_heading >= 0) {
            headingOffset = real_magnetic_heading - mission_start_theta;
        }
    }

    float getCurrentTheta() {
        float raw = getRawHeading();
        if (raw < 0) return 0.0; 
        float current = raw - headingOffset;
        while (current < 0) current += 2 * PI;
        while (current >= 2 * PI) current -= 2 * PI;
        return current;
    }
};