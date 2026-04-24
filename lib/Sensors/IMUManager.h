#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "AK09918.h"
#include "ICM20600.h"
#include "config.h"

class IMUManager {
private:
    AK09918 ak09918;
    ICM20600 icm20600;
    
    // Offset calculé au démarrage pour le gyroscope
    float gyro_offset_z;

public:
    // Variables brutes
    int16_t acc_x, acc_y, acc_z;
    int32_t mag_x, mag_y, mag_z;

    // --- VARIABLES CALCULÉES RÉINTÉGRÉES POUR LE MENU ---
    float accX, accY, accZ;       // Accélérations en m/s^2
    float gyroX, gyroY, gyroZ;    // Vitesses angulaires en rad/s
    float roll, pitch, heading;   // Orientations en Radians

    IMUManager() : icm20600(true), gyro_offset_z(0) {}

    bool begin() {
        Serial.println("Initialisation IMU (ICM20600 + AK09918)...");
        
        icm20600.initialize();
        if (ak09918.initialize() != AK09918_ERR_OK) {
            Serial.println("❌ Erreur AK09918 (Magnétomètre) !");
            return false;
        }

        ak09918.switchMode(AK09918_POWER_DOWN);
        ak09918.switchMode(AK09918_CONTINUOUS_100HZ);

        // Attente que le capteur soit prêt
        while (ak09918.isDataReady() != AK09918_ERR_OK) {
            delay(10);
        }

        // --- CALIBRATION STATIQUE DU GYROSCOPE ---
        Serial.print("Calibration du Gyroscope (Ne touchez pas le robot)... ");
        long sumZ = 0;
        for (int i = 0; i < 200; i++) {
            sumZ += icm20600.getGyroscopeZ();
            delay(10);
        }
        gyro_offset_z = sumZ / 200.0;
        Serial.println("✅ OK");

        return true;
    }

    void readMotion() {
        // Lecture Accéléromètre (La librairie renvoie des milli-g !)
        acc_x = icm20600.getAccelerationX();
        acc_y = icm20600.getAccelerationY();
        acc_z = icm20600.getAccelerationZ();

        // Conversion mg -> m/s² (1000 mg = 1 g = 9.81 m/s²)
        accX = (float)acc_x * (9.81 / 1000.0);
        accY = (float)acc_y * (9.81 / 1000.0);
        accZ = (float)acc_z * (9.81 / 1000.0);

        // Lecture Gyroscope (La librairie renvoie déjà des dps !)
        float raw_gyroX = icm20600.getGyroscopeX(); 
        float raw_gyroY = icm20600.getGyroscopeY(); 
        float raw_gyroZ = icm20600.getGyroscopeZ() - gyro_offset_z;

        // Conversion dps -> rad/s
        gyroX = raw_gyroX * (M_PI / 180.0); 
        gyroY = raw_gyroY * (M_PI / 180.0); 
        gyroZ = raw_gyroZ * (M_PI / 180.0); 

        // Lecture Magnétomètre
        ak09918.getData(&mag_x, &mag_y, &mag_z);
        mag_x -= MAG_OFFSET_X;
        mag_y -= MAG_OFFSET_Y;
        mag_z -= MAG_OFFSET_Z;
    }

    void updateEulerAngles() {
        // Calcul du Roll et Pitch avec l'accéléromètre
        roll = atan2((float)acc_y, (float)acc_z);
        pitch = atan2(-(float)acc_x, sqrt((float)acc_y * acc_y + (float)acc_z * acc_z));

        // Compensation du Tilt pour la boussole
        double Xheading = mag_x * cos(pitch) + mag_y * sin(roll) * sin(pitch) + mag_z * cos(roll) * sin(pitch);
        double Yheading = mag_y * cos(roll) - mag_z * sin(pitch);
        
        // Calcul du cap en degrés
        double heading_deg = 180.0 + (57.2957 * atan2(Yheading, Xheading)) + MAGNETIC_DECLINATION;
        
        // Normalisation entre 0 et 360
        if (heading_deg >= 360.0) heading_deg -= 360.0;
        if (heading_deg < 0.0) heading_deg += 360.0;

        // Conversion en radians pour le filtre de Kalman
        heading = heading_deg * (M_PI / 180.0);
    }

    // --- FONCTION DE CALIBRATION MANUELLE (Magnétomètre) ---
    void calibrateMagnetometer() {
        Serial.println("\n=== CALIBRATION DE LA BOUSSOLE ===");
        Serial.println("Prenez le rover dans vos mains.");
        Serial.println("Dans 3 secondes, faites des mouvements en forme de '8' dans toutes les directions !");
        delay(3000);
        Serial.print("Calibration en cours (10 secondes)...");

        int32_t val_x, val_y, val_z;
        int32_t min_x = 32767, max_x = -32768;
        int32_t min_y = 32767, max_y = -32768;
        int32_t min_z = 32767, max_z = -32768;

        uint32_t startTime = millis();
        while ((millis() - startTime) < 10000) {
            ak09918.getData(&val_x, &val_y, &val_z);
            
            if (val_x < min_x) min_x = val_x;
            if (val_x > max_x) max_x = val_x;
            if (val_y < min_y) min_y = val_y;
            if (val_y > max_y) max_y = val_y;
            if (val_z < min_z) min_z = val_z;
            if (val_z > max_z) max_z = val_z;

            Serial.print(".");
            delay(50);
        }

        int32_t off_x = min_x + (max_x - min_x) / 2;
        int32_t off_y = min_y + (max_y - min_y) / 2;
        int32_t off_z = min_z + (max_z - min_z) / 2;

        Serial.println("\n\n✅ Calibration Terminée !");
        Serial.println("COPIEZ-COLLEZ CES LIGNES DANS VOTRE config.h :");
        Serial.println("-------------------------------------------------");
        Serial.printf("const int32_t MAG_OFFSET_X = %ld;\n", off_x);
        Serial.printf("const int32_t MAG_OFFSET_Y = %ld;\n", off_y);
        Serial.printf("const int32_t MAG_OFFSET_Z = %ld;\n", off_z);
        Serial.println("-------------------------------------------------");
    }
};