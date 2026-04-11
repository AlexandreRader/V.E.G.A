#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "AK09918.h"   // La boussole (Magnétomètre)
#include "ICM20600.h"  // Le Gyroscope/Accéléromètre



class IMUManager {
private:
    AK09918 ak09918;
    ICM20600 icm20600;
    
    float headingOffset = 0.0;

public:
    // Initialisation
    bool begin() {
        Wire.begin();
        
        // 1. Démarrer le couple Gyro/Accel
        icm20600.initialize();
        
        // 2. Démarrer la boussole
        AK09918_err_type_t err = ak09918.initialize();
        if (err != AK09918_ERR_OK) {
            Serial.println("ERREUR CRITIQUE : Boussole AK09918 non détectée !");
            Serial.println("Vérifiez le câblage I2C (SDA/SCL).");
            return false;
        }

        // Configuration de la boussole en mode continu (100Hz)
        ak09918.switchMode(AK09918_CONTINUOUS_100HZ);
        return true;
    }

    // Récupérer l'angle absolu (en radians) depuis le champ magnétique terrestre
    float getRawHeading() {
        int32_t x, y, z;
        
        // Vérifier si une nouvelle donnée magnétique est prête
        if (ak09918.isDataReady() == AK09918_ERR_OK) {
            ak09918.getData(&x, &y, &z);
            
            // Calcul de l'angle Yaw (Trigonométrie classique)
            // Note : Selon le montage du capteur sur le robot, vous devrez peut-être
            // inverser x et y, ou utiliser un signe négatif.
            return atan2((float)y, (float)x);
        }
        return 0.0; // Fallback si pas de données
    }

    // Calibrer l'angle de départ de la mission Python
    void setStartHeading(float mission_start_theta) {
        // Faire quelques lectures à vide pour stabiliser le capteur
        for(int i = 0; i < 10; i++) {
            getRawHeading();
            delay(10);
        }

        float real_magnetic_heading = getRawHeading();
        headingOffset = real_magnetic_heading - mission_start_theta;
        
        Serial.printf("Offset magnétique fixé : %.2f rad\n", headingOffset);
    }

    // Récupérer l'angle courant du robot (corrigé avec le offset)
    float getCurrentTheta() {
        return getRawHeading() - headingOffset;
    }
};