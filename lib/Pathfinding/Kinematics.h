#pragma once
#include <Arduino.h>
#include "../../include/config.h"

// Structure contenant les ordres pour les 10 actionneurs (4 servos, 6 moteurs)
struct MotorCommands {
    // Angles des servomoteurs (en radians)
    float angle_FL, angle_FR, angle_RL, angle_RR;
    // Vitesses linéaires des roues (en m/s)
    float speed_FL, speed_ML, speed_RL;
    float speed_FR, speed_MR, speed_RR;
};

class Kinematics {
private:
    // --- Dimensions du Rover (à ajuster selon ton châssis réel) ---
    

public:
    Kinematics();

    /**
     * Calcule la cinématique inverse ICR pour 6 roues
     * @param v Vitesse linéaire souhaitée (m/s)
     * @param w Vitesse angulaire souhaitée (rad/s)
     */
    MotorCommands calculateDrive(float v, float w);

    /**
     * Convertit une vitesse m/s en fréquence d'impulsions (Hz) pour le A4988
     */
    float speedToStepsHz(float linear_speed);
};