#pragma once
#include <Arduino.h>

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
    const float L_AXE = 0.20;      // Distance entre le centre et l'axe avant (m)
    const float W_VOIE = 0.30;     // Largeur totale entre roues gauche et droite (m)
    const float WHEEL_RADIUS = 0.05; // Rayon de tes roues (5cm = 0.05m)
    
    // --- Paramètres des Steppers 17HS15-0404S ---
    const float STEPS_PER_REV = 200.0; // Moteurs 1.8° par pas
    const int MICROSTEPPING = 16;      // Si tes drivers A4988 sont en 1/16

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