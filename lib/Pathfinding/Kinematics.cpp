#include "Kinematics.h"
#include <math.h>

Kinematics::Kinematics() {}

MotorCommands Kinematics::calculateDrive(float v, float w) {
    MotorCommands cmd;

    // 1. Cas de la ligne droite (pour éviter une division par zéro)
    if (abs(w) < 0.001) {
        cmd.angle_FL = cmd.angle_FR = cmd.angle_RL = cmd.angle_RR = 0;
        cmd.speed_FL = cmd.speed_ML = cmd.speed_RL = v;
        cmd.speed_FR = cmd.speed_MR = cmd.speed_RR = v;
        return cmd;
    }

    // 2. Calcul du Rayon de braquage global (ICR)
    // R est la distance entre le centre du robot et le point de pivot
    float R = v / w;

    // 3. Calcul des angles de braquage (Servos)
    // Formule : theta = atan(L / (R ± W/2))
    cmd.angle_FL = atan2(L_AXE, R - (W_VOIE / 2.0));
    cmd.angle_FR = atan2(L_AXE, R + (W_VOIE / 2.0));
    cmd.angle_RL = atan2(-L_AXE, R - (W_VOIE / 2.0)); // Arrière opposé
    cmd.angle_RR = atan2(-L_AXE, R + (W_VOIE / 2.0));

    // 4. Calcul des vitesses individuelles (Steppers)
    // Chaque roue a sa propre vitesse pour ne pas glisser
    // V_roue = w * Distance_Roue_vers_ICR
    
    // Roues de Gauche
    cmd.speed_FL = w * sqrt(pow(L_AXE, 2) + pow(R - (W_VOIE / 2.0), 2));
    cmd.speed_ML = w * (R - (W_VOIE / 2.0));
    cmd.speed_RL = cmd.speed_FL;

    // Roues de Droite
    cmd.speed_FR = w * sqrt(pow(L_AXE, 2) + pow(R + (W_VOIE / 2.0), 2));
    cmd.speed_MR = w * (R + (W_VOIE / 2.0));
    cmd.speed_RR = cmd.speed_FR;

    // Correction de signe : si on tourne sur place, les vitesses doivent suivre w
    if (v == 0) {
        // Logique spécifique au pivot 360° si nécessaire
    }

    return cmd;
}

float Kinematics::speedToStepsHz(float linear_speed) {
    // 1. Vitesse linéaire -> Vitesse angulaire de la roue (rad/s)
    float wheel_omega = abs(linear_speed) / WHEEL_RADIUS;
    
    // 2. Rad/s -> Tours par seconde
    float rps = wheel_omega / (2.0 * M_PI);
    
    // 3. Tours/s -> Pas par seconde (Hz)
    // Hz = Tours/s * (Pas par tour * Facteur Microstepping)
    float hz = rps * (STEPS_PER_REV * MICROSTEPPING);
    
    return hz; 
}