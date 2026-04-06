#include "Kinematics.h"
#include <math.h>

MotorCommands Kinematics::calculateDrive(float linear_v, float angular_w) {
    MotorCommands cmd;

    // ---------------------------------------------------------
    // CAS 1 : LIGNE DROITE (ou quasi-ligne droite)
    // Si la rotation demandée est infime, on force le tout droit
    // ---------------------------------------------------------
    if (abs(angular_w) < 0.001) {
        // Servos à 0 (alignés)
        cmd.angle_FL = cmd.angle_FR = cmd.angle_RL = cmd.angle_RR = 0.0;
        
        // Toutes les roues tournent à la même vitesse
        cmd.speed_FL = cmd.speed_ML = cmd.speed_RL = linear_v;
        cmd.speed_FR = cmd.speed_MR = cmd.speed_RR = linear_v;
        
        return cmd;
    }

    // ---------------------------------------------------------
    // CAS 2 : ROTATION SUR PLACE (Spin in place)
    // La vitesse linéaire est nulle, on ne fait que pivoter
    // ---------------------------------------------------------
    if (abs(linear_v) < 0.001) {
        // Les 4 roues forment un cercle parfait autour du centre du robot
        cmd.angle_FL = atan2(L, -W);
        cmd.angle_FR = atan2(L, W);
        cmd.angle_RL = atan2(-L, -W);
        cmd.angle_RR = atan2(-L, W);

        // Les vitesses dépendent de la distance de la roue par rapport au centre
        float corner_speed = angular_w * sqrt(pow(W, 2) + pow(L, 2));
        float middle_speed = angular_w * W;

        cmd.speed_FL = -corner_speed; // Les roues gauches reculent (ou avancent selon le sens)
        cmd.speed_ML = -middle_speed;
        cmd.speed_RL = -corner_speed;

        cmd.speed_FR = corner_speed;  // Les roues droites font l'inverse
        cmd.speed_MR = middle_speed;
        cmd.speed_RR = corner_speed;

        return cmd;
    }

    // ---------------------------------------------------------
    // CAS 3 : VIRAGE CLASSIQUE (Ackermann 4WS)
    // ---------------------------------------------------------
    // Rayon de braquage (Distance du centre du robot au Centre Instantané de Rotation)
    float R = linear_v / angular_w; 

    // 1. Calcul des angles des 4 servos (en radians)
    // L'utilisation de atan2 gère automatiquement le signe selon si R est positif (gauche) ou négatif (droite)
    cmd.angle_FL = atan2(L, R - W);
    cmd.angle_FR = atan2(L, R + W);
    cmd.angle_RL = atan2(-L, R - W); // L est négatif car c'est l'essieu arrière
    cmd.angle_RR = atan2(-L, R + W);

    // 2. Calcul des vitesses des 6 roues
    // La vitesse d'une roue est proportionnelle à sa distance par rapport au CIR.
    cmd.speed_FL = angular_w * sqrt(pow(R - W, 2) + pow(L, 2));
    cmd.speed_FR = angular_w * sqrt(pow(R + W, 2) + pow(L, 2));
    
    cmd.speed_ML = angular_w * (R - W); // Les roues du milieu n'ont pas de décalage L
    cmd.speed_MR = angular_w * (R + W);

    cmd.speed_RL = angular_w * sqrt(pow(R - W, 2) + pow(-L, 2));
    cmd.speed_RR = angular_w * sqrt(pow(R + W, 2) + pow(-L, 2));

    return cmd;
}

float Kinematics::speedToStepsHz(float speed_m_s, int steps_per_rev, int microstepping) {
    // Circonférence de la roue : C = 2 * PI * R
    float circumference = 2.0 * PI * WHEEL_RADIUS;
    
    // Tours par seconde nécessaires
    float revs_per_sec = speed_m_s / circumference;
    
    // Impulsions par seconde (Fréquence en Hz pour le A4988)
    return revs_per_sec * steps_per_rev * microstepping;
}