#include "Kinematics.h"
#include <math.h>
#include "config.h"

Kinematics::Kinematics() {}
MotorCommands Kinematics::calculateDrive(float v, float w) {
    MotorCommands cmd;

    // Si aucune commande, tout est à l'arrêt
    if (abs(v) < 0.001 && abs(w) < 0.001) {
        cmd.angle_FL = cmd.angle_FR = cmd.angle_RL = cmd.angle_RR = 0;
        cmd.speed_FL = cmd.speed_ML = cmd.speed_RL = 0;
        cmd.speed_FR = cmd.speed_MR = cmd.speed_RR = 0;
        return cmd;
    }

    // 1. Calcul des composantes de vitesse vectorielle (Vx, Vy) pour chaque roue
    // On considère X pointant vers l'avant (L_AXE) et Y vers la gauche (W_VOIE/2)
    
    float vx_FL = v - w * (W_VOIE / 2.0);
    float vy_FL = w * L_AXE;
    
    float vx_FR = v - w * (-W_VOIE / 2.0);
    float vy_FR = w * L_AXE;
    
    float vx_RL = v - w * (W_VOIE / 2.0);
    float vy_RL = w * (-L_AXE);
    
    float vx_RR = v - w * (-W_VOIE / 2.0);
    float vy_RR = w * (-L_AXE);

    // Les roues du milieu n'ont pas de servo (pas de Vy), on garde juste Vx
    cmd.speed_ML = v - w * (W_VOIE / 2.0);
    cmd.speed_MR = v - w * (-W_VOIE / 2.0);

    // 2. Calcul des angles bruts (en radians)
    cmd.angle_FL = atan2(vy_FL, vx_FL);
    cmd.angle_FR = atan2(vy_FR, vx_FR);
    cmd.angle_RL = atan2(vy_RL, vx_RL);
    cmd.angle_RR = atan2(vy_RR, vx_RR);

    // 3. Calcul des vitesses brutes (Norme du vecteur)
    cmd.speed_FL = sqrt(vx_FL*vx_FL + vy_FL*vy_FL);
    cmd.speed_FR = sqrt(vx_FR*vx_FR + vy_FR*vy_FR);
    cmd.speed_RL = sqrt(vx_RL*vx_RL + vy_RL*vy_RL);
    cmd.speed_RR = sqrt(vx_RR*vx_RR + vy_RR*vy_RR);

    // 4. OPTIMISATION DE DIRECTION (Le correctif des "150 degrés")
    // Si l'angle demandé dépasse les 90° (PI/2), on le rabat de l'autre côté (-180°)
    // et on inverse la vitesse du moteur pour compenser.
    auto optimizeSteering = [](float &angle, float &speed) {
        if (angle > M_PI / 2.0) {
            angle -= M_PI;
            speed = -speed;
        } else if (angle < -M_PI / 2.0) {
            angle += M_PI;
            speed = -speed;
        }
    };

    optimizeSteering(cmd.angle_FL, cmd.speed_FL);
    optimizeSteering(cmd.angle_FR, cmd.speed_FR);
    optimizeSteering(cmd.angle_RL, cmd.speed_RL);
    optimizeSteering(cmd.angle_RR, cmd.speed_RR);

    return cmd;
}

float Kinematics::speedToStepsHz(float linear_speed) {
    // 1. Vitesse linéaire -> Vitesse angulaire de la roue (rad/s)
    float wheel_omega = linear_speed / WHEEL_RADIUS;
    
    // 2. Rad/s -> Tours par seconde
    float rps = wheel_omega / (2.0 * M_PI);
    
    // 3. Tours/s -> Pas par seconde (Hz)
    // Hz = Tours/s * (Pas par tour * Facteur Microstepping)
    float hz = rps * (STEPS_PER_REV * MICROSTEPPING);
    
    return hz; 
}