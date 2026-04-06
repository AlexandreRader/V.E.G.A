#include "PathFollower.h"
#include <math.h>

PathFollower::PathFollower() {
    resetMission();
}

void PathFollower::resetMission() {
    currentIndex = 0;
    missionComplete = false;
}

bool PathFollower::isDone() const {
    return missionComplete;
}

VelocityCommand PathFollower::update(float current_x, float current_y, float current_theta) {
    VelocityCommand cmd = {0.0, 0.0};

    // Sécurité : Si la mission est finie, on demande l'arrêt complet
    if (missionComplete || currentIndex >= PATH_SIZE) {
        missionComplete = true;
        return cmd; 
    }

    // 1. Lire la cible actuelle depuis le fichier généré
    Waypoint target = MISSION_PATH[currentIndex];

    // 2. Calculer la distance restante vers cette cible
    float dx = target.x - current_x;
    float dy = target.y - current_y;
    float distance = sqrt(dx * dx + dy * dy);

    // 3. Vérifier si on est arrivé assez près du point
    if (distance < ARRIVAL_THRESHOLD) {
        currentIndex++; // On passe au point suivant
        
        // Si c'était le dernier point, la mission est terminée
        if (currentIndex >= PATH_SIZE) {
            missionComplete = true;
            return cmd; 
        }
        
        // Mettre à jour les calculs pour le nouveau point
        target = MISSION_PATH[currentIndex];
        dx = target.x - current_x;
        dy = target.y - current_y;
    }

    // 4. Calculer l'erreur de cap (Heading Error)
    float target_angle = atan2(dy, dx); // Angle absolu vers la cible
    float angle_error = target_angle - current_theta;

    // Normaliser l'erreur pour qu'elle reste entre -PI et +PI
    // (Pour que le robot tourne du côté le plus court)
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // 5. Générer les commandes de vitesse
    cmd.linear_v = BASE_SPEED;
    
    // Si le robot doit tourner brutalement (erreur > 30 degrés), on le fait ralentir
    if (abs(angle_error) > 0.5) {
        cmd.linear_v = BASE_SPEED * 0.5; 
    }

    // La vitesse de rotation est proportionnelle à l'erreur d'angle
    cmd.angular_w = Kp_ANGULAR * angle_error;

    return cmd;
}