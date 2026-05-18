#include "PathFollower.h"
#include <math.h>
#include "config.h"

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

    // 3. Vérifier si on est arrivé assez près du point (Zone de tolérance)
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
    // On utilise dy, dx (au lieu de dx, dy) et on ajuste selon le sens des axes
    
    // NOUVEAU CALCUL DE L'ANGLE CIBLE (Pour s'aligner avec une boussole classique)
    float target_angle = atan2(dy, dx); 
    
    // Si ta boussole a le Nord à 0°, l'Est à 90°, le Sud à 180° :
    // L'axe X de la carte est l'Est, et l'axe Y est le Nord.
    // L'équation mathématique standard pour la navigation devient souvent celle-ci :
    target_angle = M_PI / 2.0 - target_angle; 
    
    // 🎯 CORRECTIF 1 : NORMALISATION ANTI ZIG-ZAG
    float angle_error = target_angle - current_theta;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;
    
    // === ASTUCE DE DÉBOGAGE ABSOLUE ===
    // Imprime ces valeurs pour voir si l'erreur d'angle tombe bien autour de zéro !
    // Serial.printf("Cible: %.1f° | Actuel: %.1f° | Erreur: %.1f°\n", 
    //               target_angle * 180/M_PI, current_theta * 180/M_PI, angle_error * 180/M_PI);
    // ==================================

    // 5. Générer les commandes de vitesse
    
    // Ton idée : Tolérance de validation à +/- 1 degré (~0.017 rad)
    float tolerance_angle = 0.017; 

    // Si on est à plus de 20° d'erreur (0.35 rad) : Stop & Turn pur
    if (abs(angle_error) > 0.35) { 
        cmd.linear_v = 0.0; 
        cmd.angular_w = constrain(Kp_ANGULAR * angle_error, -0.8, 0.8);
    } 
    // Sinon, on est dans le bon cône de direction...
    else {
        // 🎯 TA LOGIQUE DE VALIDATION :
        if (abs(angle_error) <= tolerance_angle) {
            // Le cap est PARFAIT (+/- 1°). On valide !
            cmd.linear_v = TARGET_SPEED_MS; // On roule à 0.05 m/s
            cmd.angular_w = 0.0; // 🔒 On fige les roues droites devant !
        } 
        else {
            // On est entre 1° et 20° : On avance tout en corrigeant la trajectoire en douceur
            float speed_factor = cos(angle_error);
            cmd.linear_v = TARGET_SPEED_MS * speed_factor;
            cmd.angular_w = constrain(Kp_ANGULAR * angle_error, -0.3, 0.3);
        }
    }

    return cmd;
}