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
    // On utilise dy, dx dans l'ordre standard (Y en premier)
    float target_angle = atan2(dy, dx); 

    // ❌ ON SUPPRIME CETTE LIGNE QUI FAUSSAIT LA CIBLE DE 180° :
    // target_angle = M_PI / 2.0 - target_angle; 

    // Calcul de l'erreur d'angle brute
    float angle_error = target_angle - current_theta;

    // 🎯 NORMALISATION ANTI ZIG-ZAG
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // ... (Le reste du code reste identique avec la Deadband et le Debug) ...
    
    // === ASTUCE DE DÉBOGAGE ABSOLUE ===
    // Imprime ces valeurs pour voir si l'erreur d'angle tombe bien autour de zéro !
    // Serial.printf("Cible: %.1f° | Actuel: %.1f° | Erreur: %.1f°\n", 
    //               target_angle * 180/M_PI, current_theta * 180/M_PI, angle_error * 180/M_PI);
    // ==================================

    // 5. Générer les commandes de vitesse
    
    // Tolérance élargie à 3 degrés (~0.052 rad)
    float tolerance_angle = 0.052; 

    if (abs(angle_error) > 0.35) { 
    // Pivot sur place (Erreur > 20°)
    cmd.linear_v = 0.0; 
    
    // 🛑 ON BRIDE ICI : On passe de 0.8 à 0.25 pour que le robot tourne très lentement et s'arrête pile sur l'axe
    cmd.angular_w = constrain(-Kp_ANGULAR * angle_error, -0.25, 0.25);
}
    else {
        // Zone de validation
        if (abs(angle_error) <= tolerance_angle) {
            cmd.linear_v = TARGET_SPEED_MS; 
            cmd.angular_w = 0.0;            
        } 
        else {
            // Roulage et correction douce
            float speed_factor = cos(angle_error);
            cmd.linear_v = TARGET_SPEED_MS * speed_factor;
            
            // 🎯 ON INVERSE LE VOLANT ICI AUSSI (Ajout du signe -)
            cmd.angular_w = constrain(-Kp_ANGULAR * angle_error, -0.3, 0.3);
        }
    }

    // ==========================================
    // 🛑 RADAR DE DÉBOGAGE ABSOLU (À LAISSER POUR LE TEST)
    // ==========================================
    //Serial.printf("🎯 Cible: %.1f° | Err: %.1f° | Vitesse (v): %.2f | Angle (w): %.2f\n", 
     //             target_angle * (180.0 / M_PI), 
     //             angle_error * (180.0 / M_PI), 
      //            cmd.linear_v, 
      //           cmd.angular_w);
                  
    return cmd;
}