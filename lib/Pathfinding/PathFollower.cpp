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
    float target_angle = atan2(dy, dx); 
    float angle_error = target_angle - current_theta;

    // Normalisation anti-enroulement entre -PI et PI
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // 🔍 DEBUG: Affichage complet de l'état de navigation
    static unsigned long last_debug_print = 0;
    if (millis() - last_debug_print > 500) {
        Serial.printf("📊 [PF] WP[%d]: (%.2f,%.2f) | Dist:%.2f | TargetAng:%.1f° | CurrHeading:%.1f° | Error:%+.1f°\n",
                      currentIndex, target.x, target.y, distance,
                      target_angle * 180.0 / M_PI, 
                      current_theta * 180.0 / M_PI,
                      angle_error * 180.0 / M_PI);
        last_debug_print = millis();
    }

    float tolerance_angle = 0.1; // Tolérance en ligne droite (~3 degrés)

    // ==========================================================
    // 🎯 AUTOMATE À ÉTATS AVEC HYSTÉRÉSIS (Anti-tremblement)
    // ==========================================================
    static bool in_pivot_mode = false;
    static unsigned long pivot_start_time = 0; // Timeout pour éviter de rester bloqué
    const unsigned long PIVOT_TIMEOUT_MS = 3000; // 3 secondes max en pivot

    if (in_pivot_mode) {
        // On reste en pivot sur place tant qu'on n'est pas aligné à moins de 5° (0.087 rad)
        if (abs(angle_error) < 0.087) {
            in_pivot_mode = false;
            Serial.println("🎯 [PathFollower] Cap aligné ! Passage en mode roulage.");
        }
        // ESCAPE: Si on a pivoté pendant 3 sec sans se stabiliser, force passage en roulage
        else if (millis() - pivot_start_time > PIVOT_TIMEOUT_MS) {
            in_pivot_mode = false;
            Serial.printf("⚠️ [PathFollower] TIMEOUT PIVOT ! Erreur: %.1f°. Passage forcé en roulage.\n", 
                         angle_error * 180.0 / M_PI);
        }
    } else {
        // On ne déclenche un pivot sur place que si l'erreur dépasse 35° (0.61 rad)
        if (abs(angle_error) > 0.61) {
            in_pivot_mode = true;
            pivot_start_time = millis(); // Enregistre le début du pivot
            Serial.println("🔄 [PathFollower] Écart important ! Déclenchement Pivot sur place.");
        }
    }

    // ==========================================================
    // GÉNÉRATION DES COMMANDES SELON LE MODE STABILISÉ
    // ==========================================================
    if (in_pivot_mode) {
        // Pivot pur : Vitesse d'avance nulle pour laisser travailler la direction
        cmd.linear_v = 0.0; 
        cmd.angular_w = constrain(Kp_ANGULAR * angle_error, -0.5, 0.5);
        // Debug: Affiche une fois par seconde
        static unsigned long last_mode_print = 0;
        if (millis() - last_mode_print > 1000) {
            Serial.printf("🔄 [PF MODE] PIVOT | AngularW: %.3f rad/s\n", cmd.angular_w);
            last_mode_print = millis();
        }
    } 
    else {
        // Mode Roulage : Le robot est globalement dans le bon axe
        if (abs(angle_error) <= tolerance_angle) {
            // Ligne droite parfaite
            cmd.linear_v = TARGET_SPEED_MS; 
            cmd.angular_w = 0.0;
            static unsigned long last_straight_print = 0;
            if (millis() - last_straight_print > 1000) {
                Serial.printf("➡️  [PF MODE] STRAIGHT | V: %.2f m/s\n", cmd.linear_v);
                last_straight_print = millis();
            }
        } 
        else {
            // Correction douce en roulant
            float speed_factor = cos(angle_error);
            // 🔴 SÉCURITÉ: Ne JAMAIS faire marcher le robot en arrière à cause de l'angle
            // Si speed_factor < 0, cela veut dire qu'on essaie de tourner > 90°
            // Dans ce cas, on remet à zéro le mode et on force un pivot
            if (speed_factor < 0.0) {
                in_pivot_mode = true;
                pivot_start_time = millis();
                cmd.linear_v = 0.0;
                cmd.angular_w = constrain(Kp_ANGULAR * angle_error, -0.5, 0.5);
                Serial.printf("🔴 [PF] SAFEGUARD: cos(angle_error)=%.2f < 0! Force PIVOT mode\n", speed_factor);
            } else {
                cmd.linear_v = TARGET_SPEED_MS * speed_factor;
                cmd.angular_w = constrain(Kp_ANGULAR * angle_error, -0.25, 0.25);
                static unsigned long last_curve_print = 0;
                if (millis() - last_curve_print > 1000) {
                    Serial.printf("↗️  [PF MODE] CURVE | V: %.2f m/s | W: %.3f rad/s | Factor: %.2f\n", 
                                  cmd.linear_v, cmd.angular_w, speed_factor);
                    last_curve_print = millis();
                }
            }
        }
    }
                  
    return cmd;
}