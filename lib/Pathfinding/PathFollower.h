#pragma once
#include <Arduino.h>
#include "mission_export.h" // Le fichier généré par votre station sol Python

// Structure pour renvoyer la consigne globale du robot
struct VelocityCommand {
    float linear_v;  // Vitesse d'avancement (m/s)
    float angular_w; // Vitesse de rotation (rad/s)
};

class PathFollower {
private:
    int currentIndex;
    bool missionComplete;

    // --- Paramètres de Navigation ---
    // Rayon de validation d'un point (ici 15 cm)
    const float ARRIVAL_THRESHOLD = 0.15; 
    
    // Vitesse de croisière du rover (m/s)
    const float BASE_SPEED = 0.3; 
    
    // Gain de direction (plus il est élevé, plus le robot braque fort vers la cible)
    const float Kp_ANGULAR = 1.2; 

public:
    PathFollower();
    
    void resetMission();
    bool isDone() const;

    /**
     * Calcule les vitesses V et W en fonction de la position actuelle.
     * @param current_x Position X actuelle du robot (en mètres)
     * @param current_y Position Y actuelle du robot (en mètres)
     * @param current_theta Orientation actuelle du robot (en radians)
     */
    VelocityCommand update(float current_x, float current_y, float current_theta);
};