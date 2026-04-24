#pragma once
#include <Arduino.h>
#include "../../V.E.G.A/lib/Communication/mission.h"
#include "../../include/config.h"

// Structure pour renvoyer la consigne globale du robot
struct VelocityCommand {
    float linear_v;  // Vitesse d'avancement (m/s)
    float angular_w; // Vitesse de rotation (rad/s)
};

class PathFollower {
private:
    int currentIndex;
    bool missionComplete;


public:
    PathFollower();
    
    void resetMission();
    bool isDone() const;
    int getCurrentIndex() const { return currentIndex; }

    /**
     * Calcule les vitesses V et W en fonction de la position actuelle.
     * @param current_x Position X actuelle du robot (en mètres)
     * @param current_y Position Y actuelle du robot (en mètres)
     * @param current_theta Orientation actuelle du robot (en radians)
     */
    VelocityCommand update(float current_x, float current_y, float current_theta);
};