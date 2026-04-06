#pragma once

struct WheelSpeeds {
    float left;  // Vitesse en pas/seconde ou mm/s
    float right;
};

class Kinematics {
public:
    // Calcule la vitesse différentielle pour atteindre un point
    static WheelSpeeds calculateDrive(float curX, float curY, float curTheta, Waypoint target);
    
    // Transforme la vitesse mm/s en fréquence d'impulsion pour les A4988
    static int velocityToSteps(float linearVel);
};