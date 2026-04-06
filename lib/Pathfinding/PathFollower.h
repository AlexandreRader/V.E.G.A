#pragma once
#include <Arduino.h>
#include "mission_export.h" // Le fichier généré par Python

class PathFollower {
private:
    int currentWaypointIndex = 0;
    bool missionComplete = false;
    float arrivalThreshold = 0.1; // 10cm de précision pour valider un point

public:
    void startMission();
    void update(float currentX, float currentY, float currentTheta);
    Waypoint getTarget();
    bool isDone() { return missionComplete; }
};