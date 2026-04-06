// MISSION VEGA SC317 - TRAJECTOIRE A*
#ifndef MISSION_EXPORT_H
#define MISSION_EXPORT_H

struct Waypoint {
    float x; // mètres
    float y; // mètres
};

const int PATH_SIZE = 9;
const Waypoint MISSION_PATH[PATH_SIZE] = {
    {0.40f, 5.40f},
    {0.50f, 5.50f},
    {0.60f, 5.60f},
    {0.70f, 5.70f},
    {0.80f, 5.80f},
    {0.90f, 5.90f},
    {1.00f, 5.90f},
    {1.10f, 5.90f},
    {1.20f, 5.90f},
};

#endif