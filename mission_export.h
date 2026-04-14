// ==========================================
// MISSION VEGA SC317 - TRAJECTOIRE A*
// ==========================================

#ifndef MISSION_EXPORT_H
#define MISSION_EXPORT_H

// --- Points clés de la mission ---
const float START_X = 0.600;
const float START_Y = 5.600;
const float START_THETA = -0.785; // Angle initial (radians)

const float GOAL_X = 3.000;
const float GOAL_Y = 1.300;

// --- Trajectoire ---
struct Waypoint {
    float x; // mètres
    float y; // mètres
};

const int PATH_SIZE = 20;
const Waypoint MISSION_PATH[PATH_SIZE] = {
    {0.600f, 5.600f},
    {0.800f, 5.400f},
    {0.900f, 5.100f},
    {0.900f, 4.800f},
    {0.900f, 4.500f},
    {0.900f, 4.200f},
    {0.900f, 3.900f},
    {1.000f, 3.600f},
    {1.000f, 3.300f},
    {1.000f, 3.000f},
    {1.000f, 2.700f},
    {1.000f, 2.400f},
    {1.000f, 2.100f},
    {1.200f, 1.900f},
    {1.400f, 1.700f},
    {1.700f, 1.600f},
    {2.000f, 1.500f},
    {2.200f, 1.300f},
    {2.500f, 1.300f},
    {2.800f, 1.300f},
};

#endif
