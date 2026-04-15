// ==========================================
// MISSION VEGA SC317 - TRAJECTOIRE A*
// ==========================================

#ifndef MISSION_EXPORT_H
#define MISSION_EXPORT_H

// --- Points clés de la mission ---
const float START_X = 0.300;
const float START_Y = 4.100;
const float START_THETA = 0.000; // Angle initial (radians)

const float GOAL_X = 0.800;
const float GOAL_Y = 1.100;

// --- Trajectoire ---
struct Waypoint {
    float x; // mètres
    float y; // mètres
};

const int PATH_SIZE = 25;
const Waypoint MISSION_PATH[PATH_SIZE] = {
    {0.300f, 4.000f},
    {0.500f, 4.000f},
    {0.800f, 4.000f},
    {1.100f, 4.200f},
    {1.300f, 4.300f},
    {1.600f, 4.300f},
    {1.900f, 4.200f},
    {2.100f, 4.100f},
    {2.400f, 4.000f},
    {2.700f, 3.900f},
    {3.000f, 3.700f},
    {3.100f, 3.500f},
    {3.100f, 3.200f},
    {3.200f, 2.900f},
    {3.100f, 2.600f},
    {2.800f, 2.400f},
    {2.600f, 2.200f},
    {2.300f, 2.000f},
    {2.100f, 2.000f},
    {1.800f, 2.100f},
    {1.500f, 2.100f},
    {1.300f, 1.900f},
    {1.000f, 1.800f},
    {0.800f, 1.600f},
    {0.700f, 1.300f},
};

#endif
