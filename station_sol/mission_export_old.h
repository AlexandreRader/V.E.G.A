// ==========================================
// MISSION VEGA SC317 - TRAJECTOIRE A*
// ==========================================

#ifndef MISSION_EXPORT_H
#define MISSION_EXPORT_H

// --- Points clés de la mission ---
const float START_X = 3.500;
const float START_Y = 5.800;
const float START_THETA = -1.571; // Angle initial (radians)

const float GOAL_X = 3.500;
const float GOAL_Y = 0.900;

// --- Trajectoire ---
struct Waypoint {
    float x; // mètres
    float y; // mètres
};

const int PATH_SIZE = 50;
const Waypoint MISSION_PATH[PATH_SIZE] = {
    {3.500f, 5.800f},
    {3.500f, 5.700f},
    {3.600f, 5.600f},
    {3.600f, 5.500f},
    {3.700f, 5.400f},
    {3.700f, 5.300f},
    {3.800f, 5.200f},
    {3.800f, 5.100f},
    {3.900f, 5.000f},
    {3.900f, 4.900f},
    {3.900f, 4.800f},
    {3.900f, 4.700f},
    {3.900f, 4.600f},
    {3.900f, 4.500f},
    {3.900f, 4.400f},
    {3.900f, 4.300f},
    {3.900f, 4.200f},
    {3.900f, 4.100f},
    {3.900f, 4.000f},
    {3.900f, 3.900f},
    {3.900f, 3.800f},
    {3.900f, 3.700f},
    {3.900f, 3.600f},
    {3.900f, 3.500f},
    {3.900f, 3.400f},
    {3.900f, 3.300f},
    {3.900f, 3.200f},
    {3.900f, 3.100f},
    {3.900f, 3.000f},
    {3.800f, 2.900f},
    {3.800f, 2.800f},
    {3.800f, 2.700f},
    {3.700f, 2.600f},
    {3.600f, 2.500f},
    {3.500f, 2.400f},
    {3.500f, 2.300f},
    {3.500f, 2.200f},
    {3.500f, 2.100f},
    {3.500f, 2.000f},
    {3.500f, 1.900f},
    {3.500f, 1.800f},
    {3.500f, 1.700f},
    {3.500f, 1.600f},
    {3.500f, 1.500f},
    {3.500f, 1.400f},
    {3.500f, 1.300f},
    {3.500f, 1.200f},
    {3.500f, 1.100f},
    {3.500f, 1.000f},
    {3.500f, 0.900f},
};

#endif
