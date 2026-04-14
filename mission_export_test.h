// Test mission for Wokwi simulation
// Simple rectangular path: (0,0) -> (2,0) -> (2,3) -> (0,3) -> (0,0)

#ifndef MISSION_EXPORT_H
#define MISSION_EXPORT_H

// Mission parameters
const float START_X = 0.0;
const float START_Y = 0.0;
const float START_THETA = 0.0; // Facing positive X

const float GOAL_X = 0.0;
const float GOAL_Y = 0.0; // Return to start

// Waypoint structure
struct Waypoint {
    float x;
    float y;
};

// Mission path (5 waypoints forming a rectangle)
const int PATH_SIZE = 5;
const Waypoint MISSION_PATH[PATH_SIZE] = {
    {0.0, 0.0},   // Start
    {2.0, 0.0},   // Move right
    {2.0, 3.0},   // Move up
    {0.0, 3.0},   // Move left
    {0.0, 0.0}    // Return to start
};

#endif