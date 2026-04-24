#pragma once

struct Waypoint {
    float x;
    float y;
};

const int MAX_WAYPOINTS = 50;

extern int PATH_SIZE;
extern Waypoint MISSION_PATH[MAX_WAYPOINTS];
extern bool mission_ready_to_start;

// --- NOUVEAUX PARAMÈTRES GLOBAUX ---
extern float START_X;
extern float START_Y;
extern float START_THETA;
extern float GOAL_X;
extern float GOAL_Y;
extern float GOAL_THETA;