#include <Arduino.h>
#include "mission_export.h"
#include "PathFollower.h"
#include "Kinematics.h"

// --- Objets Globaux ---
PathFollower follower;
Kinematics kinematics;

// --- Variables d'état du robot ---
// Initialisées automatiquement avec les données générées par Python !
float robot_x = START_X; 
float robot_y = START_Y;
float robot_theta = START_THETA; 

void setup() {
    Serial.begin(115200);
    
    // Exemple : Initialisation de l'IMU
    // IMU.begin();
    // IMU.setOffset(START_THETA); // On dit à l'IMU : "Ton zéro actuel correspond à cet angle"
    
    Serial.printf("Boot VEGA SC317. Départ: [%.2f, %.2f] Cap: %.2f rad\n", 
                  robot_x, robot_y, robot_theta);
}

void loop() {
    // 1. Lire l'IMU pour mettre à jour robot_theta
    // 2. Mettre à jour robot_x et robot_y (Odometrie + IMU)
    
    if (!follower.isDone()) {
        VelocityCommand v_cmd = follower.update(robot_x, robot_y, robot_theta);
        MotorCommands m_cmds = kinematics.calculateDrive(v_cmd.linear_v, v_cmd.angular_w);
        
        // Appliquer aux moteurs...
    }
}