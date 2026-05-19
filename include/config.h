#pragma once
#include <math.h>

// ==========================================
// CONFIGURATION GLOBALE - VEGA SC317
// ==========================================

// --- Dimensions du Rover (en mètres) ---

const float L_AXE = 0.115;            // Distance entre le centre et l'axe avant
const float W_VOIE = 0.30;           // Largeur totale entre roues gauche et droite
const float WHEEL_RADIUS = 0.042; 

//const float L_AXE = 0.101;            // Distance entre le centre et l'axe avant
//const float W_VOIE = 0.35;           // Largeur totale entre roues gauche et droite
//const float WHEEL_RADIUS = 0.045;     // Rayon des roues (5 cm)

// --- Paramètres des Moteurs (Steppers) ---
const float STEPS_PER_REV = 200.0;   // Moteurs 1.8° par pas
const int MICROSTEPPING = 16;        // Drivers configurés en 1/16
const float MAX_SPEED_HZ = 1500.0;  // Limite de sécurité logicielle

// --- Constantes Calculées ---
const float METERS_PER_STEP = (2.0 * M_PI * WHEEL_RADIUS) / (STEPS_PER_REV * MICROSTEPPING);

// --- Paramètres de Vitesse et Navigation ---

// ⚠️ LA VITESSE CIBLE GLOBALE DU ROVER (m/s) ⚠️
// Commence à 0.10 m/s (10 cm/s) pour les tests sur table/sol.
// Tu pourras l'augmenter à 0.30 m/s ou plus quand tu auras confiance.
const float TARGET_SPEED_MS = 0.35;  // Vitesse de croisière fluide

const float ARRIVAL_THRESHOLD = 0.15; // Rayon de validation d'un point (15 cm)
const float Kp_ANGULAR = 2.0;         // AUGMENTÉ de 0.2 → 0.5 pour réponse plus rapide

// --- Paramètres IMU & Boussole ---
// Déclinaison magnétique locale (Virton, BE = ~ +1.83°)
const float MAGNETIC_DECLINATION = 1.83; 

// Offsets du magnétomètre (À remplir après avoir fait la calibration en "8")
const int32_t MAG_OFFSET_X = -16;
const int32_t MAG_OFFSET_Y = 6;
const int32_t MAG_OFFSET_Z = -82;

// ==========================================
// CALIBRATION DE LA DIRECTION (OFFSETS)
// ==========================================
// Ajustez ces valeurs (en degrés, positif ou négatif) pour que 
// chaque roue soit parfaitement droite quand la commande est à 0.
#define OFFSET_SERVO_FL  6   // Offset Avant Gauche
#define OFFSET_SERVO_FR  6 // Offset Avant Droit
#define OFFSET_SERVO_RL  -6   // Offset Arrière Gauche
#define OFFSET_SERVO_RR  -6   // Offset Arrière Droit

// ==========================================
// INVERSION DES SERVOS (SENS DE MONTAGE)
// ==========================================
// Mettez 1 pour un sens normal. Mettez -1 pour inverser le servo.
#define DIR_SERVO_FL  -1   // Sens Avant Gauche
#define DIR_SERVO_FR  -1   // Sens Avant Droit
#define DIR_SERVO_RL  -1   // Sens Arrière Gauche
#define DIR_SERVO_RR  -1   // Sens Arrière Droit
// ==========================================
// ROUTAGE PHYSIQUE DES SERVOS (PCA9685)
// ==========================================
#define PORT_SERVO_FL 3  // Le câble Avant-Gauche est physiquement sur le port 3
#define PORT_SERVO_FR 4  // Le câble Avant-Droit est physiquement sur le port 4
#define PORT_SERVO_RL 1  // Le câble Arrière-Gauche est physiquement sur le port 1
#define PORT_SERVO_RR 2  // Le câble Arrière-Droit est physiquement sur le port 2


// ==========================================
// CALIBRATION DES MOTEURS PAS-À-PAS
// ==========================================
// Mettre à 'true' pour inverser le sens de rotation par défaut d'un moteur.
// Ordre des index : [0]=FL, [1]=FR, [2]=ML, [3]=MR, [4]=RL, [5]=RR
const bool INVERT_STEPPER[6] = {false, true, false, true, false, true}; // Exemple : inverse tout le côté droit

#define BUZZER_CHANNEL 16 // Le canal choisi sur le PCA9685
