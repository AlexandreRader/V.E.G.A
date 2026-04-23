#pragma once
#include <math.h>

// ==========================================
// CONFIGURATION GLOBALE - VEGA SC317
// ==========================================

// --- Dimensions du Rover (en mètres) ---
const float L_AXE = 0.20;            // Distance entre le centre et l'axe avant
const float W_VOIE = 0.30;           // Largeur totale entre roues gauche et droite
const float WHEEL_RADIUS = 0.05;     // Rayon des roues (5 cm)

// --- Paramètres des Moteurs (Steppers) ---
const float STEPS_PER_REV = 200.0;   // Moteurs 1.8° par pas
const int MICROSTEPPING = 16;        // Drivers configurés en 1/16
const float MAX_SPEED_HZ = 2000.0;   // Limite de sécurité logicielle

// --- Constantes Calculées ---
const float METERS_PER_STEP = (2.0 * M_PI * WHEEL_RADIUS) / (STEPS_PER_REV * MICROSTEPPING);

// --- Paramètres de Vitesse et Navigation ---

// ⚠️ LA VITESSE CIBLE GLOBALE DU ROVER (m/s) ⚠️
// Commence à 0.10 m/s (10 cm/s) pour les tests sur table/sol.
// Tu pourras l'augmenter à 0.30 m/s ou plus quand tu auras confiance.
const float TARGET_SPEED_MS = 0.05; 

const float ARRIVAL_THRESHOLD = 0.15; // Rayon de validation d'un point (15 cm)
const float Kp_ANGULAR = 1.2;         // Gain de braquage (Force avec laquelle il tourne)
