#pragma once
#include <Arduino.h>

// Structure contenant les ordres purs pour les 10 actionneurs
struct MotorCommands {
    // Angles des 4 servomoteurs (en radians)
    // 0 = Tout droit, Positif = Braquage Intérieur/Extérieur
    float angle_FL; // Front Left (Avant Gauche)
    float angle_FR; // Front Right (Avant Droit)
    float angle_RL; // Rear Left (Arrière Gauche)
    float angle_RR; // Rear Right (Arrière Droit)
    
    // Vitesses linéaires des 6 roues (en mètres/seconde)
    float speed_FL; // Front Left
    float speed_ML; // Middle Left (Milieu Gauche)
    float speed_RL; // Rear Left
    float speed_FR; // Front Right
    float speed_MR; // Middle Right (Milieu Droit)
    float speed_RR; // Rear Right
};

class Kinematics {
private:
    // ==========================================
    // 📏 DIMENSIONS PHYSIQUES DU ROVER (en mètres)
    // ==========================================
    // L = Distance entre l'essieu central et l'essieu avant (ou arrière)
    const float L = 0.25; 
    
    // W = Demi-largeur du robot (Distance du centre à la roue)
    const float W = 0.20; 

    // Rayon des roues (utile pour convertir la vitesse m/s en tr/min)
    const float WHEEL_RADIUS = 0.05; 

public:
    // Constructeur vide
    Kinematics() {}

    /**
     * Calcule la cinématique inverse pour un châssis 6 roues (4 directrices)
     * @param linear_v Vitesse linéaire désirée (en m/s)
     * @param angular_w Vitesse de rotation désirée (en rad/s)
     * @return MotorCommands contenant les angles et vitesses des 10 moteurs
     */
    MotorCommands calculateDrive(float linear_v, float angular_w);

    /**
     * Utilitaire : Convertit une vitesse en mètres/seconde en impulsions par seconde (Hz)
     * pour piloter les drivers A4988.
     * @param speed_m_s Vitesse de la roue en m/s
     * @param steps_per_rev Nombre de pas par tour (ex: 200 pour un NEMA 17)
     * @param microstepping Réglage du driver (ex: 1, 2, 4, 8, 16)
     */
    float speedToStepsHz(float speed_m_s, int steps_per_rev = 200, int microstepping = 1);
};