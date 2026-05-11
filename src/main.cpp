#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "config.h"
#include "pins.h"
//#include "Mission.h"
#include "NRF.h"
#include "Navigation.h" // Inclut implicitement EKF, ToF, Kinematics, etc.

// ==========================================
// VARIABLES DE MISSION EN RAM (Modifiées par la radio)
// ==========================================
int PATH_SIZE = 0;
Waypoint MISSION_PATH[MAX_WAYPOINTS];
bool mission_ready_to_start = false;

float START_X = 0.0;
float START_Y = 0.0;
float START_THETA = 0.0;
float GOAL_X = 0.0;
float GOAL_Y = 0.0;
float GOAL_THETA = 0.0;

// ==========================================
// INSTANCIATION DES SYSTÈMES GLOBAUX
// ==========================================
// NRF gère la télécommande.
NRF_Comm nrf(PIN_RADIO_CE, PIN_SPI_CSN);

// NavigationController gère TOUTE la mécanique et l'intelligence.
NavigationController nav; 

// ==========================================
// INITIALISATION
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000); // Laisse le port série s'ouvrir
    
    Serial.println("\n==========================================");
    Serial.println("🚀 VEGA SC317 - MODE COMPÉTITION");
    Serial.println("==========================================");

    // 1. Démarrage des bus de communication
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CSN);

    // 2. Initialisation de l'intelligence artificielle (Capteurs, EKF, Actionneurs)
    if (!nav.initialize()) {
        Serial.println("⚠️ ALARME : L'initialisation du robot a retourné des erreurs.");
        // Optionnel : Mettre la LED en ROUGE ici
    } else {
        // Optionnel : Mettre la LED en VERT ici (prêt)
    }

    // 3. Initialisation Radio
    if (nrf.begin()) {
        Serial.println("📡 Radio branchée : En écoute des ordres Station Sol...");
    } else {
        Serial.println("❌ ERREUR CRITIQUE RADIO !");
    }
}

// ==========================================
// BOUCLE PRINCIPALE (Ultra-rapide)
// ==========================================
void loop() {
    // --------------------------------------------------
    // 1. ÉCOUTE DE LA STATION SOL (Radio NRF)
    // --------------------------------------------------
    nrf.update();

    if (nrf.hasCommand()) {
        String cmd = nrf.readCommand();

        if (cmd == "MISSION_LOADED") {
            // Le NRF vient de remplir les variables globales de mission.
            // On "téléporte" virtuellement le robot au point de départ :
            nav.setInitialPosition(START_X, START_Y, START_THETA);
            Serial.printf("🎯 Mission validée en RAM (%d WP) ! Prêt pour le décollage.\n", PATH_SIZE);
        } 
        else if (cmd == "START") {
            if (mission_ready_to_start) {
                nav.startMission();
                Serial.println("🚀 ORDRE REÇU : DÉCOLLAGE !");
            } else {
                Serial.println("❌ Impossible : Aucune mission chargée !");
            }
        } 
        else if (cmd == "STOP") {
            nav.stopMission();
            Serial.println("🛑 ORDRE REÇU : ARRÊT D'URGENCE !");
        }
        else {
            Serial.printf("❓ Ordre radio inconnu : %s\n", cmd.c_str());
        }
    }

    // --------------------------------------------------
    // 2. RÉFLEXES ET CONDUITE (Géré par Navigation.h)
    // --------------------------------------------------
    // Cette fonction calcule d'elle-même le temps écoulé (dt),
    // lit les ToF, suit la courbe, et envoie les vitesses aux moteurs.
    nav.update();

    // --------------------------------------------------
    // 3. TÉLÉMÉTRIE (Vers la station sol via NRF ou Serial)
    // --------------------------------------------------
    static unsigned long last_telemetry_time = 0;
    unsigned long now = millis();
    
    if (now - last_telemetry_time >= 500) { // Toutes les demi-secondes
        last_telemetry_time = now;
        
        // On n'affiche la télémétrie que si le robot est en train de travailler
        if (nav.getCurrentState() != STATE_IDLE) {
            Serial.printf("📍 NAV | X:%.2f  Y:%.2f  Cap:%.1f° | État: %d\n", 
                          nav.getRobotX(), 
                          nav.getRobotY(), 
                          nav.getRobotTheta() * 180.0 / M_PI, 
                          nav.getCurrentState());
        }
    }
}