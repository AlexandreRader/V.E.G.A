/*
 * ==========================================
 * EXEMPLE D'UTILISATION DU RÉCEPTEUR NRF24L01
 * ==========================================
 *
 * Ce fichier montre comment intégrer la réception NRF24L01
 * dans votre programme principal.
 *
 * Copiez le code ci-dessous dans votre main.cpp ou créez
 * un fichier séparé pour les tests.
 */

#include <Arduino.h>
#include "Communication/NRF.h"

// ==========================================
// CONFIGURATION ET VARIABLES GLOBALES
// ==========================================

// Instance du récepteur (déjà déclarée dans NRF.h)
extern NRF24L01_Receiver nrf_receiver;

// Variables pour stocker les données reçues
String last_message = "";
float current_position[3] = {0.0, 0.0, 0.0}; // x, y, theta
unsigned long last_receive_time = 0;

// ==========================================
// FONCTIONS UTILITAIRES
// ==========================================

void processReceivedMessage(const String& message) {
    Serial.print("Traitement du message: ");
    Serial.println(message);

    // Exemples de traitement selon le contenu
    if (message.startsWith("CMD:")) {
        String command = message.substring(4);
        Serial.print("Commande reçue: ");
        Serial.println(command);

        // Traiter différentes commandes
        if (command == "START") {
            Serial.println("Démarrage de la mission");
            // Code pour démarrer la mission
        } else if (command == "STOP") {
            Serial.println("Arrêt d'urgence");
            // Code d'arrêt d'urgence
        } else if (command == "STATUS") {
            // Envoyer un rapport de statut
            nrf_receiver.sendResponse("STATUS:OK");
        }

    } else if (message.startsWith("POS:")) {
        // Mise à jour de position depuis la station
        Serial.println("Mise à jour de position reçue");
        // Le traitement des coordonnées se fait dans loop()
    }
}

void processReceivedPosition(float x, float y, float theta) {
    // Mise à jour de la position actuelle
    current_position[0] = x;
    current_position[1] = y;
    current_position[2] = theta;

    Serial.print("Position mise à jour: x=");
    Serial.print(x, 2);
    Serial.print(" y=");
    Serial.print(y, 2);
    Serial.print(" theta=");
    Serial.println(theta, 2);

    // Ici vous pouvez ajouter la logique de navigation
    // Par exemple: calculer une trajectoire vers cette position
}

// ==========================================
// FONCTIONS ARDUINO STANDARD
// ==========================================

void setup() {
    // Initialisation série
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n=== INITIALISATION VEGA SC317 ===");

    // Initialisation du récepteur NRF24L01
    if (!nrf_receiver.begin()) {
        Serial.println("❌ ERREUR: Impossible d'initialiser le NRF24L01");
        Serial.println("Vérifiez les connexions et redémarrez");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("✅ NRF24L01 initialisé avec succès");

    // Afficher le statut du module
    nrf_receiver.printStatus();

    Serial.println("=== PRÊT À RECEVOIR DES DONNÉES ===\n");
}

void loop() {
    // ==========================================
    // MÉTHODE 1: LECTURE DE MESSAGES TEXTE
    // ==========================================
    String message;
    if (nrf_receiver.readString(message)) {
        last_receive_time = millis();
        processReceivedMessage(message);
    }

    // ==========================================
    // MÉTHODE 2: LECTURE DE COORDONNÉES (FLOATS)
    // ==========================================
    float coordinates[3];
    if (nrf_receiver.readFloatArray(coordinates, 3)) {
        last_receive_time = millis();
        processReceivedPosition(coordinates[0], coordinates[1], coordinates[2]);
    }

    // ==========================================
    // MÉTHODE 3: LECTURE DE DONNÉES BRUTES (AVANCÉ)
    // ==========================================
    /*
    uint8_t raw_buffer[32];
    uint8_t data_length;
    if (nrf_receiver.readData(raw_buffer, data_length)) {
        last_receive_time = millis();

        // Traitement personnalisé selon votre protocole
        // Par exemple: premiers octets = type de message
        uint8_t message_type = raw_buffer[0];

        switch (message_type) {
            case 0x01: // Message texte
                // ...
                break;
            case 0x02: // Coordonnées
                // ...
                break;
            case 0x03: // Commandes
                // ...
                break;
        }
    }
    */

    // ==========================================
    // DIAGNOSTIC ET MONITORING
    // ==========================================

    // Vérifier si on reçoit toujours des données (timeout)
    static unsigned long last_check = 0;
    if (millis() - last_check > 5000) { // Toutes les 5 secondes
        if (millis() - last_receive_time > 10000) { // Plus de 10s sans données
            Serial.println("⚠️  Aucune donnée reçue depuis 10 secondes");
        }
        last_check = millis();
    }

    // Petit délai pour éviter la surcharge CPU
    delay(10);
}