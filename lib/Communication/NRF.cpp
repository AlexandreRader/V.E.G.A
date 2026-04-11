#include "NRF.h"

// Instance globale du récepteur NRF24L01
NRF24L01_Receiver nrf_receiver;

// ==========================================
// EXEMPLE D'UTILISATION DU MODULE NRF24L01
// ==========================================
/*
Dans votre code principal (main.cpp), ajoutez :

#include "Communication/NRF.h"

Puis dans setup():
    if (!nrf_receiver.begin()) {
        Serial.println("Erreur initialisation NRF24L01");
        while(1); // Boucle infinie en cas d'erreur
    }

Dans loop():
    // Méthode 1: Lecture de données brutes
    uint8_t buffer[32];
    uint8_t length;
    if (nrf_receiver.readData(buffer, length)) {
        // Traiter les données reçues dans buffer[0..length-1]
        Serial.print("Données reçues: ");
        for (uint8_t i = 0; i < length; i++) {
            Serial.print(buffer[i]);
            Serial.print(" ");
        }
        Serial.println();
    }

    // Méthode 2: Lecture de chaîne de caractères
    String message;
    if (nrf_receiver.readString(message)) {
        Serial.print("Message: ");
        Serial.println(message);

        // Exemple de réponse automatique
        if (message == "PING") {
            nrf_receiver.sendResponse("PONG");
        }
    }

    // Méthode 3: Lecture de valeurs flottantes (coordonnées, etc.)
    float coordinates[3]; // x, y, theta par exemple
    if (nrf_receiver.readFloatArray(coordinates, 3)) {
        float x = coordinates[0];
        float y = coordinates[1];
        float theta = coordinates[2];

        Serial.print("Position: x=");
        Serial.print(x);
        Serial.print(" y=");
        Serial.print(y);
        Serial.print(" theta=");
        Serial.println(theta);
    }

    delay(10); // Petit délai pour éviter la surcharge
}

// ==========================================
// CONFIGURATION CÔTÉ ÉMETTEUR (pour référence)
// ==========================================
/*
Côté émetteur, vous pouvez utiliser un code similaire :

#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

const uint8_t CHANNEL = 90;
const uint8_t PAYLOAD_SIZE = 32;
const char* TRANSMIT_ADDR = "SERV1";

void setup() {
    Serial.begin(115200);

    // Configuration pins (adapter selon votre carte)
    Mirf.cePin = 9;    // CE pin
    Mirf.csnPin = 10;  // CSN pin

    Mirf.spi = &MirfHardwareSpi;
    Mirf.init();

    Mirf.setTADDR((byte*)TRANSMIT_ADDR);
    Mirf.payload = PAYLOAD_SIZE;
    Mirf.channel = CHANNEL;
    Mirf.config();

    Serial.println("NRF24L01 émetteur initialisé");
}

void loop() {
    // Exemple d'envoi de coordonnées
    float data[3] = {1.23, 4.56, 0.78}; // x, y, theta
    Mirf.send((byte*)data);

    while (Mirf.isSending()) {
        delay(10);
    }

    Serial.println("Coordonnées envoyées");
    delay(1000);
}
*/

// Fonctions utilitaires supplémentaires peuvent être ajoutées ici si nécessaire