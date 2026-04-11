#ifndef NRF_H
#define NRF_H

#include <Arduino.h>
#include <RF24.h>
#include <SPI.h>
#include "pins.h"

// ==========================================
// MODULE NRF24L01 - Communication Radio
// ==========================================

class NRF24L01_Receiver {
private:
    // Instance RF24
    RF24 radio;

    // Configuration radio
    const uint8_t CHANNEL = 90;           // Canal radio (0-127)
    const uint8_t PAYLOAD_SIZE = 32;      // Taille du payload (max 32 octets)
    const uint64_t RECEIVE_ADDR = 0xE8E8F0F0E1LL;   // Adresse de réception (5 octets)

    // Buffer de données
    uint8_t rx_buffer[32];
    bool data_available;

public:
    NRF24L01_Receiver() : radio(PIN_RADIO_CE, PIN_SPI_CSN), data_available(false) {}

    // Initialisation du module NRF24L01
    bool begin() {
        // Initialisation SPI
        SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CSN);

        // Initialisation du module radio
        if (!radio.begin()) {
            Serial.println("Erreur: Impossible d'initialiser NRF24L01");
            return false;
        }

        // Configuration
        radio.setChannel(CHANNEL);               // Canal
        radio.setPayloadSize(PAYLOAD_SIZE);      // Taille du payload
        radio.setPALevel(RF24_PA_LOW);           // Puissance faible pour tests
        radio.setDataRate(RF24_250KBPS);         // Débit

        // Configuration en mode réception
        radio.openReadingPipe(1, RECEIVE_ADDR);  // Pipe 1 pour réception
        radio.startListening();                  // Démarrer l'écoute

        // Délai d'initialisation
        delay(100);

        Serial.println("NRF24L01 initialisé en mode réception");
        Serial.print("Adresse: 0x");
        Serial.println(RECEIVE_ADDR, HEX);
        Serial.print("Canal: ");
        Serial.println(CHANNEL);

        return true;
    }

    // Vérifier si des données sont disponibles
    bool isDataAvailable() {
        return radio.available();
    }

    // Lire les données reçues
    bool readData(uint8_t* buffer, uint8_t& length) {
        if (isDataAvailable()) {
            // Lecture des données
            radio.read(buffer, PAYLOAD_SIZE);
            length = PAYLOAD_SIZE;

            Serial.print("Données reçues (");
            Serial.print(length);
            Serial.print(" octets): ");
            for (uint8_t i = 0; i < length; i++) {
                if (buffer[i] < 16) Serial.print("0");
                Serial.print(buffer[i], HEX);
                Serial.print(" ");
            }
            Serial.println();

            return true;
        }
        return false;
    }

    // Lire les données sous forme de chaîne (si c'est du texte)
    bool readString(String& message) {
        uint8_t buffer[32];
        uint8_t length;

        if (readData(buffer, length)) {
            // Conversion en chaîne (arrêt au premier caractère nul ou fin)
            message = "";
            for (uint8_t i = 0; i < length; i++) {
                if (buffer[i] == 0) break;
                message += (char)buffer[i];
            }
            return true;
        }
        return false;
    }

    // Lire les données sous forme de valeurs numériques (ex: coordonnées)
    bool readFloatArray(float* values, uint8_t max_count) {
        uint8_t buffer[32];
        uint8_t length;

        if (readData(buffer, length)) {
            // Vérifier que la taille correspond à des floats
            if (length % 4 != 0) {
                Serial.println("Erreur: taille des données incompatible avec float array");
                return false;
            }

            uint8_t float_count = length / 4;
            if (float_count > max_count) float_count = max_count;

            // Conversion des octets en floats
            for (uint8_t i = 0; i < float_count; i++) {
                uint8_t* float_bytes = &buffer[i * 4];
                memcpy(&values[i], float_bytes, 4);
            }

            Serial.print("Floats reçus: ");
            for (uint8_t i = 0; i < float_count; i++) {
                Serial.print(values[i], 3);
                Serial.print(" ");
            }
            Serial.println();

            return true;
        }
        return false;
    }

    // Fonction utilitaire pour envoyer une réponse (optionnel)
    bool sendResponse(const char* response) {
        uint8_t response_length = strlen(response);
        if (response_length > PAYLOAD_SIZE) response_length = PAYLOAD_SIZE;

        // Copie de la réponse dans le buffer
        memcpy(rx_buffer, response, response_length);

        // Arrêter l'écoute temporairement
        radio.stopListening();

        // Configuration en mode émission
        radio.openWritingPipe(RECEIVE_ADDR);

        // Envoi de la réponse
        bool success = radio.write(rx_buffer, response_length);

        // Reprendre l'écoute
        radio.startListening();

        if (success) {
            Serial.print("Réponse envoyée: ");
            Serial.println(response);
        } else {
            Serial.println("Erreur lors de l'envoi de la réponse");
        }

        return success;
    }

    // Diagnostic du module
    void printStatus() {
        Serial.println("\n=== STATUS NRF24L01 ===");
        Serial.print("CE Pin: ");
        Serial.println(PIN_RADIO_CE);
        Serial.print("CSN Pin: ");
        Serial.println(PIN_SPI_CSN);
        Serial.print("Canal: ");
        Serial.println(CHANNEL);
        Serial.print("Adresse RX: 0x");
        Serial.println(RECEIVE_ADDR, HEX);
        Serial.print("Payload size: ");
        Serial.println(PAYLOAD_SIZE);
        Serial.println("======================");
    }
};

// Instance globale pour utilisation facile
extern NRF24L01_Receiver nrf_receiver;

#endif // NRF_H