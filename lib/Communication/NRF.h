#pragma once

#include <RF24.h>
#include <SPI.h>
#include "../../V.E.G.A/include/pins.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Paramètres RF — Groupe 4 : V.E.G.A
// ─────────────────────────────────────────────────────────────────────────────
#define NRF_CHANNEL         112
#define NRF_DATA_RATE       RF24_250KBPS
#define NRF_PA_LEVEL        RF24_PA_MAX
#define NRF_PAYLOAD_SIZE    32

// Adresse du pipe de réception (groupe 4)
static const uint64_t NRF_PIPE_ADDRESS = 0xE8E8F0F0A4LL;

// ─────────────────────────────────────────────────────────────────────────────
//  Broches SPI — ESP32 (modifiables selon ton câblage)
// ─────────────────────────────────────────────────────────────────────────────

// MOSI = GPIO 23 | MISO = GPIO 19 | SCK = GPIO 18  (SPI par défaut ESP32)

// ─────────────────────────────────────────────────────────────────────────────
//  Taille max du buffer de commandes reconstituées
//  (plusieurs trames de 32 octets peuvent former un message complet)
// ─────────────────────────────────────────────────────────────────────────────
#define NRF_CMD_BUFFER_SIZE 3200

// ─────────────────────────────────────────────────────────────────────────────
//  Classe NRF_Comm
// ─────────────────────────────────────────────────────────────────────────────
class NRF_Comm {
public:
    /**
     * @param cePin   Broche CE du module NRF24L01+
     * @param csnPin  Broche CSN du module NRF24L01+
     */
    NRF_Comm(uint8_t cePin = PIN_RADIO_CE, uint8_t csnPin = PIN_SPI_CSN);

    /**
     * Initialise le module RF.
     * @return true si le module est détecté et configuré, false sinon.
     */
    bool begin();

    /**
     * À appeler dans loop(). Lit les paquets disponibles et les accumule
     * dans le buffer interne. Retourne true si au moins un paquet a été lu.
     */
    bool update();

    /**
     * Retourne true si une ou plusieurs lignes de commande complètes
     * sont disponibles dans le buffer.
     */
    bool hasCommand() const;

    /**
     * Extrait la prochaine ligne de commande du buffer (sans le '\n').
     * Retourne une chaîne vide si aucune commande n'est disponible.
     */
    String readCommand();

    /**
     * Affiche l'état du module sur le port série (debug).
     */
    void printStatus() const;

private:
    RF24    _radio;
    uint8_t _cePin;
    uint8_t _csnPin;

    // Buffer de reconstruction des trames
    char    _rawBuf[NRF_CMD_BUFFER_SIZE];
    int     _rawLen;

    // File de commandes extraites (séparées par '\n')
    String  _cmdQueue[16];
    uint8_t _cmdHead;
    uint8_t _cmdTail;
    uint8_t _cmdCount;

    void _parseBuffer();
    void _enqueueCommand(const char* start, int len);
};