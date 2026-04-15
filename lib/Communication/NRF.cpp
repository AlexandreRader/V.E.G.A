#include "NRF.h"

// ─────────────────────────────────────────────────────────────────────────────
//  Constructeur
// ─────────────────────────────────────────────────────────────────────────────
NRF_Comm::NRF_Comm(uint8_t cePin, uint8_t csnPin)
    : _radio(cePin, csnPin),
      _cePin(cePin),
      _csnPin(csnPin),
      _rawLen(0),
      _cmdHead(0),
      _cmdTail(0),
      _cmdCount(0)
{
    memset(_rawBuf, 0, sizeof(_rawBuf));
}

// ─────────────────────────────────────────────────────────────────────────────
//  begin()
// ─────────────────────────────────────────────────────────────────────────────
bool NRF_Comm::begin() {
    if (!_radio.begin()) {
        Serial.println("[NRF] ERREUR : module non détecté. Vérifie le câblage SPI.");
        return false;
    }

    // ── Configuration RF ──────────────────────────────────────────────────────
    _radio.setChannel(NRF_CHANNEL);           // Canal 112 (groupe 4)
    _radio.setDataRate(NRF_DATA_RATE);         // 250 KBPS
    _radio.setPALevel(NRF_PA_LEVEL);           // Puissance max
    _radio.setPayloadSize(NRF_PAYLOAD_SIZE);   // Trames de 32 octets fixes
    _radio.setAutoAck(false);                  // Auto-ACK désactivé (Hub ne le supporte pas)
    _radio.disableDynamicPayloads();           // Payload fixe obligatoire sans auto-ack

    // ── Ouverture du pipe de réception ────────────────────────────────────────
    _radio.openReadingPipe(1, NRF_PIPE_ADDRESS);
    _radio.startListening();

    Serial.println("[NRF] Module initialisé.");
    Serial.printf("[NRF] Canal    : %d\n", NRF_CHANNEL);
    Serial.printf("[NRF] Adresse  : 0xE8E8F0F0A4 (pipe 1)\n");
    Serial.printf("[NRF] Débit    : 250 KBPS\n");
    Serial.printf("[NRF] Payload  : %d octets\n", NRF_PAYLOAD_SIZE);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────────
//  update()  — à appeler dans loop()
// ─────────────────────────────────────────────────────────────────────────────
bool NRF_Comm::update() {
    if (!_radio.available()) return false;

    uint8_t packet[NRF_PAYLOAD_SIZE];
    bool    gotPacket = false;

    // Vide tous les paquets disponibles dans le FIFO RF (jusqu'à 3)
    while (_radio.available()) {
        _radio.read(packet, NRF_PAYLOAD_SIZE);
        gotPacket = true;

        // ── Accumulation dans le buffer brut ─────────────────────────────────
        // On s'arrête à NRF_CMD_BUFFER_SIZE - 1 pour ne jamais déborder
        int space = (NRF_CMD_BUFFER_SIZE - 1) - _rawLen;
        if (space <= 0) {
            // Buffer plein : on flush ce qu'on peut parser puis on recommence
            _parseBuffer();
            space = (NRF_CMD_BUFFER_SIZE - 1) - _rawLen;
        }

        int toCopy = min(NRF_PAYLOAD_SIZE, space);
        memcpy(_rawBuf + _rawLen, packet, toCopy);
        _rawLen += toCopy;

        // ── Debug : affiche le paquet reçu en ASCII + hex ─────────────────────
        Serial.print("[NRF] Paquet reçu : \"");
        for (int i = 0; i < NRF_PAYLOAD_SIZE; i++) {
            if (packet[i] >= 0x20 && packet[i] < 0x7F) Serial.print((char)packet[i]);
            else if (packet[i] == '\n')                 Serial.print("\\n");
            else if (packet[i] == '\0')                 Serial.print("\\0");
            else                                        Serial.printf("\\x%02X", packet[i]);
        }
        Serial.println("\"");
    }

    if (gotPacket) _parseBuffer();
    return gotPacket;
}

// ─────────────────────────────────────────────────────────────────────────────
//  _parseBuffer()
//  Découpe _rawBuf en lignes (séparées par '\n') et les enfile dans _cmdQueue.
//  Les octets nuls de remplissage (padding) sont ignorés.
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_parseBuffer() {
    int start = 0;

    for (int i = 0; i < _rawLen; i++) {
        char c = _rawBuf[i];

        if (c == '\n') {
            // Ligne complète trouvée
            int lineLen = i - start;
            if (lineLen > 0) {
                _enqueueCommand(_rawBuf + start, lineLen);
            }
            start = i + 1;
        }
        else if (c == '\0') {
            // Octet de padding : fin du contenu utile dans cette trame
            // On envoie ce qui reste avant le premier '\0' si non vide
            int lineLen = i - start;
            if (lineLen > 0) {
                // Pas de '\n' final → commande incomplète ou dernière ligne
                // On la garde dans le buffer en attendant la suite
                break;
            }
            start = i + 1; // Skip le padding
        }
    }

    // Décale ce qui n'a pas encore été consommé vers le début du buffer
    int remaining = _rawLen - start;
    if (remaining > 0 && start > 0) {
        memmove(_rawBuf, _rawBuf + start, remaining);
    }
    _rawLen = (remaining > 0) ? remaining : 0;
    _rawBuf[_rawLen] = '\0';
}

// ─────────────────────────────────────────────────────────────────────────────
//  _enqueueCommand()
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_enqueueCommand(const char* start, int len) {
    if (_cmdCount >= 16) {
        Serial.println("[NRF] AVERTISSEMENT : file de commandes pleine, commande ignorée.");
        return;
    }
    _cmdQueue[_cmdTail] = String(start).substring(0, len);
    _cmdTail = (_cmdTail + 1) % 16;
    _cmdCount++;
    Serial.printf("[NRF] Commande extraite : \"%s\"\n", _cmdQueue[(_cmdTail + 15) % 16].c_str());
}

// ─────────────────────────────────────────────────────────────────────────────
//  hasCommand() / readCommand()
// ─────────────────────────────────────────────────────────────────────────────
bool NRF_Comm::hasCommand() const {
    return _cmdCount > 0;
}

String NRF_Comm::readCommand() {
    if (_cmdCount == 0) return String();
    String cmd = _cmdQueue[_cmdHead];
    _cmdHead = (_cmdHead + 1) % 16;
    _cmdCount--;
    return cmd;
}

// ─────────────────────────────────────────────────────────────────────────────
//  printStatus()
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::printStatus() const {
    Serial.println("──── NRF_Comm Status ─────────────────────");
    Serial.printf("  Canal         : %d\n",    NRF_CHANNEL);
    Serial.printf("  Pipe adresse  : 0xE8E8F0F0A4\n");
    Serial.printf("  Débit         : 250 KBPS\n");
    Serial.printf("  Auto-ACK      : désactivé\n");
    Serial.printf("  Payload       : %d octets\n", NRF_PAYLOAD_SIZE);
    Serial.printf("  Cmds en file  : %d\n",   _cmdCount);
    Serial.printf("  Buffer brut   : %d octets\n", _rawLen);
    Serial.println("──────────────────────────────────────────");
}