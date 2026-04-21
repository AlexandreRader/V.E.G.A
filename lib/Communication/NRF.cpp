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
    bool gotPacket = false;

    while (_radio.available()) {
        _radio.read(packet, NRF_PAYLOAD_SIZE);

        // --- SÉCURITÉ 1 : Ignorer les paquets fantômes (uniquement des zéros) ---
        bool allZeros = true;
        for (int i = 0; i < NRF_PAYLOAD_SIZE; i++) {
            if (packet[i] != 0) {
                allZeros = false;
                break;
            }
        }
        if (allZeros) continue; // Si c'est du vide, on passe au paquet suivant sans rien faire

        gotPacket = true;

        // --- Accumulation dans le buffer brut ---
        int space = (NRF_CMD_BUFFER_SIZE - 1) - _rawLen;
        if (space <= 0) {
            _parseBuffer();
            space = (NRF_CMD_BUFFER_SIZE - 1) - _rawLen;
        }

        int toCopy = min((int)NRF_PAYLOAD_SIZE, space);
        memcpy(_rawBuf + _rawLen, packet, toCopy);
        _rawLen += toCopy;

        // --- SÉCURITÉ 2 : On commente le Serial.print de debug pour libérer la console ---
        /*
        Serial.print("[NRF] Paquet reçu : \"");
        for (int i = 0; i < NRF_PAYLOAD_SIZE; i++) {
            if (packet[i] >= 0x20 && packet[i] < 0x7F) Serial.print((char)packet[i]);
            else if (packet[i] == '\n') Serial.print("\\n");
            else if (packet[i] == '\0') Serial.print("\\0");
            else Serial.printf("\\x%02X", packet[i]);
        }
        Serial.println("\"");
        */
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
    if (_rawLen == 0) return;

    // Si on a reçu 32 octets mais qu'il n'y a pas de '\n', 
    // on force le traitement du contenu comme une commande unique.
    bool foundLine = false;
    for (int i = 0; i < _rawLen; i++) {
        if (_rawBuf[i] == '\n' || _rawBuf[i] == '\r') {
            _enqueueCommand(_rawBuf, i); // Enfile ce qu'il y a AVANT le \n
            
            // On décale le reste du buffer
            int remaining = _rawLen - (i + 1);
            if (remaining > 0) {
                memmove(_rawBuf, _rawBuf + i + 1, remaining);
                _rawLen = remaining;
            } else {
                _rawLen = 0;
            }
            foundLine = true;
            break;
        }
    }

    // Force : Si le buffer est plein ou si on a un paquet complet sans \n
    if (!foundLine && _rawLen >= NRF_PAYLOAD_SIZE) {
        _enqueueCommand(_rawBuf, _rawLen);
        _rawLen = 0;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  _enqueueCommand()
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_enqueueCommand(const char* start, int len) {
    if (_cmdCount >= 16) return;

    // Création d'une String propre
    String newCmd = "";
    for(int i = 0; i < len; i++) {
        if(start[i] >= 0x20 && start[i] < 0x7F) { // Uniquement caractères imprimables
            newCmd += start[i];
        }
    }

    if (newCmd.length() > 0) {
        _cmdQueue[_cmdTail] = newCmd;
        _cmdTail = (_cmdTail + 1) % 16;
        _cmdCount++;
        Serial.printf("\n📡 [NRF] Commande validée : \"%s\"\n", newCmd.c_str());
    }
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