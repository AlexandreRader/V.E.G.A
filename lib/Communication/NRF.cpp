#include "NRF.h"
#include "mission.h"

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
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_parseBuffer() {
    if (_rawLen == 0) return;

    bool foundLine = false;
    for (int i = 0; i < _rawLen; i++) {
        // Dès qu'on trouve un 'Entrée' (\n ou \r), on sait que le message est fini !
        if (_rawBuf[i] == '\n' || _rawBuf[i] == '\r') {
            _enqueueCommand(_rawBuf, i); // On traite tout le message reconstitué
            
            // On décale le reste du buffer s'il y a d'autres données après
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

    // 🎯 CORRECTION ICI : On ne force le traitement QUE si le grand buffer de 512 octets 
    // est plein pour éviter qu'il n'explose. S'il n'est pas plein, on attend sagement 
    // l'arrivée du reste des paquets de 32 octets !
    if (!foundLine && _rawLen >= (NRF_CMD_BUFFER_SIZE - 1)) {
        _enqueueCommand(_rawBuf, _rawLen);
        _rawLen = 0;
    }
}
// ─────────────────────────────────────────────────────────────────────────────
//  _enqueueCommand()
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_enqueueCommand(const char* start, int len) {
    if (_cmdCount >= 16) return;

    String newCmd = "";
    for(int i = 0; i < len; i++) {
        if(start[i] >= 0x20 && start[i] < 0x7F) { 
            newCmd += start[i];
        }
    }

    if (newCmd.length() > 0) {
        // 🎯 INTERCEPTION DE LA MISSION
        if (newCmd.charAt(0) == 'M' || newCmd.charAt(0) == 'm') {
            Serial.printf("\n📡 [NRF] Trajectoire brute reçue (%d caractères). Traitement...\n", newCmd.length());
            
            if (_parseMissionString(newCmd)) {
                // On prévient le main.cpp avec un message simple au lieu de lui envoyer les 200 caractères
                _cmdQueue[_cmdTail] = "MISSION_LOADED";
                _cmdTail = (_cmdTail + 1) % 16;
                _cmdCount++;
            }
            return; // ⚠️ On quitte la fonction, on ne stocke pas le gros texte dans la file !
        }

        // Si ce n'est pas une mission, c'est une commande normale
        _cmdQueue[_cmdTail] = newCmd;
        _cmdTail = (_cmdTail + 1) % 16;
        _cmdCount++;
        Serial.printf("\n📡 [NRF] Commande validée : \"%s\"\n", newCmd.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  _parseMissionString() - Le Cerveau du décodage
// ─────────────────────────────────────────────────────────────────────────────
bool NRF_Comm::_parseMissionString(String payload) {
    PATH_SIZE = 0; 
    
    // 1. Extraire l'en-tête (Tout avant le premier ';')
    int headerEnd = payload.indexOf(';');
    if (headerEnd == -1) return false;

    String header = payload.substring(1, headerEnd); // On saute le 'M'
    
    // Découpage manuel de l'en-tête (6 valeurs séparées par des virgules)
    int idx[6], i = 0;
    int current_comma = header.indexOf(',');
    while (current_comma != -1 && i < 5) {
        idx[i++] = current_comma;
        current_comma = header.indexOf(',', current_comma + 1);
    }
    
    if (i == 5) { // Si on a bien trouvé 5 virgules (donc 6 valeurs)
        START_X     = header.substring(0, idx[0]).toFloat();
        START_Y     = header.substring(idx[0]+1, idx[1]).toFloat();
        START_THETA = header.substring(idx[1]+1, idx[2]).toFloat();
        GOAL_X      = header.substring(idx[2]+1, idx[3]).toFloat();
        GOAL_Y      = header.substring(idx[3]+1, idx[4]).toFloat();
        GOAL_THETA  = header.substring(idx[4]+1).toFloat();
    } else {
        Serial.println("❌ [NRF] En-tête de mission invalide !");
        return false;
    }

    // 2. Extraire la liste des waypoints (Après le premier ';')
    int startIndex = headerEnd + 1;
    int endIndex = payload.indexOf(';', startIndex);

    while (endIndex != -1 && PATH_SIZE < MAX_WAYPOINTS) {
        String wpStr = payload.substring(startIndex, endIndex);
        int commaIndex = wpStr.indexOf(',');
        
        if (commaIndex != -1) {
            MISSION_PATH[PATH_SIZE].x = wpStr.substring(0, commaIndex).toFloat();
            MISSION_PATH[PATH_SIZE].y = wpStr.substring(commaIndex + 1).toFloat();
            PATH_SIZE++;
        }
        startIndex = endIndex + 1;
        endIndex = payload.indexOf(';', startIndex);
    }

    if (PATH_SIZE > 0) {
        mission_ready_to_start = true;
        Serial.printf("✅ [NRF] Mission décodée : %d WP. Départ: [%.1f, %.1f, %.1f°]\n", 
                      PATH_SIZE, START_X, START_Y, START_THETA * 180.0/M_PI);
        return true;
    }
    return false;
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