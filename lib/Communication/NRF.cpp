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
    // 1. LE CHRONOMÈTRE DE SÉCURITÉ
    if (!_radio.available()) {
        if (_rawLen > 0 && (millis() - _lastRxTime > 100)) {
            Serial.println("⏱️ [NRF] Silence radio détecté. Forçage du décodage !");
            _parseBuffer(true);
        }
        return false;
    }

    uint8_t packet[NRF_PAYLOAD_SIZE];
    bool gotPacket = false;

    // 2. LECTURE ULTRA-RAPIDE EN BOUCLE
    while (_radio.available()) {
    _radio.read(packet, NRF_PAYLOAD_SIZE);
    _lastRxTime = millis();

    // Ignorer les paquets 100% vides
    bool allZeros = true;
    for (int i = 0; i < NRF_PAYLOAD_SIZE; i++) {
        if (packet[i] != 0) { allZeros = false; break; }
    }
    if (allZeros) continue;

    gotPacket = true;

    // ✅ CORRECTION : ne copier que jusqu'au premier \0 du padding
    // Les \0 en fin de paquet sont du remplissage, pas des données
    int usefulLen = NRF_PAYLOAD_SIZE;
    for (int i = 0; i < NRF_PAYLOAD_SIZE; i++) {
        if (packet[i] == '\0') {
            usefulLen = i;
            break;
        }
    }

    int space = (NRF_CMD_BUFFER_SIZE - 1) - _rawLen;
    int toCopy = min(usefulLen, space);
    if (toCopy > 0) {
        memcpy(_rawBuf + _rawLen, packet, toCopy);
        _rawLen += toCopy;
    }
}

    if (gotPacket) _parseBuffer(false);
    return gotPacket;
}

// ─────────────────────────────────────────────────────────────────────────────
//  _parseBuffer()
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_parseBuffer(bool force) {
    if (_rawLen == 0) return;

    bool foundLine = false;
    int start = 0;

    while (start < _rawLen && _cmdCount < NRF_CMD_QUEUE_SIZE) {
        int end = -1;
        for (int i = start; i < _rawLen; i++) {
            // ⚠️ PAS de '\0' ici — les \0 sont du padding en milieu de mission !
            if (_rawBuf[i] == '\n' || _rawBuf[i] == '\r' || _rawBuf[i] == '*') {
                end = i;
                break;
            }
        }

        if (end == -1) break;

        if (end > start) {
            _enqueueCommand(_rawBuf + start, end - start);
            foundLine = true;
        }
        start = end + 1;

        // Sauter les \0 de padding qui suivent le délimiteur
        while (start < _rawLen && _rawBuf[start] == '\0') start++;
    }

    if (start > 0) {
        int remaining = _rawLen - start;
        if (remaining > 0) {
            memmove(_rawBuf, _rawBuf + start, remaining);
            _rawLen = remaining;
        } else {
            _rawLen = 0;
        }
    }

    if (!foundLine && force && _rawLen > 0) {
        _enqueueCommand(_rawBuf, _rawLen);
        _rawLen = 0;
    }
}
// ─────────────────────────────────────────────────────────────────────────────
//  _enqueueCommand()
// ─────────────────────────────────────────────────────────────────────────────
void NRF_Comm::_enqueueCommand(const char* start, int len) {
    if (_cmdCount >= NRF_CMD_QUEUE_SIZE) {
        Serial.println("⚠️ [NRF] File pleine ! Commande perdue.");
        return;
    }

    String newCmd = "";
    for (int i = 0; i < len; i++) {
        if (start[i] >= 0x20 && start[i] < 0x7F) {
            newCmd += start[i];
        }
    }

    if (newCmd.length() > 0) {
        if (newCmd.charAt(0) == 'M' || newCmd.charAt(0) == 'm') {
            Serial.printf("\n📡 [NRF] Trajectoire brute reçue (%d caractères). Traitement...\n", newCmd.length());
            if (_parseMissionString(newCmd)) {
                _cmdQueue[_cmdTail] = "MISSION_LOADED";
                _cmdTail = (_cmdTail + 1) % NRF_CMD_QUEUE_SIZE;
                _cmdCount++;
            } else {
                Serial.printf("❌ [NRF] Mission non valide (%d car.) : %s\n", newCmd.length(), newCmd.c_str());
            }
            return;
        }

        _cmdQueue[_cmdTail] = newCmd;
        _cmdTail = (_cmdTail + 1) % NRF_CMD_QUEUE_SIZE;
        _cmdCount++;
        Serial.printf("\n📡 [NRF] Commande validée : \"%s\"\n", newCmd.c_str());
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  _parseMissionString() - Le Cerveau du décodage
// ─────────────────────────────────────────────────────────────────────────────
bool NRF_Comm::_parseMissionString(String payload) {
    PATH_SIZE = 0;

    int headerEnd = payload.indexOf(';');
    if (headerEnd == -1) return false;

    // header contient "8,2,0.6,1.52,1.4,2.5,1.97"
    String header = payload.substring(1, headerEnd); 
    
    // 1. Extraire le NWP (ex: 8)
    int firstComma = header.indexOf(',');
    int expectedNWP = header.substring(0, firstComma).toInt();
    
    // 2. Extraire le reste SANS le NWP (ex: "2,0.6,1.52,1.4,2.5,1.97")
    String coords = header.substring(firstComma + 1);

    // 3. Découper les coordonnées
    int idx[6], i = 0;
    // ⚠️ CRITIQUE : On utilise 'coords' et plus 'header' ici !
    int current_comma = coords.indexOf(','); 
    while (current_comma != -1 && i < 5) {
        idx[i++] = current_comma;
        current_comma = coords.indexOf(',', current_comma + 1);
    }

    if (i == 5) {
        float newStartX = coords.substring(0, idx[0]).toFloat();
        float newStartY = coords.substring(idx[0]+1, idx[1]).toFloat();

        /* --- ON DÉSACTIVE CE TEST TROMPEUR ---
        if (mission_ready_to_start &&
            abs(newStartX - START_X) < 0.01f &&
            abs(newStartY - START_Y) < 0.01f) {
            Serial.println("🔁 [NRF] Mission déjà chargée, doublon ignoré.");
            return false;
        }
        */

        // On accepte et on écrase directement les variables avec la nouvelle mission
        START_X     = newStartX;
        START_Y     = newStartY;
        START_THETA = coords.substring(idx[1]+1, idx[2]).toFloat();
        GOAL_X      = coords.substring(idx[2]+1, idx[3]).toFloat();
        GOAL_Y      = coords.substring(idx[3]+1, idx[4]).toFloat();
        GOAL_THETA  = coords.substring(idx[4]+1).toFloat();
    }

    // 4. Extraire la liste des waypoints
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

    if (startIndex < payload.length() && PATH_SIZE < MAX_WAYPOINTS) {
        String wpStr = payload.substring(startIndex);
        int commaIndex = wpStr.indexOf(',');
        if (commaIndex != -1) {
            MISSION_PATH[PATH_SIZE].x = wpStr.substring(0, commaIndex).toFloat();
            MISSION_PATH[PATH_SIZE].y = wpStr.substring(commaIndex + 1).toFloat();
            PATH_SIZE++;
        }
    }

    // 5. VÉRIFICATION FINALE DE SÉCURITÉ
    if (PATH_SIZE != expectedNWP) {
        Serial.printf("❌ [NRF] Erreur WP : Attendu %d, reçu %d\n", expectedNWP, PATH_SIZE);
        return false;
    }

    mission_ready_to_start = true;
    Serial.printf("✅ [NRF] Mission validée : %d/%d WP reçus.\n", PATH_SIZE, expectedNWP);
    return true;
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
    _cmdHead = (_cmdHead + 1) % NRF_CMD_QUEUE_SIZE;
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