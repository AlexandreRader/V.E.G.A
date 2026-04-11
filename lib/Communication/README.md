# Module de Communication NRF24L01

Ce dossier contient l'implémentation du module de communication radio NRF24L01 pour le rover VEGA SC317.

## Fichiers

- `NRF.h` - Classe principale pour la réception de données
- `NRF.cpp` - Implémentation et exemples d'utilisation
- `NRF_Example.cpp` - Exemple complet d'intégration dans le programme principal

## Configuration Matérielle

Le module NRF24L01 utilise les pins suivants (définis dans `pins.h`) :

- **CE (Chip Enable)** : Pin 14
- **CSN (Chip Select)** : Pin 10
- **MOSI** : Pin 11
- **MISO** : Pin 13
- **SCK** : Pin 12

## Configuration Logicielle

### Bibliothèque Utilisée
- **esp-idf-mirf** : Fork adapté pour ESP32 de la bibliothèque Mirf originale

### Paramètres de Configuration
- **Canal** : 90 (0-127)
- **Adresse de réception** : "SERV1"
- **Taille payload** : 32 octets maximum

## Utilisation

### 1. Inclusion dans le code
```cpp
#include "Communication/NRF.h"
```

### 2. Initialisation
```cpp
void setup() {
    if (!nrf_receiver.begin()) {
        Serial.println("Erreur NRF24L01");
        while(1);
    }
}
```

### 3. Lecture de données

#### Messages texte
```cpp
String message;
if (nrf_receiver.readString(message)) {
    Serial.println(message);
}
```

#### Coordonnées (floats)
```cpp
float coords[3]; // x, y, theta
if (nrf_receiver.readFloatArray(coords, 3)) {
    float x = coords[0];
    float y = coords[1];
    float theta = coords[2];
}
```

#### Données brutes
```cpp
uint8_t buffer[32];
uint8_t length;
if (nrf_receiver.readData(buffer, length)) {
    // Traiter buffer[0..length-1]
}
```

## Protocole de Communication

### Types de Messages Supportés

1. **Messages texte** : Chaînes de caractères terminées par `\0`
2. **Coordonnées** : Tableaux de floats (x, y, theta)
3. **Données brutes** : Payloads personnalisés

### Exemples de Messages

- `"CMD:START"` - Démarrer la mission
- `"CMD:STOP"` - Arrêt d'urgence
- `"POS:1.23 4.56 0.78"` - Coordonnées x,y,theta
- Données binaires pour protocoles personnalisés

## Dépannage

### Problèmes Courants

1. **Pas de données reçues**
   - Vérifier l'alimentation du module (3.3V)
   - Contrôler les connexions SPI
   - Vérifier que l'émetteur utilise la même adresse/canal

2. **Données corrompues**
   - Vérifier la taille du payload (max 32 octets)
   - Contrôler l'endianness des données binaires

3. **Module non détecté**
   - Vérifier les pins dans `pins.h`
   - Tester avec un autre module NRF24L01

### Diagnostic

Utilisez `nrf_receiver.printStatus()` pour afficher la configuration actuelle.

## Côté Émetteur

Pour tester, utilisez un Arduino/ESP32 avec le code émetteur fourni dans les commentaires de `NRF.cpp`.

## Notes Techniques

- Le module fonctionne en mode réception continue
- Délai d'initialisation : 100ms
- Consommation : ~12mA en réception
- Portée : jusqu'à 100m en champ libre