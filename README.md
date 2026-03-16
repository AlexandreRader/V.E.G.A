# 🚀 Prototype Rover 1/10 - Perseverance (Projet SC317)

![ESP32](https://img.shields.io/badge/ESP32-Dual--Core-red)
![PlatformIO](https://img.shields.io/badge/PlatformIO-Compatible-orange)
![FreeRTOS](https://img.shields.io/badge/FreeRTOS-RTOS-blue)
![C++](https://img.shields.io/badge/C++-OOP-green)

Ce dépôt contient le code source du prototype à l'échelle 1/10 du rover Perseverance, développé dans le cadre du projet d'intégration aux sciences de l'ingénieur 2026. 

Le rover est conçu pour évoluer de manière semi-autonome sur une maquette simulant le sol martien (plâtre, poussière de brique) et franchir des pentes ou obstacles.

## 📋 Table des matières
- [Fonctionnalités Principales](#-fonctionnalités-principales)
- [Architecture Matérielle](#-architecture-matérielle)
- [Architecture Logicielle (FreeRTOS)](#-architecture-logicielle-freertos)
- [Structure du Projet](#-structure-du-projet)
- [Installation et Configuration](#-installation-et-configuration)
- [Simuler avec Wokwi](#-simuler-avec-wokwi)

---

## ✨ Fonctionnalités Principales
- **Mobilité "Rocker-Bogie" :** 6 roues motrices indépendantes gérées par Steppers pour un couple maximal en franchissement.
- **Direction Ackermann :** 4 roues directrices (avant/arrière) pour des virages sans ripage.
- **Navigation Semi-Autonome (A*) :** Calcul de trajectoire embarqué vers la zone d'arrivée.
- **Détection de Pentes et Obstacles :** Fusion de données entre 4 capteurs ToF (laser) et une centrale inertielle (IMU).
- **Télémétrie sans fil :** Communication avec la station sol via NRF24L01 (paquets stricts de 32 octets).
- **Centrage final :** Détection de la cible (10 cm) via capteur Infrarouge.

---

## 🛠 Architecture Matérielle
* **Cerveau :** ESP32 DevKit V1 (Dual-Core, 240MHz)
* **Propulsion :** 6x Moteurs Pas-à-Pas + 6x Drivers 
* **Direction :** 4x Servomoteurs pilotés par module I2C
* **Détection :** * 4x Time-of-Flight
  * 1x IMU 6-axes
  * 1x Infrarouge
* **Communication :** Module Radio NRF24L01+ avec antenne SMA
* **Énergie :** Batterie LiPo 3S + Régulateur 

---

## 🧠 Architecture Logicielle (FreeRTOS)
Pour garantir la fluidité des moteurs tout en calculant des trajectoires complexes, le code exploite les deux cœurs de l'ESP32 via **FreeRTOS** :

* **CORE 0 (Stratégie & Comms) :**
  * `TaskComms` : Écoute/Envoi des paquets NRF24L01.
  * `TaskNavigation` : Algorithme A* et prise de décision.
* **CORE 1 (Temps Réel) :**
  * `TaskMotors` (Priorité Haute) : Génération des pulses pour les 6 steppers et gestion de la cinématique Ackermann.
  * `TaskSensors` (Priorité Moyenne) : Lecture I2C des ToF et de l'IMU à haute fréquence.

---

## 📂 Structure du Projet
Nous utilisons **PlatformIO** (et non l'IDE Arduino classique) pour permettre une programmation orientée objet propre et modulaire.

```text
Rover-Perseverance-110/
├── include/
│   └── config.h            # ⚙️ TOUTES LES PINS ET CONSTANTES (A modifier ici !)
├── lib/
│   ├── Comms/              # Gestion NRF24L01 et protocoles
│   ├── Detection/          # Lecture ToF, IMU et filtrage
│   ├── HardwareControl/    # Cinématique Ackermann, Steppers et Servos
│   └── Navigation/         # Algorithme A* et mapping
├── src/
│   └── main.cpp            # 🚀 Orchestrateur FreeRTOS (Setup et Tasks)
├── platformio.ini          # 📦 Configuration de compilation et dépendances
└── README.md