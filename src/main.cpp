#include <Arduino.h>
#include <Wire.h>
#include "/home/wankeur/Documents/Code/Github/V.E.G.A/mission_export.h" // Généré par ta Station Sol
#include "../lib/Navigation/Navigation.h"     // Contrôleur principal intégré
#include "NRF.h"
// ==========================================
// SYSTÈME INTÉGRÉ VEGA SC317
// ==========================================

// Instance du contrôleur principal
NavigationController nav_controller;

// Variables pour les commandes série (debug)
String serial_command = "";

// Forward declarations
void processSerialCommand(String cmd);

// ==========================================
// FONCTIONS ARDUINO STANDARD
// ==========================================

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n🚀 VEGA SC317 - DÉMARRAGE SYSTÈME");
    Serial.println("=================================");

    // Initialisation complète du système
    if (nav_controller.initialize()) {
        Serial.println("✅ Système initialisé avec succès");
        Serial.println("Commandes disponibles:");
        Serial.println("  'start'  - Démarrer la mission");
        Serial.println("  'stop'   - Arrêter la mission");
        Serial.println("  'reset'  - Reset du système");
        Serial.println("  'status' - État du système");
        Serial.println("  'help'   - Cette aide");
    } else {
        Serial.println("❌ Échec de l'initialisation - Vérifiez les connexions");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("\n=== SYSTÈME PRÊT ===\n");
}

void loop() {
    // ==========================================
    // TRAITEMENT DES COMMANDES SÉRIE (DEBUG)
    // ==========================================
    while (Serial.available()) {
        char c = Serial.read();
        if (c == '\n' || c == '\r') {
            if (serial_command.length() > 0) {
                processSerialCommand(serial_command);
                serial_command = "";
            }
        } else {
            serial_command += c;
        }
    }

    // ==========================================
    // BOUCLE PRINCIPALE DE NAVIGATION
    // ==========================================
    nav_controller.update();

    // Délai pour éviter la surcharge CPU
    delay(10);
}

// ==========================================
// TRAITEMENT DES COMMANDES SÉRIE
// ==========================================

void processSerialCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();

    Serial.printf("Commande reçue: %s\n", cmd.c_str());

    if (cmd == "start") {
        nav_controller.startMission();
    } else if (cmd == "stop") {
        nav_controller.stopMission();
    } else if (cmd == "reset") {
        nav_controller.resetSystem();
    } else if (cmd == "status") {
        nav_controller.printSystemStatus();
    } else if (cmd == "help") {
        Serial.println("Commandes disponibles:");
        Serial.println("  start  - Démarrer la mission");
        Serial.println("  stop   - Arrêter la mission");
        Serial.println("  reset  - Reset du système");
        Serial.println("  status - État du système");
        Serial.println("  help   - Cette aide");
    } else {
        Serial.printf("Commande inconnue: %s\n", cmd.c_str());
    }
}

// ==========================================
// TRAITEMENT DE LA COMMUNICATION
// ==========================================


NRF_Comm nrf;
QueueHandle_t xCmdQueue;  // Queue FreeRTOS vers TaskNavigation

void TaskComms(void* pvParameters) {
    nrf.begin();
    for (;;) {
        nrf.update();
        while (nrf.hasCommand()) {
            String cmd = nrf.readCommand();
            xQueueSend(xCmdQueue, &cmd, 0);  // → TaskNavigation
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
