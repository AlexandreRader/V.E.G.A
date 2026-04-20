#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "IMUManager.h" 
#include "HardwareControl.h"

IMUManager imu;
bool imu_ok = false;
ActuatorManager actuators;



void afficherMenu() {
    Serial.println("\n==========================================");
    Serial.println("🛠️ MENU DE TEST MATERIEL - VEGA SC317");
    Serial.println("==========================================");
    Serial.println("1 : Scanner le bus I2C");
    Serial.println("2 : Tester la LED RGB interne (Pin 38)");
    Serial.println("3 : Lire le capteur Infrarouge (Pin 48)");
    Serial.println("4 : Activer/Désactiver les moteurs (Pin 47)");
    Serial.println("5 : Faire un pas avec le Moteur 1");
    Serial.println("6 : Lire la Centrale Inertielle (IMU)");
    Serial.println("7 : Tester les Servos (Balayage)");
    Serial.println("8 : Afficher le statut des Actionneurs");
    Serial.println("==========================================");
}

void setup() {
    Serial.begin(115200);
    delay(2000); 

    Serial.println("\n\nInitialisation du système VEGA...");

    // 1. Initialisation du bus I2C (Unique pour tout le monde)
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    
    // 2. Configuration des pins directs
    pinMode(PIN_IR_OUT, INPUT);


    // 3. Initialisation de l'IMU
    Serial.println("Initialisation de l'IMU...");
    if (imu.begin()) {
        imu_ok = true;
        Serial.println("✅ IMU prête.");
    } else {
        Serial.println("⚠️ Echec de l'IMU.");
    }

    // 4. Initialisation des Actionneurs (CORRECT)
    // Cette fonction va appeler pwm.init(0x7F) et configurer les TMC2208
    Serial.println("Initialisation des actionneurs (Servos + Moteurs)...");
    if (actuators.begin()) {
        Serial.println("✅ Actionneurs prêts (PCA9685 détecté).");
    } else {
        Serial.println("⚠️ Echec PCA9685 ou Drivers Moteurs.");
    }

    afficherMenu();
}

void loop() {
    if (Serial.available() > 0) {
        char choix = Serial.read();
        if (choix == '\n' || choix == '\r') return; 

        Serial.printf("\n--- Exécution du test %c ---\n", choix);

        switch (choix) {
            case '1': {
                byte erreur, adresse;
                int nbComposants = 0;
                Serial.println("Scan I2C en cours...");
                for(adresse = 1; adresse <= 127; adresse++) {
                    Wire.beginTransmission(adresse);
                    erreur = Wire.endTransmission();
                    if (erreur == 0) {
                        Serial.printf("Composant trouvé à l'adresse: 0x%02X\n", adresse);
                        nbComposants++;
                    }
                }
                if (nbComposants == 0) Serial.println("⚠️ Aucun composant I2C trouvé !");
                else Serial.println("✅ Scan I2C terminé.");
                break;
            }
            case '2': {
                Serial.println("LED Vert...");
                neopixelWrite(38, 0, 255, 0); 
                delay(1000);
                neopixelWrite(38, 0, 0, 0);
                break;
            }
            case '3': {
                Serial.println("Lecture IR (5s) :");
                for (int i = 0; i < 20; i++) {
                    Serial.printf("État IR : %d\n", digitalRead(PIN_IR_OUT));
                    delay(250);
                }
                break;
            }
            case '4': {
                Serial.println("Activation Moteurs (LOW)");
                digitalWrite(PIN_ENABLE_MOTORS, LOW);
                delay(3000);
                Serial.println("Désactivation Moteurs (HIGH)");
                digitalWrite(PIN_ENABLE_MOTORS, HIGH);
                break;
            }
            case '5': {
                Serial.println("Moteur 1 : 200 pas en avant...");
                digitalWrite(PIN_ENABLE_MOTORS, LOW); 
                digitalWrite(PIN_DIR_M1, HIGH);       
                for (int i = 0; i < 200; i++) {
                    digitalWrite(PIN_STEP_M1, HIGH);
                    delayMicroseconds(1000);
                    digitalWrite(PIN_STEP_M1, LOW);
                    delayMicroseconds(1000);
                }
                digitalWrite(PIN_ENABLE_MOTORS, HIGH); 
                break;
            }
            case '6': {
                if (!imu_ok) {
                    Serial.println("⚠️ L'IMU n'est pas initialisée.");
                    break;
                }
                Serial.println("Lecture Physique IMU :");
                
                for(int i = 0; i < 25; i++) {
                    imu.readMotion(); // Met à jour Accel + Gyro
                    float cap_raw = imu.getRawHeading(); // Met à jour Boussole
                    
                    Serial.printf("Acc[X:%5.1f Y:%5.1f Z:%5.1f] | Gyr[X:%6.1f Y:%6.1f Z:%6.1f] | ", 
                                  imu.accX, imu.accY, imu.accZ, 
                                  imu.gyroX, imu.gyroY, imu.gyroZ);
                    
                    if (cap_raw < 0) {
                        Serial.println("Cap: ATTENTE...");
                    } else {
                        Serial.printf("Cap: %5.1f°\n", cap_raw * (180.0 / PI));
                    }
                    delay(200);
                }
                break;
            }
            // --- NOUVEAU TEST : LE SERVO ---
            case '7': {
                Serial.println("Test de balayage des 4 servos de direction...");
                // Note : On utilise maintenant uniquement les fonctions d'actuators
                Serial.println(" -> Braquage Gauche (-45°)");
                actuators.setServoAngles(-0.78, -0.78, -0.78, -0.78);
                delay(1500);

                Serial.println(" -> Braquage Droite (+45°)");
                actuators.setServoAngles(0.78, 0.78, 0.78, 0.78);
                delay(1500);

                Serial.println(" -> Retour au centre (0°)");
                actuators.setServoAngles(0, 0, 0, 0);
                break;
            }

            case '8': {
                // On décommente cette fonction pour voir l'état des vitesses
                actuators.printStatus();
                break;
            }

            default:
                Serial.println("⚠️ Choix non reconnu.");
                break;
        }
        delay(1000);
        afficherMenu();
    }
}



/*
#include <Arduino.h>
#include <Wire.h>
#include "/home/wankeur/Documents/Code/Github/V.E.G.A/mission_export.h" 
#include "../lib/Navigation/Navigation.h"
#include "NRF.h"

// ==========================================
// SYSTÈME INTÉGRÉ VEGA SC317
// ==========================================

NavigationController nav_controller;
String serial_command = "";

// Variables pour le clignotement non-bloquant
unsigned long previousLedMillis = 0;
bool ledState = false;

// Objets FreeRTOS
NRF_Comm nrf;
QueueHandle_t xCmdQueue;  // Queue FreeRTOS

// Déclarations
void processSerialCommand(String cmd);
void TaskComms(void* pvParameters);

// ==========================================
// SETUP
// ==========================================
void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\n🚀 VEGA SC317 - DÉMARRAGE SYSTÈME");
    Serial.println("=================================");

    // 1. Initialisation de la file d'attente (Queue) pour 10 messages de 32 caractères
    // On n'utilise PAS de "String" dans une Queue !
    xCmdQueue = xQueueCreate(10, sizeof(char[32])); 

    // 2. Lancement de la tâche de communication sur le Coeur 0 (Libère le Coeur 1 pour la Navigation)
    xTaskCreatePinnedToCore(
        TaskComms,      // Fonction à exécuter
        "TaskRadio",    // Nom pour le debug
        4096,           // Taille de la mémoire allouée (Stack)
        NULL,           // Paramètres
        1,              // Priorité (1 = normale)
        NULL,           // Handle
        0               // Exécuté sur le Core 0
    );

    // 3. Initialisation de la navigation
    if (nav_controller.initialize()) {
        Serial.println("✅ Système de navigation initialisé");
    } else {
        Serial.println("❌ Échec initialisation. Arrêt.");
        while (true) { delay(1000); }
    }

    Serial.println("\n=== SYSTÈME PRÊT ===\n");
}

// ==========================================
// LOOP PRINCIPAL (Coeur 1 par défaut)
// ==========================================
void loop() {
    // 1. Traitement Série (Debug)
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

    // 2. Mise à jour de la navigation (Doit tourner à VITESSE MAXIMALE)
    nav_controller.update();

    // 3. Gestion de la LED RGB (SANS DELAY)
    unsigned long currentMillis = millis();
    if (currentMillis - previousLedMillis >= 500) {
        previousLedMillis = currentMillis; // Sauvegarde le temps
        ledState = !ledState;              // Inverse l'état
        
        if (ledState) {
            neopixelWrite(38, 0, 255, 0); // Allume Vert
        } else {
            neopixelWrite(38, 0, 0, 0);   // Eteint
        }
    }

    // Petit délai de 1ms pour stabiliser le processeur (Evite le Watchdog Panic)
    delay(1);
}

// ==========================================
// FONCTIONS COMMANDES SERIE
// ==========================================
void processSerialCommand(String cmd) {
    cmd.trim();
    cmd.toLowerCase();
    Serial.printf("Commande reçue: %s\n", cmd.c_str());

    if (cmd == "start") { nav_controller.startMission(); }
    else if (cmd == "stop") { nav_controller.stopMission(); }
    else if (cmd == "reset") { nav_controller.resetSystem(); }
    else if (cmd == "status") { nav_controller.printSystemStatus(); }
    else { Serial.println("Commande inconnue."); }
}

// ==========================================
// TÂCHE FREERTOS : COMMUNICATION (Coeur 0)
// ==========================================
void TaskComms(void* pvParameters) {
    nrf.begin();
    
    // Buffer temporaire pour extraire le texte
    char cmdBuffer[32]; 

    for (;;) {
        nrf.update();
        
        while (nrf.hasCommand()) {
            String cmdString = nrf.readCommand();
            
            // On copie le String dans un tableau de char sécurisé avant envoi
            cmdString.toCharArray(cmdBuffer, 32);
            
            // Envoi dans la Queue (Ne plantera pas la RAM)
            xQueueSend(xCmdQueue, &cmdBuffer, 0); 
        }
        
        // Délai obligatoire dans une tâche FreeRTOS pour ne pas bloquer le coeur
        vTaskDelay(pdMS_TO_TICKS(10)); 
    }
}
*/