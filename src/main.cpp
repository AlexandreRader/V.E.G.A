#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "IMUManager.h" 
#include "HardwareControl.h"
#include "PathFollower.h"
#include "Kinematics.h"
#include "/home/wankeur/Documents/Code/Github/V.E.G.A/mission_export.h" // Assure-toi que PATH_SIZE > 0 ici !
#include "NRF.h"
#include "Detection.h"
#include "config.h"

// --- Objets Globaux ---
IMUManager imu;
ActuatorManager actuators;
NRF_Comm nrf(PIN_RADIO_CE, PIN_SPI_CSN);
ToFManager tof;        // Notre nouveau gestionnaire d'obstacles
bool tof_ok = false;

// --- Statuts de sécurité ---
bool imu_ok = false;
bool nrf_ok = false;
bool actuators_ok = false;

void afficherMenu() {
    Serial.println("\n==========================================");
    Serial.println("🛠️ MENU DE TEST MATERIEL - VEGA SC317");
    Serial.println("==========================================");
    Serial.println("0 : Lire les capteurs ToF (Distances)");
    Serial.println("1 : Scanner le bus I2C");
    Serial.println("2 : Tester la LED RGB interne (Pin 38)");
    Serial.println("3 : Lire le capteur Infrarouge (Pin 48)");
    Serial.println("4 : Activer/Désactiver les moteurs (Pin 47)");
    Serial.println("5 : Faire un pas avec le Moteur 1");
    Serial.println("6 : Lire la Centrale Inertielle (IMU)");
    Serial.println("7 : Tester les Servos (Balayage simple)");
    Serial.println("8 : Afficher le statut des Actionneurs");
    Serial.println("9 : Lancer une Mission Test (Simulation complète)");
    Serial.println("R : Afficher le statut de la Radio (NRF24)");
    Serial.println("==========================================");
}

void setup() {
    Serial.begin(115200);
    while(!Serial); // Sécurité pour l'ESP32-S3
    delay(500); 
    
    Serial.println("\n🚀 Démarrage du Rover VEGA...");

    // 1. Bus de communication
    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CSN);
    
    // 2. Pins IR & Moteurs
    pinMode(PIN_IR_OUT, INPUT);
    pinMode(PIN_ENABLE_MOTORS, OUTPUT);
    digitalWrite(PIN_ENABLE_MOTORS, HIGH); // Moteurs désactivés par défaut

    // 3. Initialisation IMU
    Serial.print("Initialisation IMU... ");
    if (imu.begin()) {
        imu_ok = true;
        Serial.println("✅ OK");
    } else {
        Serial.println("⚠️ FAIL");
    }

    // 4. Initialisation Actionneurs
    Serial.print("Initialisation Actionneurs... ");
    if (actuators.begin()) {
        actuators_ok = true;
        Serial.println("✅ OK");
    } else {
        Serial.println("⚠️ FAIL");
    }

    // 5. Initialisation Radio
    Serial.print("Initialisation Radio... ");
    if (nrf.begin()) {
        nrf_ok = true;
        Serial.println("✅ OK");
    } else {
        Serial.println("⚠️ FAIL (Désactivée)");
    }
    // 6. Initialisation ToF
    Serial.print("Initialisation ToF... \n");
    if (tof.begin()) {
        tof_ok = true;
    } else {
        Serial.println("⚠️ FAIL (ToF)");
    }

    afficherMenu();
}

void loop() {
    // --- PARTIE RADIO (Tâche de fond non-bloquante) ---
    if (nrf_ok) {
        nrf.update();
        while (nrf.hasCommand()) {
            String cmd = nrf.readCommand();
            Serial.println("\n📡 [RADIO] Commande reçue : " + cmd);
            
            // Bonus : Si on reçoit "stop" par radio, on arrête les moteurs
            if (cmd == "stop") {
                actuators.setServoAngles(0, 0, 0, 0);
            }
        }
    }

    // --- PARTIE MENU SÉRIE ---
    if (Serial.available() > 0) {
        char choix = Serial.read();
        if (choix == '\n' || choix == '\r') return; 

        Serial.printf("\n--- Test %c ---\n", choix);

        switch (choix) {
            
            case '0': {
                if (!tof_ok) { 
                    Serial.println("⚠️ Les capteurs ToF ne sont pas initialisés."); 
                    break; 
                }
                Serial.println("\n--- LECTURE DES CAPTEURS ToF (Pendant 10 secondes) ---");
                Serial.println("Passez votre main devant les capteurs...");
                Serial.println("Tapez 's' et Entrée pour arrêter plus tôt.");
                
                // On boucle 50 fois avec un délai de 200ms = 10 secondes max
                for (int i = 0; i < 50; i++) {
                    // Arrêt manuel si on tape 's'
                    if (Serial.available() > 0 && Serial.read() == 's') {
                        Serial.println("\n🛑 Arrêt manuel de la lecture.");
                        break;
                    }

                    // 1. Mise à jour des données
                    tof.update(); 
                    
                    // 2. Affichage
                    tof.printStatus(); 
                    
                    // 3. Petit délai pour ne pas saturer la console (5 lectures par seconde)
                    delay(200); 
                }
                
                Serial.println("\n✅ Fin du test ToF.");
                break;
            }
            
            case '1': {
                Serial.println("Scan I2C...");
                int nb = 0;
                for(byte adr = 1; adr <= 127; adr++) {
                    Wire.beginTransmission(adr);
                    if (Wire.endTransmission() == 0) {
                        Serial.printf("Trouvé : 0x%02X\n", adr);
                        nb++;
                    }
                }
                if (nb == 0) Serial.println("⚠️ Rien trouvé.");
                break;
            }
            case '2': 
                Serial.println("LED Verte (1s)");
                neopixelWrite(38, 0, 255, 0); delay(1000); neopixelWrite(38, 0, 0, 0); 
                break;
            case '3':
                Serial.println("Lecture IR (2s)");
                for(int i=0; i<10; i++) { Serial.printf("IR: %d\n", digitalRead(PIN_IR_OUT)); delay(200); }
                break;
            case '4':
                digitalWrite(PIN_ENABLE_MOTORS, !digitalRead(PIN_ENABLE_MOTORS));
                Serial.printf("Moteurs : %s\n", digitalRead(PIN_ENABLE_MOTORS) ? "OFF (Libres)" : "ON (Bloqués)");
                break;
            case '5':
                if(actuators_ok) {
                    Serial.println("Un tour de moteur 1 !");
                    // À implémenter selon ton ActuatorManager si nécessaire
                }
                break;
            case '6': {
                if (!imu_ok) {
                    Serial.println("⚠️ L'IMU n'est pas initialisée.");
                    break;
                }
                Serial.println("\n--- LECTURE COMPLÈTE IMU (Brutes + Angles) ---");
                
                for(int i = 0; i < 50; i++) {
                    // 1. Lecture Accéléromètre et Gyroscope
                    imu.readMotion(); 
                    
                    // 2. Lecture Boussole ET calcul de Pitch, Roll et Cap
                    // (Ne JAMAIS appeler getRawHeading() manuellement ici !)
                    imu.updateEulerAngles();
                    
                    // 3. Affichage global sur une seule ligne
                    Serial.printf("Acc[X:%5.1f Y:%5.1f Z:%5.1f] | Gyr[X:%6.1f Y:%6.1f Z:%6.1f] | ROLL: %5.1f° | PITCH: %5.1f° | CAP: %5.1f°\n", 
                                  imu.accX, imu.accY, imu.accZ, 
                                  imu.gyroX, imu.gyroY, imu.gyroZ,
                                  imu.roll, imu.pitch, imu.heading);
                    
                    delay(100); // 10 lectures par seconde
                }
                break;
            }
            case '7':
                if (!actuators_ok) { Serial.println("Actionneurs HS."); break; }
                Serial.println("Balayage Servos (-45° -> +45° -> 0°)");
                actuators.setServoAngles(-0.78, -0.78, -0.78, -0.78); delay(1500);
                actuators.setServoAngles(0.78, 0.78, 0.78, 0.78); delay(1500);
                actuators.setServoAngles(0, 0, 0, 0);
                break;
            case '8': 
                if (actuators_ok) actuators.printStatus(); 
                break;
            
            case '9': {
                Serial.println("\n--- DÉBUT DE LA MISSION DE TEST ---");
                
                // PROTECTION ANTI-CRASH n°1 : Vérification de la taille du chemin
                if (PATH_SIZE <= 0) {
                    Serial.println("❌ ERREUR : PATH_SIZE est 0 dans mission_export.h !");
                    Serial.println("Impossible de lancer la mission (Division par zéro évitée).");
                    break;
                }

                PathFollower follower;
                Kinematics kinematics;
                follower.resetMission();

                float robot_x = START_X;
                float robot_y = START_Y;
                float robot_theta = START_THETA;
                
                unsigned long last_update = millis();
                bool mission_complete = false;

                Serial.println("⚠️ Le robot va simuler son déplacement et braquer ses roues !");
                Serial.println("Tapez 's' et Entrée pour stopper la mission en cours.");
                delay(2000); 

                int print_counter = 0; // Pour ne pas saturer la console

                while (!mission_complete) {
                    // Vérification Radio PENDANT la mission
                    if (nrf_ok) {
                        nrf.update();
                        if (nrf.hasCommand()) {
                            Serial.println("📡 [RADIO] Interruption reçue : " + nrf.readCommand());
                        }
                    }

                    // Arrêt d'urgence Série
                    if (Serial.available() > 0 && Serial.read() == 's') {
                        Serial.println("\n🛑 MISSION INTERROMPUE !");
                        break;
                    }

                    // Calcul du temps (dt)
                    unsigned long now = millis();
                    float dt = (now - last_update) / 1000.0;
                    last_update = now;
                    
                    // PROTECTION ANTI-CRASH n°2 : Éviter dt = 0
                    if (dt <= 0.001) dt = 0.01; 

                    // Suivi de trajectoire
                    VelocityCommand cmd = follower.update(robot_x, robot_y, robot_theta);

                    // Odométrie parfaite
                    robot_theta += cmd.angular_w * dt;
                    robot_x += cmd.linear_v * cos(robot_theta) * dt;
                    robot_y += cmd.linear_v * sin(robot_theta) * dt;

                    // Cinématique et action physique
                    MotorCommands motor_cmds = kinematics.calculateDrive(cmd.linear_v, cmd.angular_w);
                    actuators.setServoAngles(motor_cmds.angle_FL, motor_cmds.angle_FR, motor_cmds.angle_RL, motor_cmds.angle_RR);

                    // Affichage allégé (1 fois sur 5) pour ne pas lagger
                    if (print_counter++ % 5 == 0) {
                        Serial.printf("WP: %d/%d | POS[X:%.2f Y:%.2f T:%.1f°] | Cmd[V:%.2f W:%.2f]\n",
                            follower.getCurrentIndex() + 1, PATH_SIZE,
                            robot_x, robot_y, robot_theta * 180.0/PI,
                            cmd.linear_v, cmd.angular_w
                        );
                    }

                    if (follower.isDone()) {
                        Serial.println("\n✅ MISSION TERMINÉE AVEC SUCCÈS !");
                        mission_complete = true;
                    }

                    delay(100); 
                }

                // Fin de mission : retour au centre
                actuators.setServoAngles(0, 0, 0, 0);
                break;
            }

            case 'r':
            case 'R': 
                if (nrf_ok) nrf.printStatus(); 
                else Serial.println("Radio désactivée au démarrage.");
                break;
                
            default:
                Serial.println("⚠️ Choix non reconnu.");
                break;
        }

        // On réaffiche le menu une fois l'action finie (avec un petit délai propre)
        delay(500);
        afficherMenu();
    }
}