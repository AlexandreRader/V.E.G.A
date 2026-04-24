#include <Arduino.h>
#include <Wire.h>
#include "pins.h"
#include "IMUManager.h" 
#include "HardwareControl.h"
#include "PathFollower.h"
#include "Kinematics.h"
#include "../../V.E.G.A/lib/Communication/mission.h"
#include "NRF.h"
#include "Detection.h"
#include "config.h"
#include "EKFManager.h"
#include "mission.h"

// --- Objets Globaux ---
IMUManager imu;
ActuatorManager actuators;
NRF_Comm nrf(PIN_RADIO_CE, PIN_SPI_CSN);
ToFManager tof;        // Notre nouveau gestionnaire d'obstacles
bool tof_ok = false;
#define SIMULATION_MODE true // Mettre sur false quand le robot sera sur ses roues !

// --- Les "Cerveaux" Globaux de la navigation ---
EKFManager ekf;
PathFollower follower;
Kinematics kinematics;

// --- Variables globales pour l'odométrie ---
long last_steps_ML = 0;
long last_steps_MR = 0;
bool mission_active = false;

// --- Statuts de sécurité ---
bool imu_ok = false;
bool nrf_ok = false;
bool actuators_ok = false;


// ==========================================
// ALLOCATION EN MÉMOIRE RAM (Modifiables par la radio !)
// ==========================================
int PATH_SIZE = 0;
Waypoint MISSION_PATH[MAX_WAYPOINTS];
bool mission_ready_to_start = false;

float START_X = 0.0;
float START_Y = 0.0;
float START_THETA = 0.0;
float GOAL_X = 0.0;
float GOAL_Y = 0.0;
float GOAL_THETA = 0.0;


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
    Serial.println("C : Calibration de l'IMU ");
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


// --- LE CERVEAU DU ROVER (À appeler à fréquence fixe, ex: 50Hz) ---
// --- Paramètre Magique pour tester sur le bureau ---

// Variables pour le simulateur
float sim_vx = 0.0;
float sim_omega = 0.0;
float sim_heading = 0.0;

void updateNavigationTask(float dt) {
    float measured_vx = 0.0;

    // ==========================================
    // 1. ACQUISITION DES DONNÉES (Réel ou Simulé)
    // ==========================================
    if (SIMULATION_MODE) {
        // --- MODE SIMULATION ---
        // On suppose que le robot a parfaitement exécuté la consigne précédente
        measured_vx = sim_vx; 
        
        // On simule la rotation de la boussole (IMU)
        sim_heading += sim_omega * dt; 
        
        // On normalise le cap simulé entre -PI et PI
        while (sim_heading > M_PI) sim_heading -= 2.0 * M_PI;
        while (sim_heading < -M_PI) sim_heading += 2.0 * M_PI;

        // Prédiction EKF avec un faux Gyroscope
        ekf.predict(0.0, 0.0, sim_omega, dt);
        // Mise à jour EKF avec fausse boussole et fausse odométrie
        ekf.update(sim_heading, measured_vx, 0.0);

    } else {
        // --- MODE RÉEL ---
        imu.readMotion();
        imu.updateEulerAngles();
        tof.update(); 

        long current_steps_ML = actuators.getStepCount(2); 
        long current_steps_MR = actuators.getStepCount(3); 
        float v_ML = ((current_steps_ML - last_steps_ML) * METERS_PER_STEP) / dt;
        float v_MR = ((current_steps_MR - last_steps_MR) * METERS_PER_STEP) / dt;
        last_steps_ML = current_steps_ML;
        last_steps_MR = current_steps_MR;

        measured_vx = (v_ML + v_MR) / 2.0; 

        // EKF Réel
        ekf.predict(imu.accX, imu.accY, imu.gyroZ, dt);
        ekf.update(imu.heading, measured_vx, 0.0);
    }

    // ==========================================
    // 2. DÉCISION (Intelligence Artificielle)
    // ==========================================
    // On demande au PathFollower où aller selon notre position EKF actuelle
    VelocityCommand cmd = follower.update(ekf.X(0), ekf.X(1), ekf.X(2));

    // On mémorise la consigne pour le prochain tour du simulateur
    sim_vx = cmd.linear_v;
    sim_omega = cmd.angular_w;

    // ==========================================
    // 3. SÉCURITÉ ET ACTIONNEURS
    // ==========================================
    // Arrêt d'urgence ToF (uniquement en mode réel pour ne pas bloquer la simulation)
    if (!SIMULATION_MODE && tof.emergencyStopRequired()) {
        cmd.linear_v = 0;
        cmd.angular_w = 0;
    }

    // Cinématique Inverse (Calcul des angles et vitesses roues)
    MotorCommands mc = kinematics.calculateDrive(cmd.linear_v, cmd.angular_w);
    
    // Envoi aux actionneurs (même en simulation, les servos et moteurs branchés vont bouger !)
    actuators.setServoAngles(mc.angle_FL, mc.angle_FR, mc.angle_RL, mc.angle_RR);
    
    actuators.setStepperSpeeds(
        kinematics.speedToStepsHz(mc.speed_FL), kinematics.speedToStepsHz(mc.speed_FR),
        kinematics.speedToStepsHz(mc.speed_ML), kinematics.speedToStepsHz(mc.speed_MR),
        kinematics.speedToStepsHz(mc.speed_RL), kinematics.speedToStepsHz(mc.speed_RR)
    );
}


void loop() {
    // 1. Lecture radio
    nrf.update();

    // 2. Traitement des commandes radio (Le Lexique)
    if (nrf.hasCommand()) {
        String cmd = nrf.readCommand();

        if (cmd == "MISSION_LOADED") {
            // Le NRF a tout décodé. On initialise l'EKF avec notre position de départ !
            ekf.reset(START_X, START_Y, START_THETA); // ⚠️ Assure-toi d'avoir une fonction reset dans ton EKF
            follower.resetMission();
            Serial.println("🎯 Robot localisé et PathFollower prêt. En attente de 'START'...");
        } 
        
        else if (cmd == "START") {
            if (mission_ready_to_start) {
                Serial.println("🚀 DÉCOLLAGE : Mission activée !");
                mission_active = true;
                actuators.enableMotors(true);
            } else {
                Serial.println("❌ Impossible de démarrer : Aucune mission en mémoire !");
            }
        } 
        
        else if (cmd == "STOP") {
            Serial.println("🛑 ARRÊT D'URGENCE !");
            mission_active = false;
            actuators.setStepperSpeeds(0,0,0,0,0,0);
            actuators.enableMotors(false);
        } 
        
        else if (cmd == "CALIB") {
            imu.calibrateMagnetometer();
        }
        
        else {
            Serial.printf("❓ Commande inconnue : %s\n", cmd.c_str());
        }
    }

    // 3. Exécution de la tâche de navigation (Si active)
    if (mission_active) {
        static unsigned long last_time = millis();
        static unsigned long last_telemetry_time = millis(); // ⏱️ Nouveau timer pour l'affichage
        
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0;
        
        if (dt >= 0.02) { // Boucle de contrôle à 50Hz
            last_time = now;
            updateNavigationTask(dt); 
            
            // --- 📡 ENVOI DE LA TÉLÉMÉTRIE (Toutes les 500 ms) ---
            if (now - last_telemetry_time >= 500) {
                last_telemetry_time = now;
                
                // Affichage de la position estimée par le Filtre de Kalman
                Serial.printf("📍 NAV | X: %.2f m | Y: %.2f m | Cap: %5.1f° ", 
                              ekf.X(0), ekf.X(1), ekf.X(2) * 180.0 / M_PI);
                
                // Si ton PathFollower a une variable publique pour l'index du point (ex: current_wp),
                // tu peux l'afficher ici. (Adapte le nom de la variable si besoin)
                // Serial.printf("| Cible WP: %d/%d ", follower.current_wp, PATH_SIZE);
                
                Serial.println(); // Retour à la ligne
            }

            // On vérifie si on est arrivé à la fin de la mission
            if (follower.isDone()) {
                Serial.println("\n✅ MISSION TERMINÉE AVEC SUCCÈS ! Objectif atteint.");
                mission_active = false;
                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false); // Optionnel : couper le courant pour refroidir
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
                //Serial.println("LED Verte (1s)");
                //neopixelWrite(38, 0, 255, 0); delay(1000); neopixelWrite(38, 0, 0, 0); 
                //break;
            case '3':
                Serial.println("Lecture IR (2s)");
                for(int i=0; i<10; i++) { Serial.printf("IR: %d\n", digitalRead(PIN_IR_OUT)); delay(200); }
                break;
            case '4':
                digitalWrite(PIN_ENABLE_MOTORS, !digitalRead(PIN_ENABLE_MOTORS));
                Serial.printf("Moteurs : %s\n", digitalRead(PIN_ENABLE_MOTORS) ? "OFF (Libres)" : "ON (Bloqués)");
                break;
                
            case '5': {
                Serial.println("\n--- TEST MOTEUR 1 (1 Tour complet - Matériel) ---");
                
                // 1. Activation
                actuators.enableMotors(true);
                delay(10);

                // 2. Lancement du mouvement (3200 pas à 1000 Hz)
                // Le moteur tournera en tâche de fond sans bloquer l'ESP32
                long target = 200 * 16; 
                actuators.moveRelative(0, target, 1000); 

                Serial.println("Rotation en cours... (Mesure d'odométrie active)");
                
                // 3. Attente non-bloquante (on peut lire les pas pendant qu'il tourne)
                while(actuators.isMotorMoving(0)) {
                    Serial.printf("Pas parcourus : %ld / %ld\r", actuators.getStepCount(0), target);
                    delay(50); 
                }

                Serial.printf("\n✅ Tour terminé. Position finale : %ld pas.\n", actuators.getStepCount(0));
                
                // 4. Désactivation
                actuators.enableMotors(false);
                break;
            }

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
                    Serial.printf("Acc[X:%5.1f Y:%5.1f Z:%5.1f] | Gyr[X:%5.1f Y:%5.1f Z:%5.1f] | Mag[X:%6d Y:%6d Z:%6d] | CAP:%5.1f°\n",
                        imu.accX, imu.accY, imu.accZ,
                        imu.gyroX, imu.gyroY, imu.gyroZ,
                        imu.mag_x, imu.mag_y, imu.mag_z,
                        imu.heading * 180.0/M_PI);
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
                if (PATH_SIZE <= 0) {
                    Serial.println("❌ Erreur : mission_export.h est vide !");
                    break;
                }

                Serial.println("\n🚀 LANCEMENT DE LA MISSION AUTONOME");
                Serial.println("Appuyez sur 's' pour stopper à tout moment.");
                
                // Initialisation des états
                actuators.enableMotors(true);
                actuators.resetOdometry();
                follower.resetMission();
                last_steps_ML = 0; last_steps_MR = 0;
                
                unsigned long last_time = millis();
                mission_active = true;

                while (mission_active) {
                    // 1. Gestion du temps (dt)
                    unsigned long now = millis();
                    float dt = (now - last_time) / 1000.0;
                    if (dt < 0.02) continue; // On tourne à 50Hz max
                    last_time = now;

                    // 2. Exécution du cerveau
                    updateNavigationTask(dt);

                    // 3. Affichage du tableau de bord (toutes les 500ms)
                    static unsigned long last_print = 0;
                    if (now - last_print > 500) {
                        Serial.printf("POS [X:%.2f Y:%.2f T:%.1f°] | WP: %d/%d | ToF: %d mm\n",
                            ekf.X(0), ekf.X(1), ekf.X(2) * 180.0/PI,
                            follower.getCurrentIndex() + 1, PATH_SIZE,
                            tof.getFrontLeftDistance());
                        last_print = now;
                    }

                    // 4. Vérification de fin de mission
                    if (follower.isDone()) {
                        Serial.println("\n✅ Mission terminée ! Arrivée au dernier point.");
                        mission_active = false;
                    }

                    // 5. Interruption manuelle
                    if (Serial.available() > 0 && Serial.read() == 's') {
                        Serial.println("\n🛑 Mission interrompue par l'utilisateur.");
                        mission_active = false;
                    }
                }

                // Arrêt complet des moteurs en fin de test
                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false);
                Serial.println("Rover en sécurité.");
                break;
            }

            case 'r':
            case 'R': 
                if (nrf_ok) nrf.printStatus(); 
                else Serial.println("Radio désactivée au démarrage.");
                break;

            case 'C':
            case 'c': {
                if (imu_ok) {
                    imu.calibrateMagnetometer();
                } else {
                    Serial.println("❌ L'IMU n'est pas initialisée.");
                }
                break;
            }
                
            default:
                Serial.println("⚠️ Choix non reconnu.");
                break;
        }

        // On réaffiche le menu une fois l'action finie (avec un petit délai propre)
        delay(500);
        afficherMenu();
    }
}


