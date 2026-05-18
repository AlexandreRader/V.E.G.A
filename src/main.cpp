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
#define SIMULATION_MODE false // Mettre sur false quand le robot sera sur ses roues !

// --- Les "Cerveaux" Globaux de la navigation ---
EKFManager ekf;
PathFollower follower;
Kinematics kinematics;

void updateNavigationTask(float dt);

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
    Serial.println("T : Mode Terminal Série (Simulation Radio)");
    Serial.println("==========================================");
}

// 1. Fonction de décodage autonome (Copie de la logique NRF)
bool decoderMissionManuelle(String payload) {
    PATH_SIZE = 0; 
    
    int headerEnd = payload.indexOf(';');
    if (headerEnd == -1) return false;

    String header = payload.substring(1, headerEnd); 
    
    int idx[6], i = 0;
    int current_comma = header.indexOf(',');
    while (current_comma != -1 && i < 5) {
        idx[i++] = current_comma;
        current_comma = header.indexOf(',', current_comma + 1);
    }
    
    if (i == 5) { 
        START_X     = header.substring(0, idx[0]).toFloat();
        START_Y     = header.substring(idx[0]+1, idx[1]).toFloat();
        START_THETA = header.substring(idx[1]+1, idx[2]).toFloat();
        GOAL_X      = header.substring(idx[2]+1, idx[3]).toFloat();
        GOAL_Y      = header.substring(idx[3]+1, idx[4]).toFloat();
        GOAL_THETA  = header.substring(idx[4]+1).toFloat();
    } else {
        Serial.println("❌ En-tête de mission invalide !");
        return false;
    }

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
        Serial.printf("✅ Mission chargée (SÉRIE) : %d WP. Départ: [%.1f, %.1f, %.1f°]\n", 
                      PATH_SIZE, START_X, START_Y, START_THETA * 180.0/M_PI);
        return true;
    }
    return false;
}

// 2. La boucle du Mode Terminal
void modeTerminalSerie() {
    Serial.println("\n==========================================");
    Serial.println("💻 MODE TERMINAL SÉRIE (Simulateur NRF)");
    Serial.println("==========================================");
    Serial.println("Envoyez vos commandes : START, STOP ou trajectoire M...;*");
    Serial.println("Envoyez 'QUIT' pour revenir au menu principal.");
    Serial.println("==========================================");

    bool in_terminal = true;
    mission_active = false; 
    unsigned long last_time = millis();

    while (in_terminal) {
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0;

        // --- 1. Exécution de la Navigation (Si activée) ---
        if (mission_active && dt >= 0.02) { 
            last_time = now;
            updateNavigationTask(dt); 

            // Télémétrie toutes les 500ms
            static unsigned long last_print = 0;
            if (now - last_print > 500) {
                Serial.printf("📍 NAV | X: %.2f m | Y: %.2f m | Cap: %.1f°\n", 
                              ekf.X(0), ekf.X(1), ekf.X(2) * 180.0 / M_PI);
                last_print = now;
            }
            
            if (follower.isDone()) {
                Serial.println("\n✅ MISSION TERMINÉE AVEC SUCCÈS !");
                mission_active = false;
                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false);
            }
        } else if (!mission_active) {
            last_time = now; // Évite un bond dans le temps au démarrage
        }

        // --- 2. Écoute du Clavier (Port Série) ---
        if (Serial.available() > 0) {
            String cmd = Serial.readStringUntil('\n'); // Lit toute la ligne
            cmd.trim(); // Nettoie les espaces et les \r invisibles

            if (cmd.length() == 0) continue;

            if (cmd.equalsIgnoreCase("QUIT")) {
                in_terminal = false;
                mission_active = false;
                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false);
                Serial.println("Sortie du mode Terminal.");
            }
            else if (cmd.equalsIgnoreCase("START")) {
                if (mission_ready_to_start) {
                    Serial.println("🚀 DÉCOLLAGE : Mission activée !");
                    mission_active = true;
                    actuators.enableMotors(true);
                } else {
                    Serial.println("❌ Impossible : Aucune mission chargée !");
                }
            }
            else if (cmd.equalsIgnoreCase("STOP")) {
                Serial.println("🛑 ARRÊT D'URGENCE !");
                mission_active = false;
                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false);
            }
            else if (cmd.charAt(0) == 'M' || cmd.charAt(0) == 'm') {
                Serial.println("📡 Trajectoire reçue via Serial. Décodage...");
                if (decoderMissionManuelle(cmd)) {
                    ekf.reset(START_X, START_Y, START_THETA);
                    follower.resetMission();
                    Serial.println("🎯 Robot localisé et prêt. Tapez 'START'.");
                }
            }
            else {
                Serial.printf("❓ Commande inconnue : %s\n", cmd.c_str());
            }
        }
    }
}

void calibrerServosInteractif(ActuatorManager& actuators) {
    Serial.println("\n==========================================");
    Serial.println("🎯 MODE DE CALIBRATION INTERACTIF DES SERVOS");
    Serial.println("==========================================");
    Serial.println("Instructions :");
    Serial.println("1, 2, 3, 4 : Sélectionner le servo (FL, FR, RL, RR)");
    Serial.println("+ / -     : Ajuster l'angle d'offset (par pas de 1°)");
    Serial.println("S         : Sauvegarder et quitter");
    Serial.println("==========================================");

    int current_servo = 1; // 1:FL, 2:FR, 3:RL, 4:RR
    int offsets[4] = {0, 0, 0, 0}; // Offsets temporaires [FL, FR, RL, RR]
    
    // On applique la position d'origine (0 radian)
    actuators.setServoAngles(0, 0, 0, 0);
    actuators.enableMotors(true);

    bool calibrating = true;
    while (calibrating) {
        if (Serial.available() > 0) {
            char key = Serial.read();
            if (key == '\n' || key == '\r') continue;

            switch (key) {
                case '1': current_servo = 1; Serial.println("👉 Servo sélectionné : Avant-Gauche (FL)"); break;
                case '2': current_servo = 2; Serial.println("👉 Servo sélectionné : Avant-Droit (FR)"); break;
                case '3': current_servo = 3; Serial.println("👉 Servo sélectionné : Arrière-Gauche (RL)"); break;
                case '4': current_servo = 4; Serial.println("👉 Servo sélectionné : Arrière-Droit (RR)"); break;

                case '+':
                case '=': // Pour certains claviers
                    offsets[current_servo - 1]++;
                    Serial.printf("🔧 Servo %d | Nouvel Offset : %d°\n", current_servo, offsets[current_servo - 1]);
                    break;

                case '-':
                    offsets[current_servo - 1]--;
                    Serial.printf("🔧 Servo %d | Nouvel Offset : %d°\n", current_servo, offsets[current_servo - 1]);
                    break;

                case 's':
                case 'S':
                    calibrating = false;
                    break;
            }

            // Application en temps réel de la correction sur le robot
            // On convertit les degrés d'offset temporaires en radians pour s'accorder avec setServoAngles
            float fl_rad = (offsets[0] * M_PI) / 180.0;
            float fr_rad = (offsets[1] * M_PI) / 180.0;
            float rl_rad = (offsets[2] * M_PI) / 180.0;
            float rr_rad = (offsets[3] * M_PI) / 180.0;
            
            actuators.setServoAngles(fl_rad, fr_rad, rl_rad, rr_rad);
        }
    }

    // Affichage du résultat final prêt à être copié
    Serial.println("\n==========================================");
    Serial.println("✅ CALIBRATION TERMINÉE !");
    Serial.println("Copiez ces lignes dans votre fichier config.h :");
    Serial.println("==========================================");
    Serial.printf("#define OFFSET_SERVO_FL  %d\n", offsets[0]);
    Serial.printf("#define OFFSET_SERVO_FR  %d\n", offsets[1]);
    Serial.printf("#define OFFSET_SERVO_RL  %d\n", offsets[2]);
    Serial.printf("#define OFFSET_SERVO_RR  %d\n", offsets[3]);
    Serial.println("==========================================\n");
}


void testerMoteursInteractif(ActuatorManager& actuators) {
    Serial.println("\n==========================================");
    Serial.println("⚙️ MODE DE TEST INDIVIDUEL DES MOTEURS (STEPPERS)");
    Serial.println("==========================================");
    Serial.println("Instructions :");
    Serial.println("Taper un chiffre de 1 à 6 pour faire tourner un moteur :");
    Serial.println("  1 : Avant-Gauche   (FL) -> Index 0");
    Serial.println("  2 : Avant-Droit    (FR) -> Index 1");
    Serial.println("  3 : Milieu-Gauche  (ML) -> Index 2");
    Serial.println("  4 : Milieu-Droit   (MR) -> Index 3");
    Serial.println("  5 : Arrière-Gauche (RL) -> Index 4");
    Serial.println("  6 : Arrière-Droit  (RR) -> Index 5");
    Serial.println("Taper 'S' pour quitter ce mode.");
    Serial.println("==========================================");

    bool testing = true;
    long target = 200 * 16; // Fait exactement 1 tour complet (si tu es en 16 microsteps)

    while (testing) {
        if (Serial.available() > 0) {
            char key = Serial.read();
            if (key == '\n' || key == '\r') continue;

            int motor_index = -1;
            String motor_name = "";

            switch (key) {
                case '1': motor_index = 0; motor_name = "Avant-Gauche (FL)"; break;
                case '2': motor_index = 1; motor_name = "Avant-Droit (FR)"; break;
                case '3': motor_index = 2; motor_name = "Milieu-Gauche (ML)"; break;
                case '4': motor_index = 3; motor_name = "Milieu-Droit (MR)"; break;
                case '5': motor_index = 4; motor_name = "Arrière-Gauche (RL)"; break;
                case '6': motor_index = 5; motor_name = "Arrière-Droit (RR)"; break;
                case 's':
                case 'S':
                    testing = false;
                    Serial.println("✅ Sortie du test des moteurs.");
                    break;
                default:
                    Serial.println("⚠️ Touche invalide. Entrez un chiffre de 1 à 6, ou S pour quitter.");
                    break;
            }

            // Si un moteur valide a été sélectionné
            if (motor_index != -1) {
                Serial.printf("\n🚀 Lancement du moteur : %s...\n", motor_name.c_str());
                
                // On active la puissance
                actuators.enableMotors(true);
                delay(10);

                // On lance la rotation à 1000 Hz
                actuators.moveRelative(motor_index, target, 1000);

                // On attend que le moteur finisse son tour
                while(actuators.isMotorMoving(motor_index)) {
                    delay(50); 
                }

                // On coupe la puissance pour éviter la chauffe
                actuators.enableMotors(false);
                Serial.println("✅ Rotation terminée. (Testez un autre chiffre ou tapez S)");
            }
        }
    }
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
    float measured_omega = 0.0;

    // ==========================================
    // 1. ACQUISITION & ODOMÉTRIE (Où suis-je ?)
    // ==========================================
    if (SIMULATION_MODE) {
        // --- MODE SIMULATION PURE ---
        // On suppose que le robot virtuel a exécuté la vitesse ordonnée
        measured_vx = sim_vx; 
        measured_omega = sim_omega;

        // On simule la rotation de la boussole virtuelle
        sim_heading += measured_omega * dt; 
        while (sim_heading > M_PI) sim_heading -= 2.0 * M_PI;
        while (sim_heading < -M_PI) sim_heading += 2.0 * M_PI;

        // Prédiction EKF purement virtuelle
        ekf.predict(measured_vx, measured_omega, dt);
        // ekf.update(sim_heading); // On peut activer l'update virtuel si besoin

    } else {
        // --- MODE RÉEL ---
        imu.readMotion();
        imu.updateEulerAngles();
        tof.update(); 

        // 🎯 ODOMÉTRIE HYBRIDE INTÉLLIGENTE
        if (sim_vx == 0.0) {
            // 🔄 1. On est en train de pivoter sur place
            // On force la vitesse à 0 pour bloquer X et Y dans l'EKF
            measured_vx = 0.0; 
            
            // On rafraîchit les anciennes variables pour éviter un "saut de pas" au redémarrage
            last_steps_ML = actuators.getStepCount(2);
            last_steps_MR = actuators.getStepCount(3);
        } 
        else {
            // 🚀 2. On est censé avancer en ligne droite
            // On va lire la VRAIE quantité de pas générée par la bibliothèque moteur
            long current_steps_ML = actuators.getStepCount(2); 
            long current_steps_MR = actuators.getStepCount(3); 
            
            // Calcul des vitesses réelles de chaque côté
            float v_ML = ((current_steps_ML - last_steps_ML) * METERS_PER_STEP) / dt;
            float v_MR = ((current_steps_MR - last_steps_MR) * METERS_PER_STEP) / dt;
            
            // Sauvegarde pour le prochain cycle
            last_steps_ML = current_steps_ML;
            last_steps_MR = current_steps_MR;

            // La vitesse réelle est la moyenne des deux côtés
            measured_vx = (v_ML + v_MR) / 2.0; 
        }

        // Évolution de l'EKF indexé sur la vraie vitesse physique
        ekf.predict(measured_vx, imu.gyroZ, dt);
    }

    // ==========================================
    // 2. DÉCISION (Où dois-je aller ?)
    // ==========================================
    // Le Cerveau calcule la nouvelle trajectoire à partir de la position EKF estimée
    VelocityCommand cmd = follower.update(ekf.X(0), ekf.X(1), ekf.X(2));

    // On mémorise cette commande pour qu'elle devienne le "measured_vx" du PROCHAIN cycle
    sim_vx = cmd.linear_v;
    sim_omega = cmd.angular_w;

    // ==========================================
    // 3. SÉCURITÉ & ACTIONNEURS (Comment j'y vais ?)
    // ==========================================
    // Arrêt d'urgence ToF (Uniquement en réel pour ne pas coincer la simulation)
    // ==========================================
    // 3. SÉCURITÉ ET ACTIONNEURS (Comment j'y vais ?)
    // ==========================================
    // Arrêt d'urgence ToF (Uniquement en réel pour ne pas coincer la simulation)
    if (!SIMULATION_MODE && tof.emergencyStopRequired()) {
        cmd.linear_v = 0.0;
        cmd.angular_w = 0.0;
    }

    // Cinématique Inverse : Calcul des angles cibles théoriques (mc.angle_...)
    MotorCommands mc = kinematics.calculateDrive(cmd.linear_v, cmd.angular_w);
    
    // 🎯 CONFIGURATION DE LA RAMPE DES SERVOS
    // Vitesse maximale de rotation autorisée pour tes servos (en radians par seconde)
    // 1.5 rad/s correspond environ à 90° par seconde. Plus ce chiffre est bas, plus le mouvement sera lent et fluide.
    const float MAX_SERVO_SPEED_RAD_S = 1.5; 
    float max_angle_change = MAX_SERVO_SPEED_RAD_S * dt; // Le déplacement max autorisé pour ce cycle

    // Variables statiques pour mémoriser la position filtrée précédente
    static float filtered_FL = 0.0;
    static float filtered_FR = 0.0;
    static float filtered_RL = 0.0;
    static float filtered_RR = 0.0;

    // Application de la rampe : on limite l'écart entre la cible (mc.angle) et la position actuelle (filtered)
    filtered_FL += constrain(mc.angle_FL - filtered_FL, -max_angle_change, max_angle_change);
    filtered_FR += constrain(mc.angle_FR - filtered_FR, -max_angle_change, max_angle_change);
    filtered_RL += constrain(mc.angle_RL - filtered_RL, -max_angle_change, max_angle_change);
    filtered_RR += constrain(mc.angle_RR - filtered_RR, -max_angle_change, max_angle_change);

    // Envoi des angles LISSÉS aux 4 Servomoteurs
    actuators.setServoAngles(filtered_FL, filtered_FR, filtered_RL, filtered_RR);
    
    // Envoi des vitesses aux 6 Moteurs Pas-à-Pas (Traction)
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
                Serial.println("\n--- TEST DES 6 MOTEURS (1 Tour complet) ---");
                testerMoteursInteractif(actuators);
                /*
                // 1. Activation globale
                actuators.enableMotors(true);
                delay(10);

                long target = 200 * 16; 
                
                // 2. Lancement du mouvement pour les 6 moteurs (index 0 à 5)
                for(int i = 0; i < 6; i++) {
                    actuators.moveRelative(i, target, 1000); 
                }

                Serial.println("Rotation en cours des 6 roues...");
                
                // 3. Attente non-bloquante (on surveille le moteur 0, vu qu'ils finiront tous en même temps)
                while(actuators.isMotorMoving(0)) {
                    Serial.printf("Pas parcourus (Moteur 1) : %ld / %ld\r", actuators.getStepCount(0), target);
                    delay(50); 
                }
                
                Serial.println("\n✅ Tour terminé pour tous les moteurs.");
                
                // 4. Désactivation
                actuators.enableMotors(false);
                */
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
                //Serial.println("Balayage Servos (-45° -> +45° -> 0°)");
                //actuators.setServoAngles(-0.78, -0.78, -0.78, -0.78); delay(1500);
                //actuators.setServoAngles(0.78, 0.78, 0.78, 0.78); delay(1500);
                actuators.setServoAngles(0, 0, 0, 0);

                calibrerServosInteractif(actuators);

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
            case 't':
            case 'T':
                modeTerminalSerie();
                break;
            case 'k':
            case 'K': {
                Serial.println("\n--- TEST DE LA CINÉMATIQUE PURE ---");
                
                // 1. On ordonne au robot d'aller tout droit
                Serial.println("1. Ligne droite (v = 0.2 m/s, w = 0)");
                MotorCommands mc1 = kinematics.calculateDrive(0.2, 0.0);
                actuators.setServoAngles(mc1.angle_FL, mc1.angle_FR, mc1.angle_RL, mc1.angle_RR);
                Serial.printf("Angles générés : FL:%.2f FR:%.2f RL:%.2f RR:%.2f\n", 
                              mc1.angle_FL, mc1.angle_FR, mc1.angle_RL, mc1.angle_RR);
                delay(4000);

                // 2. On ordonne au robot de tourner sur place vers la GAUCHE
                Serial.println("2. Rotation sur place GAUCHE (v = 0, w = 0.5 rad/s)");
                MotorCommands mc2 = kinematics.calculateDrive(0.0, 0.5);
                actuators.setServoAngles(mc2.angle_FL, mc2.angle_FR, mc2.angle_RL, mc2.angle_RR);
                Serial.printf("Angles générés : FL:%.2f FR:%.2f RL:%.2f RR:%.2f\n", 
                              mc2.angle_FL, mc2.angle_FR, mc2.angle_RL, mc2.angle_RR);
                delay(4000);

                // 3. Retour à zéro
                Serial.println("3. Retour à zéro");
                actuators.setServoAngles(0, 0, 0, 0);
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







