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
float initial_mission_theta = 0.0;

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
    int current_comma = header.indexOf(',', current_comma + 1);
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
                actuators.beep(600);
                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false);
                actuators.relaxServos();
            }
        } else if (!mission_active) {
            last_time = now; // Évite un bond dans le temps au démarrage
        }

        // --- 2. Écoute du Clavier (Port Série) ---
        if (Serial.available() > 0) {
            String cmd = Serial.readStringUntil('\n'); 
            cmd.trim(); 

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
                actuators.relaxServos();
            }
            else if (cmd.charAt(0) == 'M' || cmd.charAt(0) == 'm') {
                Serial.println("📡 Trajectoire reçue via Serial. Décodage...");
                if (decoderMissionManuelle(cmd)) {
                    initial_mission_theta = START_THETA;
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

// 🎯 Fonction pour obtenir l'inclinaison combinée du robot en degrés
float getCombinedTiltDeg() {
    // roll et pitch sont déjà mis à jour en radians dans imu.updateEulerAngles()
    float p = imu.pitch;
    float r = imu.roll;
    
    // Calcul de la magnitude de l'inclinaison en radians, puis conversion en degrés
    float tilt_rad = sqrt(p * p + r * r);
    return tilt_rad * (180.0 / M_PI);
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

    int current_servo = 1; 
    int offsets[4] = {0, 0, 0, 0}; 
    
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
                case '=': 
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

            float fl_rad = (offsets[0] * M_PI) / 180.0;
            float fr_rad = (offsets[1] * M_PI) / 180.0;
            float rl_rad = (offsets[2] * M_PI) / 180.0;
            float rr_rad = (offsets[3] * M_PI) / 180.0;
            
            actuators.setServoAngles(fl_rad, fr_rad, rl_rad, rr_rad);
        }
    }

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
    long target = 200 * 16; 

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

            if (motor_index != -1) {
                Serial.printf("\n🚀 Lancement du moteur : %s...\n", motor_name.c_str());
                actuators.enableMotors(true);
                delay(10);

                actuators.moveRelative(motor_index, target, 1000);

                while(actuators.isMotorMoving(motor_index)) {
                    delay(50); 
                }

                actuators.enableMotors(false);
                Serial.println("✅ Rotation terminée. (Testez un autre chiffre ou tapez S)");
            }
        }
    }
}


void setup() {
    Serial.begin(115200);
    //while(!Serial); 
    delay(3000); 
    
    Serial.println("\n🚀 Démarrage du Rover VEGA...");

    Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CSN);
    
    pinMode(PIN_IR_OUT, INPUT);
    pinMode(PIN_ENABLE_MOTORS, OUTPUT);
    digitalWrite(PIN_ENABLE_MOTORS, HIGH); 

    Serial.print("Initialisation IMU... ");
    if (imu.begin()) {
        imu_ok = true;
        Serial.println("✅ OK");
    } else {
        Serial.println("⚠️ FAIL");
    }

    Serial.print("Initialisation Actionneurs... ");
    if (actuators.begin()) {
        actuators_ok = true;
        Serial.println("✅ OK");
    } else {
        Serial.println("⚠️ FAIL");
    }

    Serial.print("Initialisation Radio... ");
    if (nrf.begin()) {
        nrf_ok = true;
        Serial.println("✅ OK");
    } else {
        Serial.println("⚠️ FAIL (Désactivée)");
    }

    Serial.print("Initialisation ToF... \n");
    if (tof.begin()) {
        tof_ok = true;
    } else {
        Serial.println("⚠️ FAIL (ToF)");
    }
    actuators.beep(150);
    afficherMenu();
}


// --- LE CERVEAU DU ROVER ---
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
        measured_vx = sim_vx; 
        measured_omega = sim_omega;

        sim_heading += measured_omega * dt; 
        while (sim_heading > M_PI) sim_heading -= 2.0 * M_PI;
        while (sim_heading < -M_PI) sim_heading += 2.0 * M_PI;

        ekf.predict(measured_vx, measured_omega, dt);
    } else {
        // --- MODE RÉEL OPTIMISÉ ASSERVI AUX COMPTEURS DE PAS ---
        imu.readMotion();
        imu.updateEulerAngles();
        tof.update(); 

        // 🎯 LECTURE DE L'AVANCE PHYSIQUE RÉELLE DES MOTEURS (Encodeurs)
        long current_steps_ML = actuators.getStepCount(2); // Compteur Milieu Gauche
        long current_steps_MR = actuators.getStepCount(3); // Compteur Milieu Droit
        
        // Calcul des vitesses réelles mesurées sur les roues (en m/s)
        float v_ML = ((current_steps_ML - last_steps_ML) * METERS_PER_STEP) / dt;
        float v_MR = ((current_steps_MR - last_steps_MR) * METERS_PER_STEP) / dt;
        
        // Sauvegarde pour le prochain cycle
        last_steps_ML = current_steps_ML;
        last_steps_MR = current_steps_MR;

        // La vitesse de l'EKF est la moyenne physique mesurée sur le sol
        measured_vx = (v_ML + v_MR) / 2.0; 

        // Sécurité : Si le PathFollower ordonne un arrêt complet (sim_vx == 0),
        // on force measured_vx à 0 pour éviter le bruit résiduel
        if (sim_vx == 0.0) {
            measured_vx = 0.0;
        }

        // 🔧 FILTRAGE PASSE-BAS DU GYROSCOPE (Low-Pass Filter)
        static float filtered_gyroZ = 0.05;
        float tau = 0.1; // Constante de temps (secondes)
        float alpha = dt / (dt + tau);
        filtered_gyroZ += (imu.gyroZ - filtered_gyroZ) * alpha;
        measured_omega = filtered_gyroZ;

        // L'EKF prédit la position UNIQUEMENT si les roues tournent physiquement !
        ekf.predict(measured_vx, measured_omega, dt);

        // Affichage épuré à 10Hz pour le débogage de navigation
        static unsigned long last_debug_print = 0;
        if (millis() - last_debug_print > 100) {
            Serial.printf("🔍 [NAV] V_Cmd: %.2f | V_Real: %.2f | GyroZ: %.3f rad/s | Cap: %.1f°\n", 
                          sim_vx, measured_vx, measured_omega, ekf.X(2) * 180.0 / M_PI);
            last_debug_print = millis();
        }
    }

    // ==========================================
    // 2. DÉCISION (Où dois-je aller ?)
    // ==========================================
    VelocityCommand cmd = follower.update(ekf.X(0), ekf.X(1), ekf.X(2));

    // ==========================================
    // 🎯 SÉCURITÉ FRANCHISSEMENT DE RELIEF (IMU)
    // ==========================================
    float tilt = getCombinedTiltDeg(); // Récupération de l'angle combiné (Pitch + Roll)
    float k_terrain = 1.0;             // Facteur multiplicateur par défaut ( Terrain plat )

    const float TILT_THRESHOLD_START = 6.0;  // À partir de 6° d'inclinaison, on commence à freiner
    const float TILT_THRESHOLD_MAX = 22.0;   // À 22° d'inclinaison, on est au ralentissement maximum
    const float MIN_SPEED_FACTOR = 0.15;     // Vitesse plancher (35%) pour garder du couple sans caler

    if (tilt > TILT_THRESHOLD_START) {
        // Calcul d'une rampe linéaire entre les deux seuils d'inclinaison
        float truncation = (tilt - TILT_THRESHOLD_START) / (TILT_THRESHOLD_MAX - TILT_THRESHOLD_START);
        k_terrain = 1.0 - truncation * (1.0 - MIN_SPEED_FACTOR);
        k_terrain = constrain(k_terrain, MIN_SPEED_FACTOR, 1.0);
        
        // Affichage à 1Hz dans le terminal pour suivre le comportement en relief
        static unsigned long last_tilt_print = 0;
        if (millis() - last_tilt_print > 1000) {
            Serial.printf("⚠️ [IMU RELIEF] Châssis incliné à %.1f° | Vitesse bridée à %.0f%%\n", tilt, k_terrain * 100.0);
            last_tilt_print = millis();
        }
    }

    // On applique le coefficient sur la vitesse linéaire
    cmd.linear_v = cmd.linear_v * k_terrain;

    // On mémorise la commande finale pour le prochain cycle de l'EKF
    sim_vx = cmd.linear_v;
    sim_omega = cmd.angular_w;

    // ==========================================
    // 3. SÉCURITÉ & ACTIONNEURS (Comment j'y vais ?)
    // ==========================================
    if (!SIMULATION_MODE && tof.emergencyStopRequired()) {
        cmd.linear_v = 0.0;
        cmd.angular_w = 0.0;
    }

    // Cinématique Inverse
    MotorCommands mc = kinematics.calculateDrive(cmd.linear_v, cmd.angular_w);

    // 🎯 CONFIGURATION DE LA RAMPE DES SERVOS (Slew Rate)
    const float MAX_SERVO_SPEED_RAD_S = 2.0;  // Vitesse max de rotation autorisée
    float max_angle_change = MAX_SERVO_SPEED_RAD_S * dt; 

    static float filtered_FL = 0.0;
    static float filtered_FR = 0.0;
    static float filtered_RL = 0.0;
    static float filtered_RR = 0.0;

    // Calcul de la rampe de lissage pour chaque servomoteur
    filtered_FL += constrain(mc.angle_FL - filtered_FL, -max_angle_change, max_angle_change);
    filtered_FR += constrain(mc.angle_FR - filtered_FR, -max_angle_change, max_angle_change);
    filtered_RL += constrain(mc.angle_RL - filtered_RL, -max_angle_change, max_angle_change);
    filtered_RR += constrain(mc.angle_RR - filtered_RR, -max_angle_change, max_angle_change);

    // Envoi UNIQUE et propre des angles LISSÉS aux 4 Servomoteurs
    actuators.setServoAngles(filtered_FL, filtered_FR, filtered_RL, filtered_RR);
    
    // Envoi des vitesses de traction calculées aux 6 moteurs pas-à-pas
    actuators.setStepperSpeeds(
        kinematics.speedToStepsHz(mc.speed_FL), kinematics.speedToStepsHz(mc.speed_FR),
        kinematics.speedToStepsHz(mc.speed_ML), kinematics.speedToStepsHz(mc.speed_MR),
        kinematics.speedToStepsHz(mc.speed_RL), kinematics.speedToStepsHz(mc.speed_RR)
    );
}

void loop() {
    nrf.update();

    if (nrf.hasCommand()) {
        String cmd = nrf.readCommand();

        if (cmd == "MISSION_LOADED") {
            ekf.reset(START_X, START_Y, START_THETA); 
            follower.resetMission();
            actuators.beep(80);
            delay(80);
            actuators.beep(80);
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
            for(int i = 0; i < 3; i++) {
                actuators.beep(100); // 100ms de son
                delay(150);          // 150ms de silence entre les bips
            }
        } 
        else if (cmd == "CALIB") {
            imu.calibrateMagnetometer();
        }
        else {
            Serial.printf("❓ Commande inconnue : %s\n", cmd.c_str());
        }
    }

    if (mission_active) {
        static unsigned long last_time = millis();
        static unsigned long last_telemetry_time = millis(); 
        
        unsigned long now = millis();
        float dt = (now - last_time) / 1000.0;
        
        if (dt >= 0.02) { 
            last_time = now;
            updateNavigationTask(dt); 
            
            if (now - last_telemetry_time >= 500) {
                last_telemetry_time = now;
                Serial.printf("📍 NAV | X: %.2f m | Y: %.2f m | Cap: %5.1f° \n", 
                              ekf.X(0), ekf.X(1), ekf.X(2) * 180.0 / M_PI);
            }

            if (follower.isDone()) {
        if (mission_active) {
            mission_active = false;
            
            // Arrêt physique immédiat
            actuators.setStepperSpeeds(0, 0, 0, 0, 0, 0);
            actuators.relaxServos(); // Coupe le PWM pour éviter le sifflement
            
            Serial.println("\n✅ MISSION TERMINÉE AVEC SUCCÈS ! Objectif atteint.");
            actuators.beep(600);
            
            // 🎯 RESET LOGIQUE SUR LES VRAIES VARIABLES DE TON DOSSIER DE CODE
            // MISSION_PATH[0] contient le point A exact (X et Y de départ)
            float reset_x = MISSION_PATH[0].x;
            float reset_y = MISSION_PATH[0].y;
            // Remplacer par :
            float reset_theta = ekf.X(2); // On conserve le cap actuel !
            ekf.reset(reset_x, reset_y, reset_theta);
            
            // Réinitialisation de l'index interne du PathFollower pour la prochaine mission
            follower.resetMission();
            
            // Remise à zéro des consignes de vitesse mémorisées
            sim_vx = 0.0;
            sim_omega = 0.0;
            
            Serial.printf("🔄 [RESET] Position recalée sur le départ initial -> X: %.2f m | Y: %.2f m | Cap: %.1f°\n", 
                          reset_x, reset_y, reset_theta * 180.0 / M_PI);
            Serial.println("📡 En attente d'une nouvelle commande 'START' par radio...\n");
        }
    }
        }
    }

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
                Serial.println("\n--- DASHBOARD TEMPS RÉEL : MATRICE ToF AVANT ---");
                Serial.println("Passez votre main devant les 4 coins du bloc avant...");
                Serial.println("Tapez 's' et Entrée pour arrêter le test.");
                delay(1500); 
                
                for (int i = 0; i < 100; i++) {
                    if (Serial.available() > 0 && Serial.read() == 's') {
                        Serial.println("\n🛑 Arrêt manuel de la lecture.");
                        break;
                    }
                    tof.update(); 
                    tof.printGridStatus(); 
                    delay(200); 
                }
                Serial.println("\n✅ Retour au menu principal.");
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
                break;
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
                break;
            }

            case '6': {
                if (!imu_ok) {
                    Serial.println("⚠️ L'IMU n'est pas initialisée.");
                    break;
                }
                Serial.println("\n--- LECTURE COMPLÈTE IMU (Brutes + Angles) ---");
                for(int i = 0; i < 50; i++) {
                    imu.readMotion(); 
                    imu.updateEulerAngles();
                    Serial.printf("Acc[X:%5.1f Y:%5.1f Z:%5.1f] | Gyr[X:%5.1f Y:%5.1f Z:%5.1f] | Mag[X:%6d Y:%6d Z:%6d] | CAP:%5.1f°\n",
                        imu.accX, imu.accY, imu.accZ,
                        imu.gyroX, imu.gyroY, imu.gyroZ,
                        imu.mag_x, imu.mag_y, imu.mag_z,
                        imu.heading * 180.0/M_PI);
                    delay(100); 
                }
                break;
            }
            case '7':
                if (!actuators_ok) { Serial.println("Actionneurs HS."); break; }
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
                
                actuators.enableMotors(true);
                actuators.resetOdometry();
                follower.resetMission();
                last_steps_ML = 0; last_steps_MR = 0;
                
                unsigned long last_time = millis();
                mission_active = true;

                while (mission_active) {
                    unsigned long now = millis();
                    float dt = (now - last_time) / 1000.0;
                    if (dt > 0.1) dt = 0.1; // Bride le dt à 100ms max pour éviter les sauts spatio-temporels
                    if (dt < 0.02) continue; 
                    last_time = now;

                    updateNavigationTask(dt);

                    static unsigned long last_print = 0;
                    if (now - last_print > 500) {
                        Serial.printf("POS [X:%.2f Y:%.2f T:%.1f°] | WP: %d/%d | ToF TL: %d mm\n",
                            ekf.X(0), ekf.X(1), ekf.X(2) * 180.0/M_PI,
                            follower.getCurrentIndex() + 1, PATH_SIZE,
                            tof.getTopLeftDistance());
                        last_print = now;
                    }

                    if (follower.isDone()) {
                        Serial.println("\n✅ Mission terminée ! Arrivée au dernier point.");
                        mission_active = false;
                        actuators.relaxServos();
                        actuators.enableMotors(false);
                        actuators.beep(600);
                    }

                    if (Serial.available() > 0 && Serial.read() == 's') {
                        Serial.println("\n🛑 Mission interrompue par l'utilisateur.");
                        mission_active = false;
                        actuators.relaxServos();
                        actuators.enableMotors(false);
                    }
                }

                actuators.setStepperSpeeds(0,0,0,0,0,0);
                actuators.enableMotors(false);
                actuators.relaxServos();
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
                Serial.println("1. Ligne droite (v = 0.2 m/s, w = 0)");
                MotorCommands mc1 = kinematics.calculateDrive(0.2, 0.0);
                actuators.setServoAngles(mc1.angle_FL, mc1.angle_FR, mc1.angle_RL, mc1.angle_RR);
                Serial.printf("Angles générés : FL:%.2f FR:%.2f RL:%.2f RR:%.2f\n", 
                              mc1.angle_FL, mc1.angle_FR, mc1.angle_RL, mc1.angle_RR);
                delay(4000);

                Serial.println("2. Rotation sur place GAUCHE (v = 0, w = 0.5 rad/s)");
                MotorCommands mc2 = kinematics.calculateDrive(0.0, 0.5);
                actuators.setServoAngles(mc2.angle_FL, mc2.angle_FR, mc2.angle_RL, mc2.angle_RR);
                Serial.printf("Angles générés : FL:%.2f FR:%.2f RL:%.2f RR:%.2f\n", 
                              mc2.angle_FL, mc2.angle_FR, mc2.angle_RL, mc2.angle_RR);
                delay(4000);

                Serial.println("3. Retour à zéro");
                actuators.setServoAngles(0, 0, 0, 0);
                break;
            }
            default:
                Serial.println("⚠️ Choix non reconnu.");
                break;
        }

        delay(500);
        afficherMenu();
    }
}