#include <Arduino.h>
#include "pins.h"
#include <HybridNode.h> // Votre librairie

// Les états du rover
enum RoverState {
  IDLE,             // En attente d'un ordre
  CALCULATING_PATH, // Le A* tourne
  FOLLOWING_PATH,   // Les moteurs tournent
  EMERGENCY_STOP    // Le ToF a vu un fantôme non cartographié !
};

RoverState currentState = IDLE;
// (Variables fictives pour l'exemple)
std::vector<Point> currentPath; 
int pathIndex = 0;

void setup() {
  // Initialisation (Moteurs, ToF, WiFi pour la station sol...)
}

void loop() {
  // ==========================================
  // COUCHE 1 : LE RÉFLEXE (Priorité Absolue)
  // ==========================================
  // On lit les ToF à chaque tour de boucle (ultra rapide)
  float distanceToF = readFrontToF(); 

  // Si on roule et qu'un obstacle imprévu surgit à moins de 30 cm
  if (currentState == FOLLOWING_PATH && distanceToF < 0.3) {
      // COUPURE IMMÉDIATE DES MOTEURS (via la broche ENABLE)
      digitalWrite(PIN_ENABLE_MOTORS, HIGH);
      currentState = EMERGENCY_STOP;
      Serial.println("ERREUR DE CARTE ! Obstacle fantôme détecté par ToF. Arrêt.");
      // Ici, on pourrait envoyer un ping à la station sol pour demander une nouvelle Cost Map
  }

  // ==========================================
  // COUCHE 2 : LE CERVEAU (Machine à états)
  // ==========================================
  switch (currentState) {
    
    case IDLE:
      // Si on reçoit un objectif X/Y de la station sol
      if (receivedNewTarget()) {
          currentState = CALCULATING_PATH;
      }
      break;

   case CALCULATING_PATH: {
      // 1. Charger les données du fichier .h
      auto mission = MissionManager::loadCurrentMission();

      Serial.printf("Départ: %.2f, %.2f | Cible: %.2f, %.2f\n", 
                    mission.startX, mission.startY, 
                    mission.goalX, mission.goalY);

      // 2. Initialiser l'Hybrid A* avec la carte statique
      HybridAStar planner(mission.costMap, MAP_WIDTH, MAP_HEIGHT, MAP_RESOLUTION);

      // 3. Lancer le calcul
      bool success = planner.findPath(
          mission.startX, mission.startY, mission.startTheta,
          mission.goalX, mission.goalY
      );

      if (success) {
          currentPath = planner.getPath();
          currentState = FOLLOWING_PATH;
      }
      break;

    case FOLLOWING_PATH:
      // On exécute la cinématique inverse pour suivre le chemin
      // (Envoi des impulsions aux moteurs via A4988)
      bool reachedWaypoint = driveToNextPoint(currentPath[pathIndex]);
      
      if (reachedWaypoint) {
          pathIndex++;
          if (pathIndex >= currentPath.size()) {
              Serial.println("Destination atteinte !");
              digitalWrite(PIN_ENABLE_MOTORS, HIGH); // Repos
              currentState = IDLE;
          }
      }
      break;

    case EMERGENCY_STOP:
      // Attente d'instructions de la station sol ou que l'obstacle disparaisse
      if (distanceToF > 0.5) {
          Serial.println("Voie libérée, reprise de la trajectoire.");
          digitalWrite(PIN_ENABLE_MOTORS, LOW);
          currentState = FOLLOWING_PATH;
      }
      break;
  }
}