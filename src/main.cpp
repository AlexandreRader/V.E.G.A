#include <Arduino.h>

void setup() {
  // Initialisation du port série
  Serial.begin(115200);
  Serial.println("Démarrage du VEGA SC317...");
}

void loop() {
  Serial.println("Le robot est en attente d'ordres !");
  delay(2000);
}