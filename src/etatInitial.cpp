#include "etatInitial.h"

/**
 * Fonction exécutée pendant l'état initial
 * éteint la LED et passe à l'état ATTENTE sur appui du bouton
 * @param aucun
 * @return aucun
 */
void EtatInitial() {
  if(machine.executeOnce) {
    Serial.println("Entrée dans l'ÉTAT INITIAL");
    digitalWrite(PIN_LED, LOW);
  }
  
}

/**
 * Fonction de transition de l'état initial vers l'état d'attente
 * @param aucun
 * @return true si le bouton est pressé, false sinon
 */
bool transition_Initial_Attente() {
  return bouton.pressed();
}
