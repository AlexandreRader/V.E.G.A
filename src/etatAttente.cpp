#include "etatAttente.h"

/**
 * Fonction exécutée pendant l'état d'attente
 * Allume la LED et passe à l'état ACTION après 3 secondes
 * @param aucun
 * @return aucun
 */
void EtatAttente() {
  if(machine.executeOnce) {
    Serial.println("Entrée dans l'ÉTAT ATTENTE");
    tempsDebut = millis();
    digitalWrite(PIN_LED, HIGH);
  }
  
}

/**
 * Fonction de transition de l'état d'attente vers l'état initial
 * @param aucun
 * @return true si le bouton est pressé, false sinon
 */
bool transition_Attente_Initial() {
  return bouton.pressed();
}

/**
 * Fonction de transition de l'état d'attente vers l'état d'action
 * @param aucun
 * @return true si le délai est écoulé, false sinon
 */
bool transition_Attente_Action() {
  return (millis() - tempsDebut >= DELAI_ATTENTE);
}
