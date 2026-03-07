#include "etatFinal.h"

/**
 * Fonction exécutée pendant l'état final
 * Print de l'état, éteint la LED et attend que le bouton soit pressé
 * @param aucun
 * @return aucun
 */
void EtatFinal() {
  if(machine.executeOnce) {
    Serial.println("Entrée dans l'ÉTAT FINAL");
    digitalWrite(PIN_LED, LOW);
    // oublier l'état du bouton pour éviter une transition immédiate si le bouton était déjà pressé en entrant dans cet état
    bouton.toggled();
  }
    
  // Exemple d'utilisation d'autres fonctionnalités de la librairie Button
  if (bouton.released()) {
    Serial.println("Bouton relâché dans l'état final");
  }
}

/**
 * Fonction de transition de l'état final vers l'état initial
 * @param aucun
 * @return true si le bouton est pressé, false sinon
 */
bool transition_Final_Initial() {
  return bouton.pressed();
}
