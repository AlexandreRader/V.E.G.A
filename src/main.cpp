#include "config.h"


#include "etatInitial.h"
#include "etatAttente.h"
#include "etatAction.h"
#include "etatFinal.h"

// Définition des variables globales
StateMachine machine = StateMachine();
Button bouton(PIN_BOUTON);
unsigned long tempsDebut = 0;

// Création des états
State* etatInitial = machine.addState(&EtatInitial);
State* etatAttente = machine.addState(&EtatAttente);
State* etatAction = machine.addState(&EtatAction);
State* etatFinal = machine.addState(&EtatFinal);

void setup() {
  // Initialisation de la communication série
  Serial.begin(9600);
  Serial.println("Démarrage de la machine d'état");
  
  // Configuration des broches
  pinMode(PIN_LED, OUTPUT);
  // Pas besoin de configurer PIN_BOUTON car la librairie Button s'en charge
  bouton.begin();
  
  // Configuration des transitions
  etatInitial->addTransition(&transition_Initial_Attente, etatAttente);
  etatAttente->addTransition(&transition_Attente_Action, etatAction);
  etatAttente->addTransition(&transition_Attente_Initial, etatInitial);
  etatAction->addTransition(&transition_Action_Final, etatFinal);
  etatFinal->addTransition(&transition_Final_Initial, etatInitial);
  
  // Démarrer la machine dans l'état initial
  machine.run();
}

void loop() {
  // Mettre à jour la machine d'état
  machine.run();
  
  // Petit délai pour éviter une utilisation excessive du CPU
  delay(DELAI_BOUCLE_MS);
}
