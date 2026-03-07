# Machine d'État - Projet Exemple

Ce projet pourra vous servir de base pour la programmation de vos véhicules autonomes. Il montre comment utiliser une machine d'état avec la librairie StateMachine et la gestion des boutons avec la librairie Button.

## Librairies utilisées

### StateMachine
La librairie StateMachine (https://github.com/jrullan/StateMachine) permet de créer facilement une machine d'état en définissant :
- Des états avec leurs fonctions associées (entrée, pendant, sortie)
- Des transitions entre ces états

### Button
La librairie Button (https://registry.platformio.org/libraries/mark170987/Button) offre une gestion avancée des boutons :
- Gestion automatique du debouncing
- Détection de différents événements (pression, relâchement, pression longue)
- Configuration automatique des pins en mode INPUT_PULLUP

Voici quelques exemples d'utilisation de la librairie Button :

- `bouton.pressed()` : Retourne `true` uniquement lorsque le bouton est pressé pour la première fois (déclenché sur un front montant).
- `bouton.released()` : Retourne `true` uniquement lorsque le bouton est relâché pour la première fois (déclenché sur un front descendant).
- `bouton.toggled()` : Retourne `true` chaque fois que l'état du bouton change (pression ou relâchement).
- `bouton.read()` : Retourne l'état actuel du bouton (déclenché ou relâché) après détection de rebond.
- `bouton.has_changed()` : Indique si l'état du bouton a changé depuis le dernier appel à `read()`.


## Fonctionnement de l'exemple

Cette machine d'état simple comporte quatre états :
1. **État Initial** : Attend qu'un bouton soit pressé
2. **État Attente** : Attend pendant 3 secondes ou qu'un bouton soit pressé (LED allumée) 
3. **État Action** : Fait clignoter la LED pendant 5 secondes
4. **État Final** : Attend qu'un bouton soit pressé pour recommencer

## Structure du projet

Le projet suit la structure standard platformIO. Les fichiers cpp, contenus dans le dossier src/ continent le code en lui même (gestion de la machine d'état, du bouton,...). Les fichiers h, contenus dans le dossier include/ contiennent les déclarations des fonctions et des variables utilisées dans le code.
Dans cette structure, chaque état du système a été intégré dans un fichier séparé, ce qui permet de mieux organiser le code et de faciliter la maintenance. Chaque fichier contient les fonctions spécifiques à l'état correspondant.
Voici une représentation de la structure du projet :


```structure
projet/
├── include/          # Fichiers d'en-tête (.h)
│   ├── config.h      # Configuration globale
│   ├── etatInitial.h
│   ├── etatAttente.h
│   ├── etatAction.h
│   ├── etatFinal.h
├── src/              # Fichiers source (.cpp)
│   ├── main.cpp      # Fichier principal
│   ├── etatInitial.cpp
│   ├── etatAttente.cpp
│   ├── etatAction.cpp
│   ├── etatFinal.cpp
├── platformio.ini    # Configuration PlatformIO et dépendances
└── README.md         # Documentation
```


## Configuration du projet

Le fichier `platformio.ini` doit inclure les dépendances suivantes. Pour ajouter des dépendances, vous pouvez ajouter des url ou des noms de bibliothèques dans la section `lib_deps` :
```ini
lib_deps = 
    https://github.com/jrullan/StateMachine.git
    mark170987/Button@^1.0.0
```

Le fichier `config.h` sert à configurer les broches et les paramètres globaux du projet. Vous pouvez ajuster ces paramètres selon vos besoins. idéalement, aucune valeur ne devrait être définie dans les autres fichiers source.


# Simulation (Wokwi)

Wokwi est un simulateur en ligne qui permet de tester vos projets Arduino sans matériel physique. Voici comment l'utiliser pour ce projet :

## Installation
1. installez l'extension Wokwi pour VSCode : [Wokwi pour vscode](https://docs.wokwi.com/vscode/getting-started).
2. Appuyez sur `F1` et tapez `Wokwi: requesta new license` pour obtenir une licence gratuite. Vous pouvez utiliser votre compte github pour vous connecter.

## Configuration du projet
Deux fichiers sont nécessaires pour l'utilisation de Wokwi :

    - `wokwi.toml` : fichier de configuration du projet. Il doit être à la racine du projet. Il contient les informations sur le projet et les composants matériels utilisés.

    - `diagram.json` : fichier de configuration des composants matériels. Il doit être à la racine du projet. Il contient les informations sur les composants matériels utilisés et leurs connexions. Ce fichier ne peut pas être édité dans VSCode en version gratuite, mais un éditeur est disponible en ligne en créant un projet dans Wokwi. Le contenu peut être copié et collé dans le fichier local.

Ces fichier sont déjà présents dans le projet. Vous pouvez les modifier selon vos besoins. Pour plus d'informations, consultez la documentation officielle de Wokwi : [https://docs.wokwi.com](https://docs.wokwi.com).

*Remarque : pour éditer le fichier `diagram.json`, il est nécessaire de le renommer temporairement (par exemple, `diagram_.json`).*

## Simulation

1. Cliquez sur le bouton "Start Simulation" pour lancer la simulation.(ou appuyez sur `F1` et tapez `Wokwi: Start Simulation`)
2. Observez le comportement de votre machine d'état et interagissez avec les composants (par exemple, en appuyant sur le bouton).
3. Si nécessaire, ajustez votre code ou la configuration matérielle pour corriger les erreurs.

Pour plus d'informations, consultez la documentation officielle de Wokwi : [https://docs.wokwi.com](https://docs.wokwi.com).
