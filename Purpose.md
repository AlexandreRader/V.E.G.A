## 🎯 L'Objectif Global

Le robot doit rallier un point **A** à un point **B** en suivant une trajectoire calculée par une station sol, tout en restant capable d'éviter des obstacles imprévus et en garantissant que sa position réelle sur la carte reste fidèle à la réalité malgré les glissements de terrain.

---

## 🏗️ Architecture Matérielle & Rôles

Voici comment tes composants s'articulent pour former le système complet :

|Sous-système|Composant(s)|Rôle "Biologique"|
|---|---|---|
|**Cerveau**|**ESP32-S3**|Orchestre les calculs mathématiques (EKF) et la gestion des deux cœurs.|
|**Squelette & Muscles**|**6x NEMA 17** + **TMC2208**|Propulsion. Les TMC2208 assurent un mouvement fluide et silencieux.|
|**Articulations**|**4x MG90** + **PCA9685**|Direction. Le PCA9685 libère l'ESP32 en gérant les angles via I2C.|
|**Oreille Interne**|**IMU 9-DOF Grove**|Équilibre et cap. Mesure l'accélération et l'orientation magnétique.|
|**Vue (Proximité)**|**4x ToF VL53L3CX**|Réflexe de survie. Détecte les obstacles avant l'impact.|
|**Système Nerveux**|**NRF24L01**|Communication. Reçoit les ordres de mission et renvoie la télémétrie.|
|**Énergie**|**LiPo 3S** + **LM2596**|Sang et métabolisme. Fournit la puissance aux moteurs et régule le 5V.|
|**Vision Finale**|**Capteur IR 38kHz**|Précision terminale. Verrouille la cible une fois arrivé sur zone.|

---

## 💻 Les 3 Missions du Code

### 1. L'Estimation (Où suis-je ?)

C'est le rôle du **Filtre de Kalman Étendu (EKF)**.

- Il utilise tes matrices mathématiques pour fusionner deux sources : les pas envoyés aux **NEMA 17** (théorie) et les données de l'**IMU** (réalité).
    
- Cela permet de corriger les erreurs si une roue patine ou si le robot dévie de sa trajectoire.
    

### 2. La Décision (Où dois-je aller ?)

Le **PathFollower** compare la position estimée par l'EKF avec les waypoints reçus par le **NRF24L01**.

- Il génère une consigne de vitesse (V) et de rotation (ω) pour rester sur la ligne tracée par le PC.
    

### 3. L'Action (Comment j'y vais ?)

La **Cinématique ICR (Instantaneous Center of Rotation)** transforme les ordres V et ω en commandes physiques.

- Elle calcule les 4 angles des **MG90** (via PCA9685) et les 6 fréquences de pas (Hz) des **TMC2208**.
    
- Elle assure que le robot peut pivoter sur lui-même (**Pivot Turn**) sans glisser.
    

---

## 🔄 Le Cycle de Vie d'une Mission

1. **Préparation (PC) :** Tu prends la photo, le PC génère les points et les envoie via **NRF24**.
    
2. **Initialisation (ESP32) :** Le robot décode les points et calibre l'**IMU**.
    
3. **Navigation (Boucle Principale) :**
    
    - **Cœur 1 :** Calcule l'EKF et le prochain mouvement (Intelligence).
        
    - **Cœur 0 :** Génère les impulsions moteurs et surveille les **ToF** (Réflexes).
        
4. **Évitement :** Si un **ToF** détecte un obstacle, le robot s'arrête, contourne, et l'EKF met à jour la position pendant la déviation.
    
5. **Docking :** Proche de la cible, le **capteur IR** prend le relais pour un arrêt parfait pile au-dessus de l'objectif.