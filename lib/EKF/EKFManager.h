#pragma once
#include <BasicLinearAlgebra.h>

using namespace BLA;

class EKFManager {
public:
    // État : x, y, theta (On retire vx, vy et omega de la matrice)
    Matrix<3> X; 
    Matrix<3, 3> P; // Incertitude (Covariance)
    Matrix<3, 3> Q; // Bruit de processus (Mécanique)
    Matrix<1, 3> H; // Matrice d'observation (On n'observe que Theta)
    Matrix<1, 1> R; // Bruit du capteur (Boussole)

    EKFManager() {
        X.Fill(0);

        // Confiance initiale
        P.Fill(0); 
        P(0,0) = 1; P(1,1) = 1; P(2,2) = 1;
        
        // Q : Bruit de prédiction (Glissement mécanique)
        // Augmente ces valeurs si ton robot patine beaucoup sur le sol
        Q.Fill(0); 
        Q(0,0) = 0.01; // Incertitude générée sur X
        Q(1,1) = 0.01; // Incertitude générée sur Y
        Q(2,2) = 0.05; // Incertitude générée sur la rotation
        
        // R : Bruit du capteur
        // Augmente cette valeur si ta boussole tremble ou est bruitée
        R(0,0) = 0.5; 
        
        // H : Lien entre capteurs et état
        // On indique que notre seule mesure correspond à l'état 2 (Theta)
        H.Fill(0);
        H(0, 2) = 1; 
    }

    // La prédiction n'utilise plus l'accéléromètre, mais la vitesse linéaire ordonnée (v)
    // et la vitesse de rotation instantanée du gyroscope (omega)
    void predict(float v, float omega, float dt) {
        float theta = X(2);

        // 1. Modèle Cinématique Différentiel
        X(0) += v * cos(theta) * dt;
        X(1) += v * sin(theta) * dt;
        X(2) += omega * dt;

        // 🎯 SÉCURITÉ : Garder l'angle entre -PI et PI pour éviter que les chiffres explosent
        while (X(2) > M_PI) X(2) -= 2.0 * M_PI;
        while (X(2) < -M_PI) X(2) += 2.0 * M_PI;

        // 2. Linéarisation (Jacobienne F)
        // On calcule les dérivées pour tracer la tangente de la courbe
        Matrix<3, 3> F_mat;
        F_mat.Fill(0); 
        F_mat(0,0) = 1; F_mat(1,1) = 1; F_mat(2,2) = 1;
        F_mat(0, 2) = -v * sin(theta) * dt;
        F_mat(1, 2) =  v * cos(theta) * dt;

        // 3. Mise à jour de l'incertitude
        P = F_mat * P * ~F_mat + Q;
    }

    // La correction ne se fait plus que sur la Boussole
    void update(float measured_theta) {
        // 1. Calcul de l'erreur brute
        float y_err = measured_theta - X(2);

        // 🎯 SÉCURITÉ CRITIQUE : Le bug du passage par zéro (Nord)
        // Si je prédis 359° et que la boussole lit 1°, l'erreur est de +2°, pas de -358° !
        while (y_err > M_PI) y_err -= 2.0 * M_PI;
        while (y_err < -M_PI) y_err += 2.0 * M_PI;

        Matrix<1> Z_diff = {y_err};

        // 2. Calcul du Gain de Kalman (L'arbitre)
        Matrix<1, 1> S = H * P * ~H + R;
        Matrix<3, 1> K = P * ~H * Inverse(S); // Inverse d'une matrice 1x1 est immédiat sur l'ESP32 !

        // 3. Mise à jour de l'état (Application de la correction)
        X = X + K * Z_diff;

        // 4. Baisse de l'incertitude globale
        Matrix<3, 3> I; 
        I.Fill(0); I(0,0) = 1; I(1,1) = 1; I(2,2) = 1;
        P = (I - K * H) * P;
    }

    // --- RÉINITIALISATION DE LA POSITION ---
    void reset(float start_x, float start_y, float start_theta) {
        X(0) = start_x;
        X(1) = start_y;
        X(2) = start_theta;

        // On donne à l'EKF une confiance presque aveugle en cette position de départ
        P.Fill(0); 
        P(0,0) = 0.001; 
        P(1,1) = 0.001; 
        P(2,2) = 0.001; 
        
        Serial.printf("🎯 EKF Réinitialisé -> X:%.2f, Y:%.2f, Theta:%.1f°\n", 
                      start_x, start_y, start_theta * 180.0/M_PI);
    }
};







/*

EKF INITIAL BASE SUR LE GIT DU ROVER

#pragma once
#include <BasicLinearAlgebra.h>

using namespace BLA;

class EKFManager {
public:
    // État : x, y, theta, vx, vy, omega
    Matrix<6> X; 
    Matrix<6, 6> P; // Incertitude (Covariance)
    Matrix<6, 6> Q; // Bruit de processus
    Matrix<3, 6> H; // Matrice d'observation (IMU : theta, ax, ay)
    Matrix<3, 3> R; // Bruit des capteurs

    EKFManager() {
        X.Fill(0);
        // Initialize identity matrices manually
        P.Fill(0); P(0,0) = 1; P(1,1) = 1; P(2,2) = 1; P(3,3) = 1; P(4,4) = 1; P(5,5) = 1;
        Q.Fill(0); Q(0,0) = 0.1; Q(1,1) = 0.1; Q(2,2) = 0.1; Q(3,3) = 0.1; Q(4,4) = 0.1; Q(5,5) = 0.1;
        R.Fill(0); R(0,0) = 0.05; R(1,1) = 0.05; R(2,2) = 0.05;
        
        // On observe seulement Theta (IMU) et les accélérations converties en vitesses
        H.Fill(0);
        H(0, 2) = 1; // On observe Theta
        H(1, 3) = 1; // On observe vx
        H(2, 4) = 1; // On observe vy
    }

    void predict(float ax, float ay, float omega, float dt) {
        // 1. Modèle de processus (Ton image process_model.png)
        float theta = X(2);
        float v = sqrt(X(3)*X(3) + X(4)*X(4));

        X(0) += v * cos(theta) * dt;
        X(1) += v * sin(theta) * dt;
        X(2) += omega * dt;
        X(3) += ax * dt;
        X(4) += ay * dt;
        X(5) = omega;

        // 2. Linéarisation : Matrice d'état de transition
        Matrix<6, 6> F_mat;
        F_mat.Fill(0); F_mat(0,0) = 1; F_mat(1,1) = 1; F_mat(2,2) = 1; F_mat(3,3) = 1; F_mat(4,4) = 1; F_mat(5,5) = 1;
        F_mat(0, 2) = -v * sin(theta) * dt;
        F_mat(1, 2) = v * cos(theta) * dt;
        F_mat(0, 3) = cos(theta) * dt;
        F_mat(1, 4) = sin(theta) * dt;

        // 3. Mise à jour de l'incertitude : P = F*P*F' + Q
        P = F_mat * P * ~F_mat + Q;
    }

    void update(float measured_theta, float measured_vx, float measured_vy) {
        // Mise à jour de Kalman classique (Correction par les capteurs)
        Matrix<3> Z = {measured_theta, measured_vx, measured_vy};
        auto K = P * ~H * Inverse(H * P * ~H + R);
        X = X + K * (Z - H * X);
        Matrix<6, 6> I; I.Fill(0); I(0,0) = 1; I(1,1) = 1; I(2,2) = 1; I(3,3) = 1; I(4,4) = 1; I(5,5) = 1;
        P = (I - K * H) * P;
    }

    // --- RÉINITIALISATION DE LA POSITION ---
    // Appelé quand le robot reçoit une nouvelle mission par radio
    void reset(float start_x, float start_y, float start_theta) {
        // 1. On force les nouvelles coordonnées dans le vecteur d'état
        X(0) = start_x;
        X(1) = start_y;
        X(2) = start_theta;

        // 2. Réinitialisation de l'incertitude (Matrice de Covariance P)
        // On dit au filtre : "Fais-moi confiance, je suis 100% sûr d'être à ce point précis, oublie le passé."
        // (Si tu utilises BasicLinearAlgebra)
        P.Fill(0); 
        P(0,0) = 0.001; // Très faible incertitude sur X
        P(1,1) = 0.001; // Très faible incertitude sur Y
        P(2,2) = 0.001; // Très faible incertitude sur le Cap (Theta)
        
        Serial.printf("🎯 EKF Réinitialisé -> X:%.2f, Y:%.2f, Theta:%.1f°\n", 
                      start_x, start_y, start_theta * 180.0/M_PI);
    }

};
*/