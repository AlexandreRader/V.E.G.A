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
};