#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ==========================================
// MAPPING ESP32-S3 - ROVER VEGA SC317 (N8R8)
// ==========================================

// --- BUS I2C (Centrales, Servos, ToF) ---
#define PIN_I2C_SDA 4
#define PIN_I2C_SCL 5

// --- BUS SPI (Radio NRF24) ---
#define PIN_SPI_CSN   10
#define PIN_SPI_MOSI  11
#define PIN_SPI_SCK   12
#define PIN_SPI_MISO  13
#define PIN_RADIO_CE  14

// --- CAPTEURS TOF (Broches XSHUT) ---
#define PIN_TOF_1_LT   6 // left top
#define PIN_TOF_2_RT   7 // right top
#define PIN_TOF_3_LB  8 // left bottom
#define PIN_TOF_4_RB  9 // right bottom

// --- PROPULSION (Steppers M1 à M6) ---
#define PIN_ENABLE_MOTORS 47

// Moteur 1 (Avant Gauche)
#define PIN_STEP_M1 2
#define PIN_DIR_M1  1

// Moteur 2 (Milieu Gauche)
#define PIN_STEP_M2 38
#define PIN_DIR_M2  21

// Moteur 3 (Arrière Gauche)
#define PIN_STEP_M3 42
#define PIN_DIR_M3  41

// Moteur 4 (Avant Droit)
#define PIN_STEP_M4 40
#define PIN_DIR_M4  39

// Moteur 5 (Milieu Droit)
#define PIN_STEP_M5 17
#define PIN_DIR_M5  18

// Moteur 6 (Arrière Droit)
#define PIN_STEP_M6 15
#define PIN_DIR_M6  16

// --- COMMUNICATION SERIE (Console / Debug) ---
#define PIN_UART_RX 44
#define PIN_UART_TX 43

// --- CAPTEURS INFRAROUGES ---
#define PIN_IR_OUT    48   // Lecture du signal Infrarouge (Retour booléen)
#define PIN_IR_ENABLE 3    // (Optionnel) Pin d'activation du capteur IR

#endif // PINS_H