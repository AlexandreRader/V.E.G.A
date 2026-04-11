#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

// ==========================================
// MAPPING ESP32-S3 - ROVER VEGA SC317
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
#define PIN_TOF_1_AVANT   6
#define PIN_TOF_2_ARRIERE 7
#define PIN_TOF_3_GAUCHE  8
#define PIN_TOF_4_DROITE  9

// --- PROPULSION (Steppers M1 à M6) ---
#define PIN_ENABLE_MOTORS 14
// Moteur 1 (Avant Gauche)
#define PIN_STEP_M1 15
#define PIN_DIR_M1  16

// Moteur 2 (Milieu Gauche)
#define PIN_STEP_M2 17
#define PIN_DIR_M2  18

// Moteur 3 (Arrière Gauche)
#define PIN_STEP_M3 42
#define PIN_DIR_M3  41

// Moteur 4 (Avant Droit)
#define PIN_STEP_M4 40
#define PIN_DIR_M4  39

// Moteur 5 (Milieu Droit)
#define PIN_STEP_M5 38
#define PIN_DIR_M5  37

// Moteur 6 (Arrière Droit)
#define PIN_STEP_M6 36
#define PIN_DIR_M6  35

// --- COMMUNICATION SERIE (TMC2208) ---
#define PIN_UART_RX 44
#define PIN_UART_TX 43

// --- CAPTEUR IR DOCKING ---
#define PIN_IR_DOCKING 34
#define PIN_DIR_M6  35

// --- CAPTEURS DIVERS ---
#define PIN_IR_SENSOR 21

#endif // PINS_H