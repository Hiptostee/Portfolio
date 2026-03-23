#pragma once
#include <stdint.h>

// ---------------- I2C ----------------
constexpr uint8_t I2C_ADDR = 0x12;
constexpr uint8_t REG_MOTOR_SPEEDS = 51;
constexpr uint8_t REG_ENCODERS = 60;
constexpr uint8_t REG_PID_GAINS = 70;
constexpr uint8_t REG_TELEM_FL = 73;

// Pico I2C1 pins (rename to avoid collisions with core macros)
constexpr int I2C_SDA_PIN = 26;
constexpr int I2C_SCL_PIN = 27;

// ---------------- TB6612 ----------------
constexpr int STBY = 15;

// Motor pins (your mapping)
constexpr int FL_PWM = 8, FL_IN1 = 10, FL_IN2 = 9;
constexpr int FR_PWM = 13, FR_IN1 = 11, FR_IN2 = 12;
constexpr int BL_PWM = 2, BL_IN1 = 3, BL_IN2 = 4;
constexpr int BR_PWM = 7, BR_IN1 = 5, BR_IN2 = 6;

// Encoders
constexpr int FL_ENC_A = 18, FL_ENC_B = 19;
constexpr int FR_ENC_A = 16, FR_ENC_B = 17;
constexpr int BL_ENC_A = 22, BL_ENC_B = 28;
constexpr int BR_ENC_A = 20, BR_ENC_B = 21;

// Encoder directions
constexpr int ENC_DIR_FL = -1;
constexpr int ENC_DIR_FR = +1;
constexpr int ENC_DIR_BL = +1;
constexpr int ENC_DIR_BR = +1;

// Control loop
constexpr uint32_t CONTROL_MS = 20;
constexpr float DT = (float)CONTROL_MS / 1000.0f;

// cmd -> target velocity (ticks/sec)
constexpr float MAX_TICKS_S = 3500.0f;