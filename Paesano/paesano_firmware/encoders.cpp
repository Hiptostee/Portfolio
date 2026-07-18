#include <Arduino.h>
#if defined(ARDUINO_ARCH_RP2040)
#include "hardware/gpio.h"
#endif

#include "config.hpp"
#include "shared.hpp"
#include "encoders.hpp"

static inline void updateEncoder(uint8_t pinA, uint8_t pinB, int8_t direction, volatile int32_t &encoder)
{
#if defined(ARDUINO_ARCH_RP2040)
  const uint32_t gpio = gpio_get_all();
  const bool a = (gpio & (1u << pinA)) != 0;
  const bool b = (gpio & (1u << pinB)) != 0;
#else
  const bool a = digitalRead(pinA);
  const bool b = digitalRead(pinB);
#endif
  encoder += (a == b ? +1 : -1) * direction;
}

static inline void isrFL() { updateEncoder(FL_ENC_A, FL_ENC_B, ENC_DIR_FL, encFL); }
static inline void isrFR() { updateEncoder(FR_ENC_A, FR_ENC_B, ENC_DIR_FR, encFR); }
static inline void isrBL() { updateEncoder(BL_ENC_A, BL_ENC_B, ENC_DIR_BL, encBL); }
static inline void isrBR() { updateEncoder(BR_ENC_A, BR_ENC_B, ENC_DIR_BR, encBR); }

void encodersInit()
{
  pinMode(FL_ENC_A, INPUT_PULLUP);
  pinMode(FL_ENC_B, INPUT_PULLUP);
  pinMode(FR_ENC_A, INPUT_PULLUP);
  pinMode(FR_ENC_B, INPUT_PULLUP);
  pinMode(BL_ENC_A, INPUT_PULLUP);
  pinMode(BL_ENC_B, INPUT_PULLUP);
  pinMode(BR_ENC_A, INPUT_PULLUP);
  pinMode(BR_ENC_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(FL_ENC_A), isrFL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(FR_ENC_A), isrFR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BL_ENC_A), isrBL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(BR_ENC_A), isrBR, CHANGE);
}
