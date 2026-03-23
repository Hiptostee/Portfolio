#include <Arduino.h>
#include "config.hpp"
#include "shared.hpp"
#include "encoders.hpp"

static inline void isrFL()
{
  bool a = digitalRead(FL_ENC_A), b = digitalRead(FL_ENC_B);
  encFL += (a == b ? +1 : -1) * ENC_DIR_FL;
}
static inline void isrFR()
{
  bool a = digitalRead(FR_ENC_A), b = digitalRead(FR_ENC_B);
  encFR += (a == b ? +1 : -1) * ENC_DIR_FR;
}
static inline void isrBL()
{
  bool a = digitalRead(BL_ENC_A), b = digitalRead(BL_ENC_B);
  encBL += (a == b ? +1 : -1) * ENC_DIR_BL;
}
static inline void isrBR()
{
  bool a = digitalRead(BR_ENC_A), b = digitalRead(BR_ENC_B);
  encBR += (a == b ? +1 : -1) * ENC_DIR_BR;
}

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