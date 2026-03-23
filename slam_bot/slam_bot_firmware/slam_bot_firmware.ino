#include <Arduino.h>

#include "motor.hpp"
#include "encoders.hpp"
#include "i2c_iface.hpp"
#include "control.hpp"

void setup()
{
  motorInit();
  encodersInit();
  i2cInit();
  controlInit();
}

void loop()
{
  controlTick();
}