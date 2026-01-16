#include <Arduino.h>
#include "config.hpp"
#include "shared.hpp"
#include "motor.hpp"

void motorInit()
{
  pinMode(STBY, OUTPUT);
  digitalWrite(STBY, HIGH);

  pinMode(FL_PWM, OUTPUT);
  pinMode(FL_IN1, OUTPUT);
  pinMode(FL_IN2, OUTPUT);
  pinMode(FR_PWM, OUTPUT);
  pinMode(FR_IN1, OUTPUT);
  pinMode(FR_IN2, OUTPUT);
  pinMode(BL_PWM, OUTPUT);
  pinMode(BL_IN1, OUTPUT);
  pinMode(BL_IN2, OUTPUT);
  pinMode(BR_PWM, OUTPUT);
  pinMode(BR_IN1, OUTPUT);
  pinMode(BR_IN2, OUTPUT);

  setAll(0, 0, 0, 0);
}

void setMotor(int in1, int in2, int pwm, int speed)
{
  speed = clamp255(speed);

  if (speed > 0)
  {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
    analogWrite(pwm, speed);
  }
  else if (speed < 0)
  {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
    analogWrite(pwm, -speed);
  }
  else
  {
    analogWrite(pwm, 0);
    digitalWrite(in1, HIGH);
    digitalWrite(in2, HIGH);
  }
}

void setAll(int fl, int fr, int bl, int br)
{
  setMotor(FL_IN1, FL_IN2, FL_PWM, fl);
  setMotor(FR_IN1, FR_IN2, FR_PWM, fr);
  setMotor(BL_IN1, BL_IN2, BL_PWM, bl);
  setMotor(BR_IN1, BR_IN2, BR_PWM, br);
}