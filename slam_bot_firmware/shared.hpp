#pragma once
#include <stdint.h>

struct MotorState
{
  float iTerm = 0.0f;
  float lastMeas = 0.0f;
  bool first = true;
};

struct HoldState
{
  float iTerm = 0.0f;
  int32_t lastErr = 0;
  bool first = true;
};

enum ControlMode : uint8_t
{
  MODE_DRIVE = 0,
  MODE_HOLD = 1
};

// ---------------- shared state ----------------
// Encoders
extern volatile int32_t encFL, encFR, encBL, encBR;

// Latest cmd from Pi (-127..127)
extern volatile int8_t cmdFL, cmdFR, cmdBL, cmdBR;

// I2C reg pointer
extern volatile uint8_t currentReg;

// PID gains (fixed point)
extern volatile int16_t kp_q, ki_q, kd_q;
extern volatile int16_t kp_q_hold, ki_q_hold, kd_q_hold;

// telemetry (FL)
extern volatile int32_t tele_tpos_fl, tele_pos_fl, tele_err_fl, tele_pwm_fl;

// target positions (ticks) — latched on Pico at stop moment
extern volatile int32_t target_pos_FL, target_pos_FR, target_pos_BL, target_pos_BR;

// mode + requests
extern volatile ControlMode mode;
extern volatile bool hold_reset_req;
extern volatile bool was_moving;

// PID states
extern MotorState pidFL, pidFR, pidBL, pidBR;
extern HoldState holdFL, holdFR, holdBL, holdBR;

// ---------------- helpers ----------------
int clamp255(int v);
int16_t read_i16_le(class TwoWire &w);