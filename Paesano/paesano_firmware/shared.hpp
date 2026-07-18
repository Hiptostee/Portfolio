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

// ---------------- ISR / I2C shared state ----------------
// Written by encoder ISRs; read by control and I2C handlers.
extern volatile int32_t encFL, encFR, encBL, encBR;

// Written by I2C receive; read by control.
extern volatile int8_t cmdFL, cmdFR, cmdBL, cmdBR;

// Written by I2C receive; read by I2C request.
extern volatile uint8_t currentReg;

// Written by I2C receive; read by control.
extern volatile int16_t kp_q, ki_q, kd_q;
extern volatile int16_t kp_q_hold, ki_q_hold, kd_q_hold;

// Written by control; read by I2C request.
extern volatile int32_t tele_tpos_fl, tele_pos_fl, tele_err_fl, tele_pwm_fl;
extern volatile int32_t tele_tpos_fr, tele_pos_fr, tele_err_fr, tele_pwm_fr;
extern volatile int32_t tele_tpos_bl, tele_pos_bl, tele_err_bl, tele_pwm_bl;
extern volatile int32_t tele_tpos_br, tele_pos_br, tele_err_br, tele_pwm_br;

// Written by I2C receive at stop latch; read by control.
extern volatile int32_t target_pos_FL, target_pos_FR, target_pos_BL, target_pos_BR;

// Written by I2C receive/control; read by control.
extern volatile ControlMode mode;
extern volatile bool hold_reset_req;
extern volatile bool was_moving;

// ---------------- control-loop state, not touched from ISRs ----------------
extern MotorState pidFL, pidFR, pidBL, pidBR;
extern HoldState holdFL, holdFR, holdBL, holdBR;

// ---------------- helpers ----------------
int clamp255(int v);
int16_t read_i16_le(class TwoWire &w);
