#include <Wire.h>
#include <stdint.h>
#include <math.h>

// ---------------- I2C ----------------
#define I2C_ADDR 0x12
#define REG_MOTOR_SPEEDS 51
#define REG_ENCODERS 60
#define REG_PID_GAINS 70
#define REG_TELEM_FL 73

// Pico I2C1 pins
const int I2C_SDA = 26;
const int I2C_SCL = 27;

// ---------------- TB6612 ----------------
const int STBY = 15;

// Motor pins
const int FL_PWM = 8, FL_IN1 = 10, FL_IN2 = 9;
const int FR_PWM = 13, FR_IN1 = 11, FR_IN2 = 12;
const int BL_PWM = 2, BL_IN1 = 3, BL_IN2 = 4;
const int BR_PWM = 7, BR_IN1 = 5, BR_IN2 = 6;

// Encoders
const int FL_ENC_A = 18, FL_ENC_B = 19;
const int FR_ENC_A = 16, FR_ENC_B = 17;
const int BL_ENC_A = 22, BL_ENC_B = 28;
const int BR_ENC_A = 20, BR_ENC_B = 21;

// Encoder directions
int ENC_DIR_FL = -1;
int ENC_DIR_FR = +1;
int ENC_DIR_BL = +1;
int ENC_DIR_BR = +1;

// Encoder counts
volatile int32_t encFL = 0, encFR = 0, encBL = 0, encBR = 0;

// Latest cmd from Pi (-127..127)
volatile int8_t cmdFL = 0, cmdFR = 0, cmdBL = 0, cmdBR = 0;

// I2C reg pointer
volatile uint8_t currentReg = 0;

// Control loop
const uint32_t CONTROL_MS = 20;
const float DT = CONTROL_MS / 1000.0f;

// cmd -> target velocity (ticks/sec)
const float MAX_TICKS_S = 3500.0f;

// PID gains (fixed point from Pi)
volatile int16_t kp_q = 0; // kp = kp_q/1000
volatile int16_t ki_q = 0; // ki = ki_q/100000
volatile int16_t kd_q = 0; // kd = kd_q/10

// ---------------- telemetry (FL only) ----------------
// 4x int32: vtgt, vmeas, u_raw, u_out
volatile int32_t tele_vtgt_fl = 0;
volatile int32_t tele_vmeas_fl = 0;
volatile int32_t tele_uraw_fl = 0;
volatile int32_t tele_uout_fl = 0;

struct MotorState
{
  float iTerm = 0;
  float lastMeas = 0;
  bool first = true;
};

// ---------------- helpers ----------------
static inline int clamp255(int v)
{
  if (v > 255)
    return 255;
  if (v < -255)
    return -255;
  return v;
}

static inline int16_t read_i16_le(TwoWire &w)
{
  uint8_t lo = (uint8_t)w.read();
  uint8_t hi = (uint8_t)w.read();
  return (int16_t)((hi << 8) | lo);
}

// ---------------- motor ----------------
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
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void setAll(int fl, int fr, int bl, int br)
{
  setMotor(FL_IN1, FL_IN2, FL_PWM, fl);
  setMotor(FR_IN1, FR_IN2, FR_PWM, fr);
  setMotor(BL_IN1, BL_IN2, BL_PWM, bl);
  setMotor(BR_IN1, BR_IN2, BR_PWM, br);
}

// ---------------- encoder ISRs ----------------
void isrFL()
{
  bool a = digitalRead(FL_ENC_A), b = digitalRead(FL_ENC_B);
  encFL += (a == b ? +1 : -1) * ENC_DIR_FL;
}
void isrFR()
{
  bool a = digitalRead(FR_ENC_A), b = digitalRead(FR_ENC_B);
  encFR += (a == b ? +1 : -1) * ENC_DIR_FR;
}
void isrBL()
{
  bool a = digitalRead(BL_ENC_A), b = digitalRead(BL_ENC_B);
  encBL += (a == b ? +1 : -1) * ENC_DIR_BL;
}
void isrBR()
{
  bool a = digitalRead(BR_ENC_A), b = digitalRead(BR_ENC_B);
  encBR += (a == b ? +1 : -1) * ENC_DIR_BR;
}

// ---------------- I2C callbacks ----------------
void onReceive(int n)
{
  if (n <= 0)
    return;
  currentReg = (uint8_t)Wire1.read();
  n--;

  if (currentReg == REG_MOTOR_SPEEDS && n >= 4)
  {
    int8_t fl = (int8_t)Wire1.read();
    int8_t fr = (int8_t)Wire1.read();
    int8_t bl = (int8_t)Wire1.read();
    int8_t br = (int8_t)Wire1.read();
    noInterrupts();
    cmdFL = fl;
    cmdFR = fr;
    cmdBL = bl;
    cmdBR = br;
    interrupts();
    while (n-- > 4)
      (void)Wire1.read();
    return;
  }

  if (currentReg == REG_PID_GAINS && n >= 6)
  {
    int16_t kp = read_i16_le(Wire1);
    int16_t ki = read_i16_le(Wire1);
    int16_t kd = read_i16_le(Wire1);
    noInterrupts();
    kp_q = kp;
    ki_q = ki;
    kd_q = kd;
    interrupts();
    while (n-- > 6)
      (void)Wire1.read();
    return;
  }

  while (n-- > 0)
    (void)Wire1.read();
}

void onRequest()
{
  if (currentReg == REG_ENCODERS)
  {
    int32_t fl, fr, bl, br;
    noInterrupts();
    fl = encFL;
    fr = encFR;
    bl = encBL;
    br = encBR;
    interrupts();

    uint8_t out[16];
    auto pack32 = [&](int idx, int32_t v)
    {
      out[idx + 0] = (uint8_t)(v & 0xFF);
      out[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
      out[idx + 2] = (uint8_t)((v >> 16) & 0xFF);
      out[idx + 3] = (uint8_t)((v >> 24) & 0xFF);
    };
    pack32(0, fl);
    pack32(4, fr);
    pack32(8, bl);
    pack32(12, br);
    Wire1.write(out, 16);
    return;
  }

  if (currentReg == REG_TELEM_FL)
  {
    int32_t vt, vm, ur, uo;
    noInterrupts();
    vt = tele_vtgt_fl;
    vm = tele_vmeas_fl;
    ur = tele_uraw_fl;
    uo = tele_uout_fl;
    interrupts();

    uint8_t out[16];
    auto pack32 = [&](int idx, int32_t v)
    {
      out[idx + 0] = (uint8_t)(v & 0xFF);
      out[idx + 1] = (uint8_t)((v >> 8) & 0xFF);
      out[idx + 2] = (uint8_t)((v >> 16) & 0xFF);
      out[idx + 3] = (uint8_t)((v >> 24) & 0xFF);
    };
    pack32(0, vt);
    pack32(4, vm);
    pack32(8, ur);
    pack32(12, uo);
    Wire1.write(out, 16);
    return;
  }

  Wire1.write((uint8_t)0);
}

static MotorState pidFL, pidFR, pidBL, pidBR;

void resetPIDStates()
{
  pidFL = MotorState{};
  pidFR = MotorState{};
  pidBL = MotorState{};
  pidBR = MotorState{};
}

int computeMotorPID(float target, float meas, MotorState &s,
                    float kp, float ki, float kd, float dt,
                    float *u_raw_out)
{
  float e = target - meas;

  float dMeas = 0.0f;
  if (!s.first)
    dMeas = (meas - s.lastMeas) / dt;
  else
    s.first = false;
  s.lastMeas = meas;

  float pTerm = kp * e;
  float dTerm = -kd * dMeas;

  const float kF = 255.0f / MAX_TICKS_S;

  const float kS = 25.0f; // try 20–40 on TB6612 setups

  float pwm_ff = kF * target;
  if (fabsf(target) > 1.0f)
  {
    pwm_ff += (target > 0.0f) ? kS : -kS;
  }
  else
  {
    pwm_ff = 0.0f;
  }

  const float I_PWM_MAX = 80.0f;

  if (fabsf(ki) < 1e-9f)
  {
    s.iTerm = 0.0f;
  }
  else
  {
    float iPWM_now = ki * s.iTerm;
    if (iPWM_now > I_PWM_MAX)
      iPWM_now = I_PWM_MAX;
    if (iPWM_now < -I_PWM_MAX)
      iPWM_now = -I_PWM_MAX;

    float u_pre = pwm_ff + pTerm + iPWM_now + dTerm;

    bool sat_hi = (u_pre >= 255.0f);
    bool sat_lo = (u_pre <= -255.0f);

    bool allow_i = (!sat_hi && !sat_lo) ||
                   (sat_hi && e < 0.0f) ||
                   (sat_lo && e > 0.0f);

    if (allow_i)
      s.iTerm += e * dt;

    float iTerm_max = I_PWM_MAX / fabsf(ki);
    if (s.iTerm > iTerm_max)
      s.iTerm = iTerm_max;
    if (s.iTerm < -iTerm_max)
      s.iTerm = -iTerm_max;
  }

  float iTermPWM = ki * s.iTerm;
  if (iTermPWM > I_PWM_MAX)
    iTermPWM = I_PWM_MAX;
  if (iTermPWM < -I_PWM_MAX)
    iTermPWM = -I_PWM_MAX;

  float u = pwm_ff + pTerm + iTermPWM + dTerm;

  if (u_raw_out)
    *u_raw_out = u;

  return clamp255((int)lroundf(u));
}

void runVelocityPID(float vFL, float vFR, float vBL, float vBR)
{
  int8_t cFL, cFR, cBL, cBR;
  uint8_t en;
  int16_t kpq, kiq, kdq;

  noInterrupts();
  cFL = cmdFL;
  cFR = cmdFR;
  cBL = cmdBL;
  cBR = cmdBR;
  kpq = kp_q;
  kiq = ki_q;
  kdq = kd_q;
  interrupts();

  if (cFL == 0 && cFR == 0 && cBL == 0 && cBR == 0)
  {
    resetPIDStates();
    setAll(0, 0, 0, 0);
    return;
  }

  // 3. Convert gains
  const float kp = ((float)kpq) / 1000.0f;
  const float ki = ((float)kiq) / 100000.0f;
  const float kd = ((float)kdq) / 10.0f;

  const float k_cmd_to_ticks = MAX_TICKS_S / 127.0f;

  // 4. Compute Targets
  float targetFL = cFL * k_cmd_to_ticks;
  float targetFR = cFR * k_cmd_to_ticks;
  float targetBL = cBL * k_cmd_to_ticks;
  float targetBR = cBR * k_cmd_to_ticks;

  float u_raw_fl = 0.0f;

  int fl_out = computeMotorPID(targetFL, vFL, pidFL, kp, ki, kd, DT, &u_raw_fl);
  int fr_out = computeMotorPID(targetFR, vFR, pidFR, kp, ki, kd, DT, nullptr);
  int bl_out = computeMotorPID(targetBL, vBL, pidBL, kp, ki, kd, DT, nullptr);
  int br_out = computeMotorPID(targetBR, vBR, pidBR, kp, ki, kd, DT, nullptr);

  // 6. Drive Motors
  setAll(fl_out, fr_out, bl_out, br_out);

  noInterrupts();
  tele_vtgt_fl = (int32_t)lroundf(targetFL);
  tele_vmeas_fl = (int32_t)lroundf(vFL);
  tele_uraw_fl = (int32_t)lroundf(u_raw_fl);
  tele_uout_fl = (int32_t)fl_out;
  interrupts();
}

// ---------------- setup/loop ----------------
void setup()
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

  Wire1.setSDA(I2C_SDA);
  Wire1.setSCL(I2C_SCL);
  Wire1.begin(I2C_ADDR);
  Wire1.onReceive(onReceive);
  Wire1.onRequest(onRequest);
  Wire1.setClock(100000);

  setAll(0, 0, 0, 0);
}

void loop()
{
  static uint32_t lastMs = 0;
  uint32_t now = millis();
  if (now - lastMs < CONTROL_MS)
    return;
  lastMs = now;

  // snapshot encoder counts
  int32_t eFL, eFR, eBL, eBR;
  noInterrupts();
  eFL = encFL;
  eFR = encFR;
  eBL = encBL;
  eBR = encBR;
  interrupts();

  // velocity estimate (ticks/sec)
  static bool init = false;
  static int32_t lastFL = 0, lastFR = 0, lastBL = 0, lastBR = 0;

  if (!init)
  {
    lastFL = eFL;
    lastFR = eFR;
    lastBL = eBL;
    lastBR = eBR;
    init = true;
    return;
  }

  int32_t dFL = eFL - lastFL;
  int32_t dFR = eFR - lastFR;
  int32_t dBL = eBL - lastBL;
  int32_t dBR = eBR - lastBR;

  lastFL = eFL;
  lastFR = eFR;
  lastBL = eBL;
  lastBR = eBR;

  float vFL = dFL / DT;
  float vFR = dFR / DT;
  float vBL = dBL / DT;
  float vBR = dBR / DT;

  runVelocityPID(vFL, vFR, vBL, vBR);
}