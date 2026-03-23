#pragma once
void resetVelocityPIDStates();
int computeMotorPID(float target, float meas, struct MotorState &s,
                    float kp, float ki, float kd, float dt,
                    float *u_raw_out);

void runVelocityPID(float vFL, float vFR, float vBL, float vBR);