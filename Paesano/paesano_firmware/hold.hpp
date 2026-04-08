#pragma once
void resetHoldPIDStates();

float computeHoldVelocity(int32_t targetPos, int32_t pos,
                          struct HoldState &s,
                          float kp_hold, float ki_hold, float kd_hold);