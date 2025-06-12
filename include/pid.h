#pragma once
#include "IMU.h"
#include "math.h"


void PID_pitch_offset(void);
float PID_get_pitch(float pitch, float dt);
float PID_pitch_acceleration(void);
float PID_gyro_acceleration(float dt);
double PID_correction_output(float pitch, double dt, float* setpoint);