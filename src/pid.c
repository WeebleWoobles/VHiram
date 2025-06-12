#include "pid.h"
#include "IMU.h"
#include <math.h>
#include <stdio.h>

/// increase to reduce noise, decrease to correct reaction time (alpha) 

const float ALPHA = 0.80f;
#define KU 15
#define KT 1
#define KP 44.138
#define KI 111.69
#define KD 4.36

double integral = 0;
float previous_error = 0;


float PID_get_pitch(float prev_pitch, float dt) {
    float ax, ay, az, gx, gy, gz;
    if (!IMU_ReadAccelerometer(&ax, &ay, &az) || !IMU_ReadGyroscope(&gx, &gy, &gz)) {
        printf("Failed to read IMU in PID_get_pitch\n");
        return prev_pitch;
    }

    // Apply gyroscope offset (from calibration)
    gx -= offset_gyro_x;

    // Calculate accelerometer pitch
    float pitch_accel = atan2f(ay, sqrtf(ax * ax + az * az)) * (180.0f / M_PI);
    // Apply pitch offset if needed (uncomment after valid calibration)
    pitch_accel -= offset_pitch;

    // Calculate gyroscope pitch
    float gx_deg = gx / 131.0f; // ±250°/s (MPU-6050 sensitivity)
    float pitch_gyro = prev_pitch + gx_deg * dt;

    // Complementary filter
    const float ALPHA = 0.98f;
    return ALPHA * pitch_gyro + (1.0f - ALPHA) * pitch_accel;
}

double PID_correction_output(float pitch, double dt, float* setpoint) {
    // Makes components
    // Error away from vertical
    float error = 0.0f - pitch;

    // Make integral
    integral += error * 0.001f;

    // Derivative
    double derivative = (error - previous_error)/ 0.001f;
    // error .5, prev = .10, derivative = -.5
    // make output

    double PID = (KP * error) + (KI * integral) + (KD * derivative);
    return PID / 30000;
}

