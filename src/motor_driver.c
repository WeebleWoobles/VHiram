#include "motor_driver.h"


void MOTOR_set_speed(float speed) {
    MOTOR_set_left(speed);
    MOTOR_set_right(speed);  
}

#define PWM_DEADZONE 0.45f

void MOTOR_set_left(float speed) {
    bool direction = (speed > 0.0f);
    gpio_set_level(H_BRIDGE_L_IN1, !direction);
    gpio_set_level(H_BRIDGE_L_IN2, direction);

    float mag = fabs(speed);

    // Remap speed to start at the deadzone
    float pwm_f = 0.0f;
    if (mag > 0.01f) { // only apply if moving
        pwm_f = PWM_DEADZONE + (1.0f - PWM_DEADZONE) * mag;
    }

    int pwm = (int)(pwm_f * 255.0f);
    if (pwm > 255) pwm = 255;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_L, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_L);
}

void MOTOR_set_right(float speed) {
    bool direction = (speed > 0.0f);
    gpio_set_level(H_BRIDGE_R_IN1, !direction);
    gpio_set_level(H_BRIDGE_R_IN2, direction);

    float mag = fabs(speed);

    float pwm_f = 0.0f;
    if (mag > 0.01f) {
        pwm_f = PWM_DEADZONE + (1.0f - PWM_DEADZONE) * mag;
    }

    int pwm = (int)(pwm_f * 255.0f);
    if (pwm > 255) pwm = 255;

    ledc_set_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_R, pwm);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, PWM_CHANNEL_R);
}