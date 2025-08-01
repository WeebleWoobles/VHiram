Init.c
#include "init.h"
#include "IMU.h"
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"



void init_gpio(void) {
    printf("Initializing GPIO\n");
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(LED_PIN, 0);
    gpio_set_direction(H_BRIDGE_L_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE_L_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE_L_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE_R_IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE_R_IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(H_BRIDGE_R_PWM, GPIO_MODE_OUTPUT);
    gpio_set_direction(IMU_AD0_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(IMU_AD0_PIN, 0);
    // Optional: Power cycle IMU
    #ifdef IMU_POWER_PIN
    gpio_set_direction(IMU_POWER_PIN, GPIO_MODE_OUTPUT);
    gpio_set_level(IMU_POWER_PIN, 0); // Off
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(IMU_POWER_PIN, 1); // On
    vTaskDelay(pdMS_TO_TICKS(100));
    #endif
    gpio_set_level(H_BRIDGE_L_IN1, 0);
    gpio_set_level(H_BRIDGE_L_IN2, 0);
    gpio_set_level(H_BRIDGE_L_PWM, 0);
    gpio_set_level(H_BRIDGE_R_IN1, 0);
    gpio_set_level(H_BRIDGE_R_IN2, 0);
    gpio_set_level(H_BRIDGE_R_PWM, 0);
}


void init_pwm(void) {
    printf("Initializing PWM\n");
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = PWM_TIMER,
        .duty_resolution = PWM_RESOLUTION,
        .freq_hz = PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);
    ledc_channel_config_t left = {
        .gpio_num = H_BRIDGE_L_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL_L,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&left);
    ledc_channel_config_t right = {
        .gpio_num = H_BRIDGE_R_PWM,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = PWM_CHANNEL_R,
        .timer_sel = PWM_TIMER,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&right);
}


void init_main(void) {
    init_gpio();

    init_pwm();
    vTaskDelay(pdMS_TO_TICKS(100));
    bool imu = IMU_Init();
    if (!imu) {
        printf("IMU initialization failed\n");
    }
    vTaskDelay(pdMS_TO_TICKS(200));
    if (!IMU_Calibrate()) {
        printf("IMU calibration failed\n");
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    // Test runtime data
}
