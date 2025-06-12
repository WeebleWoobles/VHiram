#pragma once
#include "main.h"

void init_main(void);

/****************
 *GPIO DEFFINITIONS
 *****************/
#include "pinout.h"
#include "driver/ledc.h"

#define PWM_FREQ       20000     // PWM frequency in Hz
#define PWM_RESOLUTION LEDC_TIMER_8_BIT  // Resolution of PWM duty (8-bit means 0-255)
#define PWM_CHANNEL_L  LEDC_CHANNEL_0    // PWM channel for Left motor
#define PWM_CHANNEL_R  LEDC_CHANNEL_1    // PWM channel for Right motor
#define PWM_TIMER      LEDC_TIMER_0 

void init_gpio(void);
void init_pwm(void);

/****************
 *I2C DEFFINITIONS
 *****************/
#include "driver/i2c.h"
#include "IMU.h"
void init_I2C(void);

/****************
 *I2C DEFFINITIONS
 *****************/
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>