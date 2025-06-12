#pragma once
#include "pid.h"
#define I2C_MASTER_FREQ_HZ  100000     //I2C CLOCK SPEED (100KHZ)
#define LOG_TAG "ROBOT_MAIN" 
#include "motor_driver.h"

// RTOS TASKS
void print_values(void* pvParameters);
void led_task(void* pvParameters);
void main_control(void* pvParamters);