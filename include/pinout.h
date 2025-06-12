#pragma once
#include <driver/gpio.h>        //GPIO Driver

#define LED_PIN GPIO_NUM_2 // Pin2 : On board LED
#define L_ULTRA_TRIG GPIO_NUM_32 // Pin 6: Digital Output
#define L_ULTRA_ECHO GPIO_NUM_33 // Pin 7: Digital Input
#define R_ULTRA_TRIG GPIO_NUM_25 // Pin 8: Digital Output
#define R_ULTRA_ECHO GPIO_NUM_26 // Pin 9: Digital Input
#define L_MOTOR_DIR GPIO_NUM_27 // Pin 10: Digital Input
#define L_MOTOR_SPEED GPIO_NUM_14 // Pin 11: Digital Input
#define R_MOTOR_DIR GPIO_NUM_12 // Pin 12: Digital Input
#define R_MOTOR_SPEED GPIO_NUM_13 // Pin 13: Digital Input
#define I2C_MASTER_SCL_IO GPIO_NUM_22 // Pin 17: I2C_SCL
#define PIN_RX0 GPIO_NUM_44 // Pin 19: Digital I/O
#define I2C_MASTER_SDA_IO GPIO_NUM_21 // Pin 20: I2C_SDA
#define H_BRIDGE_L_PWM GPIO_NUM_5 // Pin 23: Output
#define H_BRIDGE_L_IN1 GPIO_NUM_17 // Pin 24: Digital Output
#define H_BRIDGE_L_IN2 GPIO_NUM_16 // Pin 25: Digital Output
#define H_BRIDGE_R_PWM GPIO_NUM_32 // Pin 26: Digital Output
#define H_BRIDGE_R_IN1 GPIO_NUM_33 // Pin 27: Digital Output
#define H_BRIDGE_R_IN2 GPIO_NUM_25 // Pin 28: Digital Output
#define IMU_AD0_PIN GPIO_NUM_18