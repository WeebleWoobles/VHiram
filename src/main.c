#include "init.h"
#include "main.h"
#include "pid.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "timing_test.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "nvs_flash.h"   

extern PIDController my_pid_controller;

extern float PID_get_pitch(float prev_pitch, float dt);
extern void MOTOR_set_speed(float speed);
extern bool IMU_ReadBytes(uint8_t reg_addr, uint8_t* data, uint16_t length);

#define DERIVATIVE_FILTER_ALPHA 0.7f

float pitch = 0.0f;
float pid_output = 0.0f;

void scan_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ
    };
    i2c_param_config(I2C_NUM_0, &conf);
    i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);

    printf("Scanning I2C bus...\n");
    for (uint8_t addr = 1; addr < 127; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            printf("Found device at 0x%02X\n", addr);
        }
    }
}

void led_task(void* pvParameters) {
    while (1) {
        gpio_set_level(LED_PIN, 0);
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void print_values(void* pvParameters) {
    while (1) {
        printf("Pitch: %.2f, PID Output: %.2f, Motor Cmd: %.2f\n", 
               pitch, pid_output, -pid_output);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void main_control(void* pvParameters) {
    float* setpoint_ptr = (float*)pvParameters;
    const int control_period_ms = 1;
    const float dt = control_period_ms / 1000.0f;

    while (1) {
        pitch = PID_get_pitch(pitch, dt);
        pid_output = PID_correction_output(&my_pid_controller, *setpoint_ptr, pitch, dt);
        float motor_cmd = pid_output / my_pid_controller.output_limit;
        if (motor_cmd > 1.0f) motor_cmd = 1.0f;
        if (motor_cmd < -1.0f) motor_cmd = -1.0f;
        MOTOR_set_speed(-motor_cmd);
        vTaskDelay(pdMS_TO_TICKS(control_period_ms));
    }
}

void app_main() {
    // NVS init
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_main();

    my_pid_controller.kp = 10.0f;
    my_pid_controller.ki = 10.0f;
    my_pid_controller.kd = 5.0f;
    my_pid_controller.integral = 0.0f;
    my_pid_controller.previous_error = 0.0f;
    my_pid_controller.filtered_derivative = 0.0f;
    my_pid_controller.output_limit = 255.0f;
    my_pid_controller.alpha = DERIVATIVE_FILTER_ALPHA;

    scan_i2c();

    uint8_t imu_raw[14] = {0};
    bool imu_ok = IMU_ReadBytes(0x3B, imu_raw, 14);
    if (!imu_ok) {
        printf("Failed to read IMU data\n");
    } else {
        printf("Raw Accel/Gyro: ");
        for (int i = 0; i < 14; i++) {
            printf("%02X ", imu_raw[i]);
        }
        printf("\n");
    }

    float setpoint = -2.0f;

    xTaskCreate(print_values, "PrintValues", 2048, NULL, 5, NULL);
    xTaskCreate(main_control, "MainControl", 4096, &setpoint, 5, NULL);
    xTaskCreate(led_task, "LEDTask", 1024, NULL, 5, NULL);
    xTaskCreate(timing_test_task, "timing_test", 4096, &setpoint, 5, NULL); // Changed to &setpoint

}
