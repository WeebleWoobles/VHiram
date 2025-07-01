main.c
#include "init.h"
#include "main.h"
#include "ESP8266_interface.h"
#include "timing_test.h"

// RTOS TASKS

#include "driver/i2c.h"
#include <stdio.h>

void scan_i2c(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
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

void app_main() {

    init_main();

    // Initialize ESP-NOW for wireless communication
    esp_err_t ret = esp_now_init_interface();
    if (ret != ESP_OK) {
        printf("Failed to initialize ESP-NOW: %s\n", esp_err_to_name(ret));
    } else {
        printf("ESP-NOW initialized successfully\n");
    }

    uint8_t raw[14] = {0};
    bool ok = IMU_ReadBytes(0x3B, raw, 14);
    if (!ok) {
    printf("Failed to read IMU data\n");
    } else {
    printf("Raw Accel/Gyro: ");
    for (int i = 0; i < 14; i++) {
        printf("%02X ", raw[i]);
    }
    printf("\n");
}

    // Setpoint is initialized to 0, but is a pointer because bluetooth with edit this. 
    float setpoint = 0.0f;
    float* setpter = &setpoint;


    //MOTOR TEST:
    // while(1){
    //     //RUN MOTORS FORWARD 5s
    //     // MOTOR_set_speed(255);
    //     // printf("Motor Forward\n");
    //     // vTaskDelay(pdMS_TO_TICKS(5000));
    //     // //RUN MOTORS Backwards 5s
    //     // MOTOR_set_speed(-255);
    //     // printf("motor backward\n");
    //     // vTaskDelay(pdMS_TO_TICKS(5000));

    //     MOTOR_set_speed(1);
    //     printf("Motor Forward\n");
    //     vTaskDelay(pdMS_TO_TICKS(5000));
    //     //RUN MOTORS Backwards 5s
    //     MOTOR_set_speed(-1);
    //     printf("motor backward\n");
    //     vTaskDelay(pdMS_TO_TICKS(5000));
    // }


    //TEST RTOS AND PID CONTROLLING MOTORS
    xTaskCreate(print_values, "Print Values Task", 2048, NULL, 5, NULL);
    xTaskCreate(main_control,"PID Test task", 4096, setpter, 5, NULL);
    xTaskCreate(led_task,"LED task", 1024, NULL, 5, NULL);
    xTaskCreate(timing_test_task, "TimingTestTask", 4096, &setpoint, 5, NULL);

}

void led_task(void* pvParameters){
    while (1) {
        gpio_set_level(LED_PIN, 0);  // LED ON (active-low)
        vTaskDelay(pdMS_TO_TICKS(1000));
        gpio_set_level(LED_PIN, 1);  // LED OFF
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

float pitch = 0;
double output = 0;

void print_values(void* pvParameters){
    while(1){
        printf("Pitch = %f, PID output = %f\n", pitch, output);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void main_control(void* pvParameters) {
    int control_time = 1; 
    double dt_in_s = control_time / 1000;
    while(1){
        //Step 0: get duration of time from last check
        
        // Check for joystick input from ESP-NOW
        joystick_data_t joystick = get_joystick_data();
        if (joystick.data_received) {
            // Process joystick input here
            // You can modify the setpoint or control behavior based on joystick input
            if (joystick.forward) {
                // Adjust setpoint for forward movement
                printf("Forward command received\n");
            }
            if (joystick.backward) {
                // Adjust setpoint for backward movement
                printf("Backward command received\n");
            }
            if (joystick.left) {
                // Adjust for left turn
                printf("Left command received\n");
            }
            if (joystick.right) {
                // Adjust for right turn
                printf("Right command received\n");
            }
            clear_joystick_data_flag();
        }
        
        //Step 1: get pitch datafrom IMU and PID
        pitch = PID_get_pitch(pitch ,dt_in_s);
        //Pitch will average out, Need perfect level surface to really calibrate
        
        //Step 2: Get output for motor
        output = PID_correction_output(pitch,  dt_in_s, pvParameters);

        //Step 3: Insert it into motor
        MOTOR_set_speed(-output);

        vTaskDelay(pdMS_TO_TICKS(control_time));
    }
}
