// #include "timing_test.h"
// #include "pid.h"
// #include "main.h"
// #include "driver/i2c.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "esp_timer.h"
// #include <stdio.h>

// extern PIDController my_pid_controller;
// extern float PID_get_pitch(float prev_pitch, float dt);
// extern void MOTOR_set_speed(float speed);

// void timing_test_task(void* pvParameters) {
//     float* setpoint_ptr = (float*)pvParameters;
//     const int control_period_ms = 1;
//     const float dt = control_period_ms / 1000.0f;

//     unsigned long last_control_time = 0;
//     unsigned long last_imu_time = 0;
//     unsigned long imu_count = 0;
//     unsigned long control_count = 0;

//     float pitch = 0.0f;
//     float pid_output = 0.0f;

//     while (1) {
//         unsigned long now = esp_timer_get_time(); // Microseconds

//         // ---- Measure IMU Timing ----
//         float prev_pitch = pitch;
//         pitch = PID_get_pitch(pitch, dt);
//         if (pitch != prev_pitch) { // Assuming pitch only updates on new IMU data
//             imu_count++;
//             if (imu_count % 100 == 0) {
//                 unsigned long imu_delta = now - last_imu_time;
//                 last_imu_time = now;
//                 printf("[IMU] ~%.2f Hz (Δ %lu us)\n", 1e6 / (float)imu_delta, imu_delta);
//             }
//         }

//         // ---- Run PID + Control Loop Timing ----
//         pid_output = PID_correction_output(&my_pid_controller, *setpoint_ptr, pitch, dt);
//         float motor_cmd = pid_output / my_pid_controller.output_limit;
//         if (motor_cmd > 1.0f) motor_cmd = 1.0f;
//         if (motor_cmd < -1.0f) motor_cmd = -1.0f;
//         MOTOR_set_speed(-motor_cmd);

//         control_count++;
//         if (control_count % 100 == 0) {
//             unsigned long control_delta = now - last_control_time;
//             last_control_time = now;
//             printf("[CTRL] ~%.2f Hz (Δ %lu us)\n", 1e6 / (float)control_delta, control_delta);
//         }

//         vTaskDelay(pdMS_TO_TICKS(control_period_ms));
//     }
// }
