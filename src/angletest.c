#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include "pid.h"        

float pitch = 0.0f;

void print_angles(void* pvParameters) {
    const TickType_t xDelay = pdMS_TO_TICKS(500); // Print every 500ms
    float prev_pitch = 0.0f;
    const float dt = 0.5f; // 500ms in seconds for pitch calculation

    while (1) {
        pitch = PID_get_pitch(prev_pitch, dt); // Update pitch
        printf("Angle: %.3f degrees\n", pitch); // Print angle with 3 decimal places
        prev_pitch = pitch;
        vTaskDelay(xDelay);
    }
}
