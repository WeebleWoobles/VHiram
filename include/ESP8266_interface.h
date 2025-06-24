#pragma once

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "nvs_flash.h"

// Structure to hold joystick states
typedef struct {
    bool forward;
    bool backward;
    bool left;
    bool right;
    bool data_received;
} joystick_data_t;

// ESP-NOW callback function
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

// Initialize ESP-NOW
esp_err_t esp_now_init_interface(void);

// Initialize WiFi for ESP-NOW
esp_err_t wifi_init_sta(void);

// Get current joystick data
joystick_data_t get_joystick_data(void);

// Clear the data received flag
void clear_joystick_data_flag(void);