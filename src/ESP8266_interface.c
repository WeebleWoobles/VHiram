#include "ESP8266_interface.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_now.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "ESP_NOW_RECEIVER";

// Global joystick data structure
static joystick_data_t joystick_data = {0};

// ESP-NOW callback function for data reception
void esp_now_recv_cb(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len) {
    if (len >= 4) { // Ensure at least 4 bytes received
        joystick_data.forward  = data[0];
        joystick_data.backward = data[1];
        joystick_data.left     = data[2];
        joystick_data.right    = data[3];
        joystick_data.data_received = true;

        ESP_LOGI(TAG, "Joystick states received: Forward: %d | Backward: %d | Left: %d | Right: %d", 
                 joystick_data.forward, joystick_data.backward, joystick_data.left, joystick_data.right);
    } else {
        ESP_LOGW(TAG, "Received data too short! Length: %d", len);
    }
}

// Get current joystick data
joystick_data_t get_joystick_data(void) {
    return joystick_data;
}

// Clear the data received flag
void clear_joystick_data_flag(void) {
    joystick_data.data_received = false;
}

// Initialize WiFi in station mode for ESP-NOW
esp_err_t wifi_init_sta(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_sta();
    
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    ESP_LOGI(TAG, "WiFi initialized in station mode");
    return ESP_OK;
}

// Initialize ESP-NOW
esp_err_t esp_now_init_interface(void) {
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // Initialize WiFi
    ESP_ERROR_CHECK(wifi_init_sta());
    
    // Initialize ESP-NOW
    ESP_ERROR_CHECK(esp_now_init());
    
    // Register receive callback
    ESP_ERROR_CHECK(esp_now_register_recv_cb(esp_now_recv_cb));
    
    ESP_LOGI(TAG, "ESP-NOW initialized successfully");
    return ESP_OK;
}