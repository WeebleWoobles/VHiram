#include "IMU.h"
#include "main.h"
#include "driver/i2c.h"
#include <math.h>

// Define I2C settings if not already defined
#ifndef I2C_NUM_0
#define I2C_NUM_0 ((i2c_port_t)0)
#endif

#define I2C_MASTER_SCL_IO GPIO_NUM_22
#define I2C_MASTER_SDA_IO GPIO_NUM_21
#define I2C_MASTER_FREQ_HZ 100000

// IMU connection flag
static bool is_initialized = false;

// Calibration offsets
float offset_accel_x = 0.0f;
float offset_accel_y = 0.0f;
float offset_accel_z = 0.0f;
float offset_gyro_x = 0.0f;
float offset_gyro_y = 0.0f;
float offset_gyro_z = 0.0f;
float offset_pitch = 0.0f;

// Forward declarations
bool IMU_WriteByte(uint8_t reg, uint8_t data);

bool IMU_Init(void)
{
    if (is_initialized) return true;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    esp_err_t ret = i2c_param_config(I2C_NUM_0, &conf);
    if (ret != ESP_OK) return false;

    ret = i2c_driver_install(I2C_NUM_0, conf.mode, 0, 0, 0);
    if (ret != ESP_OK) return false;

    // Wake up IMU
    if (!IMU_WriteByte(0x6B, 0x00)) return false;
    IMU_Delay(100);

    // Set clock source
    if (!IMU_WriteByte(0x6B, 0x01)) return false;
    IMU_Delay(10);

    // Enable all sensors
    if (!IMU_WriteByte(0x6C, 0x00)) return false;
    IMU_Delay(10);

    // Set Gyroscope Full Scale to ±2000 dps
    if (!IMU_WriteByte(0x1B, 0x18)) return false;
    IMU_Delay(10);

    // Set Accelerometer Full Scale to ±16g
    if (!IMU_WriteByte(0x1C, 0x18)) return false;
    IMU_Delay(10);

    // Disable DLPF for max bandwidth
    if (!IMU_WriteByte(0x1A, 0x00)) return false;
    IMU_Delay(10);

    // Max sample rate (divider = 0)
    if (!IMU_WriteByte(0x19, 0x00)) return false;
    IMU_Delay(10);

    uint8_t whoami = 0;
    if (!IMU_ReadByte(0x75, &whoami)) {
        return false;
    }

    is_initialized = true;
    return true;
}

bool IMU_ReadData(IMU_Data_t* data)
{
    if (!is_initialized || data == NULL) return false;

    uint8_t raw_data[14];
    if (!IMU_ReadBytes(IMU_REG_ACCEL_XOUT_H, raw_data, 14)) return false;

    data->accel_x = (int16_t)((raw_data[0] << 8) | raw_data[1]) / 16384.0f;
    data->accel_y = (int16_t)((raw_data[2] << 8) | raw_data[3]) / 16384.0f;
    data->accel_z = (int16_t)((raw_data[4] << 8) | raw_data[5]) / 16384.0f;
    data->temperature = (int16_t)((raw_data[6] << 8) | raw_data[7]) / 340.0f + 36.53f;
    data->gyro_x = (int16_t)((raw_data[8] << 8) | raw_data[9]) / 131.0f;
    data->gyro_y = (int16_t)((raw_data[10] << 8) | raw_data[11]) / 131.0f;
    data->gyro_z = (int16_t)((raw_data[12] << 8) | raw_data[13]) / 131.0f;

    IMU_ConvertData(data);

    float pitch = atan2f(data->accel_x, sqrtf(data->accel_y * data->accel_y + data->accel_z * data->accel_z)) * (180.0f / M_PI);

    if (pitch > 5.0f) {
        // printf("Tilting FORWARD: %.2f\u00b0\n", pitch);
    } else if (pitch < -5.0f) {
        // printf("Tilting BACKWARD: %.2f\u00b0\n", pitch);
    } else {
        // printf("UPRIGHT: %.2f\u00b0\n", pitch);
    }

    return true;
}

void IMU_ConvertData(IMU_Data_t* data)
{
    (void)data;
}

bool IMU_Calibrate(void)
{
    if (!is_initialized) return false;

    IMU_Data_t sum = {0};
    double pitch_sum = 0.0;

    for (int i = 0; i < 100; i++) {
        IMU_Data_t temp;
        if (!IMU_ReadData(&temp)) return false;

        sum.accel_x += temp.accel_x;
        sum.accel_y += temp.accel_y;
        sum.accel_z += temp.accel_z;
        sum.gyro_x += temp.gyro_x;
        sum.gyro_y += temp.gyro_y;
        sum.gyro_z += temp.gyro_z;

        pitch_sum += atan2f(temp.accel_y, sqrtf(temp.accel_x * temp.accel_x + temp.accel_z * temp.accel_z)) * (180.0f / M_PI);

        IMU_Delay(10);
    }

    offset_accel_x = sum.accel_x / 100.0f;
    offset_accel_y = sum.accel_y / 100.0f;
    offset_accel_z = (sum.accel_z / 100.0f) - 1.0f;

    offset_gyro_x = sum.gyro_x / 100.0f;
    offset_gyro_y = sum.gyro_y / 100.0f;
    offset_gyro_z = sum.gyro_z / 100.0f;

    offset_pitch = pitch_sum / 100.0f;

    IMU_Delay(10);
    return true;
}

bool IMU_IsConnected(void)
{
    if (!is_initialized) return false;

    uint8_t data = 0;
    return IMU_ReadByte(IMU_REG_PWR_MGMT_1, &data);
}

float IMU_ReadTemperature(void)
{
    IMU_Data_t data;
    if (!IMU_ReadData(&data)) return 0.0f;
    return data.temperature;
}

bool IMU_ReadAccelerometer(float* accel_x, float* accel_y, float* accel_z)
{
    if (!is_initialized || accel_x == NULL || accel_y == NULL || accel_z == NULL) return false;

    IMU_Data_t data;
    if (!IMU_ReadData(&data)) return false;

    *accel_x = data.accel_x;
    *accel_y = data.accel_y;
    *accel_z = data.accel_z;
    return true;
}

bool IMU_ReadGyroscope(float* gyro_x, float* gyro_y, float* gyro_z)
{
    if (!is_initialized || gyro_x == NULL || gyro_y == NULL || gyro_z == NULL) return false;

    IMU_Data_t data;
    if (!IMU_ReadData(&data)) return false;

    *gyro_x = data.gyro_x;
    *gyro_y = data.gyro_y;
    *gyro_z = data.gyro_z;
    return true;
}

bool IMU_ReadRawData(uint8_t* raw_data)
{
    if (!is_initialized || raw_data == NULL) return false;
    return IMU_ReadBytes(IMU_REG_ACCEL_XOUT_H, raw_data, 14);
}

bool IMU_WriteData(uint8_t reg, uint8_t data)
{
    if (!is_initialized) return false;

    uint8_t buf[2] = {reg, data};
    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_ADDRESS, buf, 2, pdMS_TO_TICKS(100));
    return (ret == ESP_OK);
}

bool IMU_ReadByte(uint8_t reg, uint8_t* data)
{
    if (!is_initialized || data == NULL) return false;

    esp_err_t ret = i2c_master_write_read_device(I2C_NUM_0, IMU_I2C_ADDRESS, &reg, 1, data, 1, pdMS_TO_TICKS(100));
    return (ret == ESP_OK);
}

bool IMU_ReadBytes(uint8_t reg, uint8_t* data, uint16_t length)
{
    if (!is_initialized || data == NULL) return false;

    esp_err_t ret = i2c_master_write_read_device(I2C_NUM_0, IMU_I2C_ADDRESS, &reg, 1, data, length, pdMS_TO_TICKS(100));
    return (ret == ESP_OK);
}

bool IMU_WriteByte(uint8_t reg, uint8_t data)
{
    return IMU_WriteData(reg, data);
}

bool IMU_WriteBytes(uint8_t reg, const uint8_t* data, uint16_t length)
{
    if (!is_initialized || data == NULL) return false;

    uint8_t buf[1 + length];
    buf[0] = reg;
    for (uint16_t i = 0; i < length; i++) {
        buf[i + 1] = data[i];
    }

    esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0, IMU_I2C_ADDRESS, buf, length + 1, pdMS_TO_TICKS(100));
    return (ret == ESP_OK);
}

void IMU_Delay(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

bool IMU_Reset(void)
{
    if (!is_initialized) return false;

    if (!IMU_WriteByte(IMU_REG_PWR_MGMT_1, 0x80)) return false;
    IMU_Delay(100);
    return true;
}
