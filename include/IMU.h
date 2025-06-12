#pragma once

#include <stdint.h>
#include <stdbool.h>

// imu interface (i2c communication with imu sensor)
//
// this module provides an interface to communicate with an imu sensor over i2c.
// it handles initialization, data reading, calibration, and conversion of raw imu
// data into usable units (acceleration, angular velocity, temperature). the module
// also includes utility functions for low-level i2c read/write operations.

// imu i2c address (needs adjustment based on actual sensor)
#define IMU_I2C_ADDRESS      0x68

// imu register addresses (values may need adjustment for specific sensor)
#define IMU_REG_PWR_MGMT_1   0x6B
#define IMU_REG_ACCEL_XOUT_H 0x3B
#define IMU_REG_GYRO_XOUT_H  0x43

// structure for imu data
typedef struct {
    float accel_x;      // x-axis acceleration (g)
    float accel_y;      // y-axis acceleration (g)
    float accel_z;      // z-axis acceleration (g)
    float gyro_x;       // x-axis angular velocity (degrees/s)
    float gyro_y;       // y-axis angular velocity (degrees/s)
    float gyro_z;       // z-axis angular velocity (degrees/s)
    float temperature;  // temperature (celsius)
} IMU_Data_t;

extern float offset_accel_x;
extern float offset_accel_y;
extern float offset_accel_z;
extern float offset_gyro_x;
extern float offset_gyro_y;
extern float offset_gyro_z;
extern float offset_pitch;
// function declaration

/**
 * @brief initializes the imu sensor and i2c communication.
 * @return true if initialization succeeds, false if it fails.
 */
bool IMU_Init(void);

/**
 * @brief reads data from the imu sensor.
 * @param[out] data pointer to store the imu data.
 * @return true if read succeeds, false if it fails.
 */
bool IMU_ReadData(IMU_Data_t* data);

/**
 * @brief converts raw imu data into physical units (e.g., degrees/s).
 * @param[in,out] data pointer to the data to convert.
 */
void IMU_ConvertData(IMU_Data_t* data);

/**
 * @brief calibrates the imu sensor.
 * @return true if calibration succeeds, false if it fails.
 */
bool IMU_Calibrate(void);

/**
 * @brief checks if the imu is connected and responsive.
 * @return true if connected, false if not.
 */
bool IMU_IsConnected(void);

/**
 * @brief reads the temperature from the imu.
 * @return temperature in degrees celsius.
 */
float IMU_ReadTemperature(void);

/**
 * @brief reads accelerometer data from the imu.
 * @param[out] accel_x pointer to store x-axis acceleration.
 * @param[out] accel_y pointer to store y-axis acceleration.
 * @param[out] accel_z pointer to store z-axis acceleration.
 * @return true read succeeds, false if it fails
 */
bool IMU_ReadAccelerometer(float* accel_x, float* accel_y, float* accel_z);

/**
 * @brief reads gyroscope data from the imu.
 * @param[out] gyro_x pointer to store x-axis angular velocity.
 * @param[out] gyro_y pointer to store y-axis angular velocity.
 * @param[out] gyro_z pointer to store z-axis angular velocity.
 * @return true if read succeeds, false if it fails.
 */
bool IMU_ReadGyroscope(float* gyro_x, float* gyro_y, float* gyro_z);

/**
 * @brief reads raw data from the imu.
 * @param[out] raw_data pointer to store the raw bytes.
 * @return true if read succeeds, false if it fails.
 */
bool IMU_ReadRawData(uint8_t* raw_data);

/**
 * @brief writes data to an imu register.
 * @param[in] reg register address to write to.
 * @param[in] data data to write.
 * @return true if write succeeds, false if it fails.
 */
bool IMU_WriteData(uint8_t reg, uint8_t data);

/**
 * @brief reads a single byte from an imu register.
 * @param[in] reg register address to read from.
 * @param[out] data pointer to store the byte.
 * @return true if read succeeds, false if it fails.
 */
bool IMU_ReadByte(uint8_t reg, uint8_t* data);

/**
 * @brief reads multiple bytes from the imu.
 * @param[in] reg starting register address.
 * @param[out] data pointer to store the bytes.
 * @param[in] length number of bytes to read.
 * @return true if read succeeds, false if it fails.
 */
bool IMU_ReadBytes(uint8_t reg, uint8_t* data, uint16_t length);

/**
 * @brief writes a single byte to an imu register.
 * @param[in] reg register address to write to.
 * @param[in] data byte to write.
 * @return true if write succeeds, false if it fails.
 */
bool IMU_WriteByte(uint8_t reg, uint8_t data);

/**
 * @brief writes multiple bytes to the imu.
 * @param[in] reg starting register address.
 * @param[in] data pointer to the bytes to write.
 * @param[in] length number of bytes to write.
 * @return true if write succeeds, false if it fails.
 */
bool IMU_WriteBytes(uint8_t reg, const uint8_t* data, uint16_t length);

/**
 * @brief delays execution for a specified time.
 * @param[in] ms number of milliseconds to delay.
 */
void IMU_Delay(uint32_t ms);

/**
 * @brief resets the imu to its initial state.
 * @return true if reset succeeds, false if it fails.
 */
bool IMU_Reset(void);
