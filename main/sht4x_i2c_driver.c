#include <stdio.h>
#include "driver/i2c_master.h"
#include "sdkconfig.h"
#include "sht4x_i2c_driver.h"
#include "esp_log.h"
#include "esp_err.h"

#define I2C_TIMEOUT_MS      50
#define SHT4X_I2C_ADDR      0x44
#define SHT4X_SERIAL_NUMBER 0x89

static const char *TAG = "SHT4X_DRIVER";

/**
 * @brief Computes the CRC-8 checksum using the polynomial 0x31 (x^8 + x^5 + x^4 + 1).
 *
 * This function calculates the CRC-8 checksum as per the Sensirion SHT4x sensor specification.
 * It processes each byte in the input data using bitwise XOR and shifts, ensuring data integrity.
 *
 * @param[in] data Pointer to the data buffer for which the CRC is to be computed.
 * @param[in] len Length of the data buffer in bytes.
 *
 * @return The computed 8-bit CRC checksum.
 *
 * @note The initial CRC value is 0xFF.
 * @note The polynomial used is 0x31 (x^8 + x^5 + x^4 + 1).
 * @note This CRC method is commonly used for Sensirion sensors.
 */
uint8_t sht4x_crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Sends a command to the SHT4x sensor over I2C.
 *
 * This function writes a single command byte to the SHT4x sensor via the I2C bus.
 * It temporarily registers the sensor as an I2C device, transmits the command,
 * and then removes the device from the bus.
 *
 * @param[in] bus_handle Handle to the I2C master bus.
 * @param[in] command The command byte to send to the sensor.
 *
 * @return 
 *      - `ESP_OK` if the command was successfully sent.
 *      - `ESP_FAIL` if the device could not be added to the I2C bus.
 *      - Error codes from `i2c_master_transmit()` in case of communication failure.
 *
 * @note The function temporarily registers the SHT4x device before sending the command 
 *       and removes it afterward.
 * @note The I2C communication speed is set to 100 kHz.
 */
esp_err_t sht4x_write_command(i2c_master_bus_handle_t bus_handle, uint8_t command) {
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 100000, // 100kHz
        .device_address = SHT4X_I2C_ADDR,
    };
    i2c_master_dev_handle_t dev_handle;
    
    if (i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ESP_FAIL;
    }
    
    esp_err_t ret = i2c_master_transmit(dev_handle, &command, 1, I2C_TIMEOUT_MS);
    
    if (i2c_master_bus_rm_device(dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device");
    }
    
    return ret;
}

/**
 * @brief Sends a command to the SHT4x sensor and reads the response data.
 *
 * This function communicates with the SHT4x sensor over I2C by sending a command 
 * and retrieving the resulting data. It dynamically adds the device to the I2C bus,
 * performs the transmission, and then removes the device from the bus.
 *
 * @param[in] bus_handle Handle to the I2C master bus.
 * @param[in] command Command byte to send to the sensor.
 * @param[out] data Pointer to a buffer where the received data will be stored.
 * @param[in] len Number of bytes to read from the sensor.
 *
 * @return 
 *      - `ESP_OK` if the operation was successful.
 *      - `ESP_FAIL` if the device could not be added to the I2C bus.
 *      - Error codes from `i2c_master_transmit_receive()` in case of communication failure.
 *
 * @note The function temporarily registers the sensor as an I2C device before communication
 *       and removes it afterward. Ensure the bus handle is properly initialized before calling this function.
 * @note The default I2C clock speed is set to 100 kHz.
 */
esp_err_t sht4x_read_data(i2c_master_bus_handle_t bus_handle, uint8_t command, uint8_t *data, size_t len) {
    i2c_device_config_t i2c_dev_conf = {
        .scl_speed_hz = 100000, // 100kHz
        .device_address = SHT4X_I2C_ADDR,
    };
    i2c_master_dev_handle_t dev_handle;
    
    if (i2c_master_bus_add_device(bus_handle, &i2c_dev_conf, &dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device");
        return ESP_FAIL;
    }
    
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &command, 1, data, len, I2C_TIMEOUT_MS);
    
    if (i2c_master_bus_rm_device(dev_handle) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to remove I2C device");
    }
    
    return ret;
}

/**
 * @brief Measures temperature and humidity using the SHT4x sensor.
 *
 * This function sends a measurement command to the SHT4x sensor, reads the data, 
 * performs CRC validation, and converts the raw values into temperature (°C) 
 * and relative humidity (%RH).
 *
 * @param[in] bus_handle Handle to the I2C master bus.
 * @param[in] command Measurement command specifying precision mode.
 * @param[out] temperature Pointer to store the measured temperature in degrees Celsius.
 * @param[out] humidity Pointer to store the measured relative humidity in percentage.
 *
 * @return 
 *      - `ESP_OK` if the measurement was successful.
 *      - `ESP_FAIL` if the sensor failed to respond.
 *      - `ESP_ERR_INVALID_CRC` if the CRC check failed.
 *      - `ESP_ERR_INVALID_RESPONSE` if the measured values are out of range.
 *
 * @note The function validates CRC checks for data integrity.
 * @note Temperature range: -40°C to 125°C, Humidity range: 0% to 100% RH.
 * @note Ensure the sensor has sufficient time (~8-20ms) to complete the measurement before calling this function.
 */
esp_err_t sht4x_measure_temp_humidity(i2c_master_bus_handle_t bus_handle, uint16_t command, float *temperature, float *humidity) {
    uint8_t data[6];
    esp_err_t ret = sht4x_read_data(bus_handle, command, data, sizeof(data));
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read data");
        return ESP_FAIL;
    }

    if (sht4x_crc8(&data[0], 2) != data[2] || sht4x_crc8(&data[3], 2) != data[5]) {
        ESP_LOGE(TAG, "CRC error");
        return ESP_ERR_INVALID_CRC;
    }
    
    uint16_t temp_raw = (data[0] << 8) | data[1];
    uint16_t hum_raw = (data[3] << 8) | data[4];
    
    *temperature = -45.0f + 175.0f * ((float)temp_raw / 65535.0f);
    *humidity = -6.0f + 125.0f * ((float)hum_raw / 65535.0f);
    
    if (*temperature < -40.0f || *temperature > 125.0f || *humidity < 0.0f || *humidity > 100.0f) {
        ESP_LOGW(TAG, "Invalid sensor readings, ignoring data");
        return ESP_ERR_INVALID_RESPONSE;
    }
    
    return ESP_OK;
}

/**
 * @brief Reads the serial number of the SHT4x sensor.
 *
 * This function sends the serial number read command to the SHT4x sensor,
 * retrieves the response, verifies the CRC checks, and extracts the serial number.
 *
 * @param[in] bus_handle Handle to the I2C master bus.
 * @param[out] serial_number Pointer to store the retrieved 32-bit serial number.
 *
 * @return 
 *      - `ESP_OK` if the serial number was successfully read and verified.
 *      - `ESP_FAIL` if the I2C read operation failed or CRC verification failed.
 *
 * @note The function performs CRC-8 validation on the received data to ensure integrity.
 */
esp_err_t sht4x_read_serial_number(i2c_master_bus_handle_t bus_handle, uint32_t *serial_number) {
    uint8_t data[6];
    esp_err_t ret = sht4x_read_data(bus_handle, SHT4X_SERIAL_NUMBER, data, sizeof(data));
    
    if (ret != ESP_OK || sht4x_crc8(&data[0], 2) != data[2] || sht4x_crc8(&data[3], 2) != data[5]) {
        return ESP_FAIL;
    }
    
    *serial_number = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[3] << 8) | (uint32_t)data[4];
    
    return ESP_OK;
}

/**
 * @brief Performs a soft reset of the SHT4x sensor.
 *
 * This function sends the soft reset command (0x94) to the SHT4x sensor over I2C.
 * A soft reset reinitializes the sensor without requiring a power cycle.
 *
 * @param[in] bus_handle Handle to the I2C master bus.
 *
 * @return 
 *      - `ESP_OK` on successful command transmission.
 *      - An appropriate `esp_err_t` error code on failure.
 *
 * @note Ensure that the I2C bus is initialized before calling this function.
 * @note The sensor requires a brief time (~1ms) to complete the reset.
 */
esp_err_t sht4x_soft_reset(i2c_master_bus_handle_t bus_handle) {
    return sht4x_write_command(bus_handle, 0x94);
}
