/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_log.h"
#include "sht4x_i2c_driver.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
 
static gpio_num_t i2c_gpio_sda = 14;
static gpio_num_t i2c_gpio_scl = 9;
 
i2c_master_bus_handle_t i2c_bus_handle;
static i2c_port_t i2c_port = I2C_NUM_0;
 
static esp_err_t err;
static SemaphoreHandle_t i2c_mutex;
static bool new_data = false;
float temperature, humidity;

static const char *TAG = "APP_MAIN";

/**
 * @brief Task to periodically read temperature and humidity from the SHT4x sensor.
 *
 * This task runs continuously and performs temperature and humidity measurements using
 * the SHT4x sensor. It implements retry mechanisms for both CRC failures and general 
 * communication failures. If CRC failures exceed a defined threshold, the sensor is reset.
 * If multiple resets occur without resolution, the ESP32 is restarted.
 *
 * @param pvParameters Unused task parameters (required by FreeRTOS).
 *
 * @note The function runs indefinitely and should be created as a FreeRTOS task.
 * @note Uses `i2c_mutex` for thread-safe I2C communication.
 */
void sht4x_read_task(void *pvParameters) {
    static int total_crc_failures = 0;
    const int max_crc_retries = 5;
    const int max_general_retries = 3;
    const int max_crc_failures = 10;
    while (1) {
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
            int crc_retry_count = 0;
            int general_retry_count = 0;

            do {
                err = sht4x_measure_temp_humidity(i2c_bus_handle, SHT4X_MEASURE_MED_PRECISION, &temperature, &humidity);
                if (err == ESP_OK) {
                    total_crc_failures = 0;
                    new_data = true;
                    break;
                } else if (err == ESP_ERR_INVALID_CRC) {
                    ESP_LOGW("SHT4X", "CRC mismatch, retrying... (%d/%d)", crc_retry_count + 1, max_crc_retries);
                    crc_retry_count++;
                } else {
                    ESP_LOGW("SHT4X", "Measurement failed (error: %s), retrying... (%d/%d)", 
                             esp_err_to_name(err), general_retry_count + 1, max_general_retries);
                    general_retry_count++;
                    // we retry, but if still fails, we just log it, in this case physical inspection must be done.
                }
                vTaskDelay(pdMS_TO_TICKS(100));
            } while (crc_retry_count < max_crc_retries && general_retry_count < max_general_retries);

            if (general_retry_count >= max_general_retries) {
                ESP_LOGE("SHT4X", "Measurement failed multiple times, check physical connections!");
            }

            if (crc_retry_count >= max_crc_retries) {
                ESP_LOGE("SHT4X", "Repeated CRC failures! Resetting sensor...");
                sht4x_soft_reset(i2c_bus_handle);
                vTaskDelay(pdMS_TO_TICKS(10));
                total_crc_failures++;

                ESP_LOGW("SHT4X", "Total CRC failures: %d", total_crc_failures);
                if (total_crc_failures >= max_crc_failures) {
                    // We have reset the sensor 5 times, restart ESP32
                    ESP_LOGE("SHT4X", "Critical sensor failure, restarting ESP32...");
                    esp_restart();
                }
            }

            xSemaphoreGive(i2c_mutex);
        }
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

/**
 * @brief Main application entry point for initializing and reading SHT4x sensor data.
 *
 * This function initializes the I2C bus and attempts to connect to the SHT4x sensor.
 * If initialization fails, it retries up to `max_retries` times before restarting 
 * the system. Once initialized, it reads the sensor's serial number and creates 
 * a mutex for safe I2C access. It then starts a task to periodically read temperature 
 * and humidity data from the sensor.
 *
 * @note Assumes `i2c_port`, `i2c_gpio_scl`, `i2c_gpio_sda`, `TAG`, `temperature`, 
 *       `humidity`, `new_data`, and `err` are declared globally.
 *
 * @warning If the I2C bus cannot be initialized after `max_retries` attempts, 
 *          the ESP32 will restart.
 */
void app_main(void) {
    int retry_count = 0;
    const int max_retries = 10;
    uint32_t serial_number = 0;

    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = i2c_port,
        .scl_io_num = i2c_gpio_scl,
        .sda_io_num = i2c_gpio_sda,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
 
    do {
        err = i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "I2C bus initialized successfully");
            break;
        }
    
        ESP_LOGE(TAG, "Failed to initialize I2C bus: %s. Retrying... (%d/%d)", 
                 esp_err_to_name(err), retry_count + 1, max_retries);
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // Wait 1 second before retrying
        retry_count++;
    
        if (retry_count >= max_retries) {
            ESP_LOGE(TAG, "I2C initialization failed after %d attempts! Restarting system...", max_retries);
            esp_restart();  // Soft reset the ESP32
        }
    } while (true);

    err = sht4x_read_serial_number(i2c_bus_handle, &serial_number);
    if (err == ESP_OK) {
        uint8_t *sn_bytes = (uint8_t *)&serial_number;
        ESP_LOGI(TAG,"Read success: 0x%02X 0x%02X 0x%02X 0x%02X", 
            sn_bytes[3], sn_bytes[2], sn_bytes[1], sn_bytes[0]);
    } else {
        ESP_LOGE(TAG, "Error reading SN: %s", esp_err_to_name(err));
        sht4x_soft_reset(i2c_bus_handle); //if error getting serial number,
        esp_restart();                    //restart esp32 and reset the sensor
    }

    i2c_mutex = xSemaphoreCreateMutex();
    if (i2c_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        esp_restart(); // Restart if mutex creation fails
    }

    xTaskCreate(sht4x_read_task, "SHT4X_Read_Task", 4096, NULL, 5, NULL);

    while (1) {
        if (xSemaphoreTake(i2c_mutex, portMAX_DELAY)) {
            if (new_data == true) {
                new_data = false;
                if (err == ESP_OK) {
                    ESP_LOGI(TAG,"Temperature: %.2f C, Humidity: %.2f", temperature, humidity);
                } else {
                    ESP_LOGE(TAG, "Error: %s", esp_err_to_name(err));
                }
            }
            xSemaphoreGive(i2c_mutex);
        }
    }
}
