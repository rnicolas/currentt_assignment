/*
 * SPDX-FileCopyrightText: 2022-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SHT4X_MEASURE_HIGH_PRECISION        0xFD
#define SHT4X_MEASURE_MED_PRECISION         0xF6
#define SHT4X_MEASURE_LOW_PRECISION         0xE0
#define SHT4X_MEASURE_HEATING_200mW_1S      0x39
#define SHT4X_MEASURE_HEATING_200mW_0_1S    0x32
#define SHT4X_MEASURE_HEATING_110mW_1S      0x2F
#define SHT4X_MEASURE_HEATING_110mW_0_1S    0x24
#define SHT4X_MEASURE_HEATING_20mW_1S       0x1E
#define SHT4X_MEASURE_HEATING_20mW_0_1S     0x15

esp_err_t sht4x_soft_reset(i2c_master_bus_handle_t bus_handle);
esp_err_t sht4x_read_serial_number(i2c_master_bus_handle_t bus_handle, uint32_t *serial_number);
esp_err_t sht4x_measure_temp_humidity(i2c_master_bus_handle_t bus_handle, uint16_t command, float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif
