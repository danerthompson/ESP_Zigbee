// Written by Dane Thompson (Use at your own risk!)
// 8/19/2024
#pragma once

#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define HDC1080_CONVERSION_DELAY_MS 50          // 50 ms delay between requesting sensor readings and actually reading them
#define HDC1080_I2C_TRANFER_TIMEOUT_MS 50       // Timeout for I2C bus when talking to HDC1080

#define HDC1080_CONFIG_TEMP_14BIT_RES_MASK          0x0000
#define HDC1080_CONFIG_TEMP_11BIT_RES_MASK          0x0400
#define HDC1080_CONFIG_HUMIDITY_14BIT_RES_MASK      0x0000
#define HDC1080_CONFIG_HUMIDITY_11BIT_RES_MASK      0x0100
#define HDC1080_CONFIG_HUMIDITY_8BIT_RES_MASK       0x0200
#define HDC1080_CONFIG_HEATER_ENABLE_MASK           0x2000
#define HDC1080_CONFIG_HEATER_DISABLE_MASK          0x0000
#define HDC1080_CONFIG_TEMP_OR_HUMIDITY_MODE_MASK   0x0000  // Note, this mode is not implemented in the driver
#define HDC1080_CONFIG_TEMP_AND_HUMIDITY_MODE_MASK  0x1000

#define HDC1080_TEMP_REG_ADDR       0x00
#define HDC1080_CONFIG_REG_ADDR     0x02

typedef struct hdc1080_settings_struct {
    i2c_master_dev_handle_t device_handle;
    uint16_t configuration;
} hdc1080_settings_t;

esp_err_t HDC1080_Read_Vals(hdc1080_settings_t *hdc1080_settings, float* temperature, float* humidity);
esp_err_t HDC1080_Write_Configuration(hdc1080_settings_t *hdc1080_settings);