// Written by Dane Thompson (Use at your own risk!)
// 8/22/2024
#pragma once

#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define OPT3001_I2C_TRANFER_TIMEOUT_MS 50       // Timeout for I2C bus when talking to HDC1080

#define OPT3001_CONFIG_AUTO_FULLSCALE_RANGE_MASK  0xC000             
#define OPT3001_CONFIG_CONVERSION_TIME_100MS_MASK 0x0000
#define OPT3001_CONFIG_CONVERSION_TIME_800MS_MASK 0x0800
#define OPT3001_CONFIG_CONV_MODE_SHUTDOWN_MASK    0x0000
#define OPT3001_CONFIG_CONV_MODE_SINGLESHOT_MASK  0x0200
#define OPT3001_CONFIG_CONV_MODE_CONTINUOUS_MASK  0x0600    // Not implemented
#define OPT3001_CONFIG_LATCH_LATCHED_MASK         0x0010    // This is the default
#define OPT3001_CONFIG_LATCH_HYSTERESIS_MASK      0x0000 
// NOT IMPLEMENTED: polarity field, mask exponent field, fault count field

#define OPT3001_RESULT_REG_ADDR     0x00
#define OPT3001_CONFIG_REG_ADDR     0x01

typedef struct opt3001_settings_struct {
    i2c_master_dev_handle_t device_handle;
    uint16_t configuration;
} opt3001_settings_t;

esp_err_t OPT3001_Read_Lux(opt3001_settings_t *opt3001_settings, float* lux);
esp_err_t OPT3001_Write_Configuration(opt3001_settings_t *opt3001_settings);