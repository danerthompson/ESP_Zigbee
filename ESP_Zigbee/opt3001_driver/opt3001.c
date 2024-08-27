// Written by Dane Thompson (Use at your own risk!)
// 8/19/2024
#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "opt3001.h"

static const char TAG[] = "OPT3001_driver";

/// @brief Send read sensor request, wait for conversion, and read measurements
/// @param opt3001_settings Pointer to settings struct which contains I2C device handle
/// @param lux Float pointer to store received lux in
/// @return ESP_OK if measurement is read, ESP_ERR_TIMEOUT otherwise
esp_err_t OPT3001_Read_Lux(opt3001_settings_t *opt3001_settings, float* lux) {
    uint8_t buf_to_send = OPT3001_RESULT_REG_ADDR;
    if (i2c_master_transmit(opt3001_settings->device_handle, &buf_to_send, sizeof(buf_to_send), OPT3001_I2C_TRANFER_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGD(TAG, "Sent read request");
        vTaskDelay(pdMS_TO_TICKS(opt3001_settings->configuration & OPT3001_CONFIG_CONVERSION_TIME_800MS_MASK? 1000 : 200));
        uint8_t read_buf[2] = {0};
        if (i2c_master_receive(opt3001_settings->device_handle, read_buf, sizeof(read_buf), OPT3001_I2C_TRANFER_TIMEOUT_MS) == ESP_OK) {
            // Compute lux based on the datasheet
            uint8_t exponent = read_buf[0] >> 4;
            float lsb_size = 0.01;
            for (uint8_t i = 0; i < exponent; i++) {
                lsb_size *= 2.0;
            }
            *lux = lsb_size * (float)(((read_buf[0] & 0x0F) << 8)| read_buf[1]);

            ESP_LOGD(TAG, "Lux: %.3f", *lux);
            return ESP_OK;
        }
    }
    
    return ESP_ERR_TIMEOUT;
}

/// @brief Write configuration to opt3001
/// @param opt3001_settings Pointer to settings struct which contains I2C device handle and desired configuration settings
/// @return ESP_OK if configuration is written, ESP_ERR_TIMEOUT otherwise
esp_err_t OPT3001_Write_Configuration(opt3001_settings_t *opt3001_settings) {
    uint8_t buf_to_send[3];
    uint16_t configuration = opt3001_settings->configuration;
    buf_to_send[0] = OPT3001_CONFIG_REG_ADDR;
    buf_to_send[1] = configuration >> 8;
    buf_to_send[2] = configuration & 0x00FF;
    if (i2c_master_transmit(opt3001_settings->device_handle, buf_to_send, sizeof(buf_to_send), OPT3001_I2C_TRANFER_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGD(TAG, "Wrote to config register");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

