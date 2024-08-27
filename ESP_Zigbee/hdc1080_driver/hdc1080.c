// Written by Dane Thompson (Use at your own risk!)
// 8/19/2024
#include <stdio.h>

#include "esp_check.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hdc1080.h"

static const char TAG[] = "HDC1080_driver";

/// @brief Send read sensor request, wait for conversion, and read measurements
/// @param hdc1080_settings Pointer to settings struct which contains I2C device handle
/// @param temperature Float pointer to store received temperature in
/// @param humidity Float pointer to store received humidity in
/// @return ESP_OK if measurement is read, ESP_ERR_TIMEOUT otherwise
esp_err_t HDC1080_Read_Vals(hdc1080_settings_t *hdc1080_settings, float* temperature, float* humidity) {
    uint8_t buf_to_send = HDC1080_TEMP_REG_ADDR;
    if (i2c_master_transmit(hdc1080_settings->device_handle, &buf_to_send, sizeof(buf_to_send), HDC1080_I2C_TRANFER_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGD(TAG, "Sent read request");
        vTaskDelay(pdMS_TO_TICKS(HDC1080_CONVERSION_DELAY_MS));
        uint8_t read_buf[4] = {0};
        if (i2c_master_receive(hdc1080_settings->device_handle, read_buf, sizeof(read_buf), HDC1080_I2C_TRANFER_TIMEOUT_MS) == ESP_OK) {
            ESP_LOGD(TAG, "%02X, %02X, %02X, %02X", read_buf[0], read_buf[1], read_buf[2], read_buf[3]);
            *temperature = (float)(((read_buf[0]<<8)|read_buf[1]) / 65536.0 * 165.0) - 40.0;
            *humidity = (float)(((read_buf[2]<<8)|read_buf[3]) / 65536.0 * 100.0);
            ESP_LOGD(TAG, "Temp: %.3f, Humidity: %.3f", *temperature, *humidity);
            return ESP_OK;
        }
    }
    return ESP_ERR_TIMEOUT;
}

/// @brief Write configuration to HDC1080
/// @param hdc1080_settings Pointer to settings struct which contains I2C device handle and desired configuration settings
/// @return ESP_OK if configuration is written, ESP_ERR_TIMEOUT otherwise
esp_err_t HDC1080_Write_Configuration(hdc1080_settings_t *hdc1080_settings) {
    uint8_t buf_to_send[3];
    uint16_t configuration = hdc1080_settings->configuration;
    configuration = configuration & 0xB700; // Ensure that no read only bits are written to
    buf_to_send[0] = HDC1080_CONFIG_REG_ADDR;
    buf_to_send[1] = configuration >> 8;
    buf_to_send[2] = configuration & 0x00FF;
    if (i2c_master_transmit(hdc1080_settings->device_handle, buf_to_send, sizeof(buf_to_send), HDC1080_I2C_TRANFER_TIMEOUT_MS) == ESP_OK) {
        ESP_LOGD(TAG, "Wrote to config register");
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

