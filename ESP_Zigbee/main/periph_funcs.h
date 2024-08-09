#pragma once
//#include "driver/i2c_master.h"
#include "driver/i2c.h"

// I2C stuff for HDC1080
#define I2C_SCL GPIO_NUM_11
#define I2C_SDA GPIO_NUM_10
#define I2C_PORT 0
#define I2C_MASTER_TX_BUF_DISABLE 0
#define I2C_MASTER_RX_BUF_DISABLE 0
#define I2C_READ_TIMEOUT_PERIOD   ((TickType_t)200 / portTICK_PERIOD_MS)
#define HDC1080_ADDR 0x40
bool i2c_init(void);

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

// ADC configuration
#define BAT_ADC         ADC_UNIT_1
#define BAT_ADC_CH      ADC_CHANNEL_0   // GPIO 0
#define BAT_ADC_ATTEN   ADC_ATTEN_DB_12
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
void adc_calibration_deinit(adc_cali_handle_t handle);