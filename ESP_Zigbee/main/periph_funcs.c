#include "esp_err.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "periph_funcs.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Initialize I2C driver
///////////////////////////////////////////////////////////////////////////////////////////
bool i2c_init(void)
{
    /* I2C MASTER MODE, PULLUPS ENABLED */
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_SDA,
        .sda_pullup_en = false,
        .scl_io_num = I2C_SCL,
        .scl_pullup_en = false,
        .master.clk_speed = 100000};
    /* CONFIGURE THE PORT */
    esp_err_t err = i2c_param_config(0, &i2c_conf);
    if (err != ESP_OK)
    {
        ESP_LOGE("I2C", "ERROR CONFIGURING I2C PORT %d", err);
        return false;
    }

    /* LOAD THE DRIVER */
    err = i2c_driver_install(I2C_PORT, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK)
    {
        if (err == ESP_ERR_INVALID_ARG)
        {
            ESP_LOGE("I2C", "ERROR INSTALLING I2C DRIVER, INVALID ARGUMENT");
        }
        else if (err == ESP_FAIL)
        {
            ESP_LOGE("I2C", "I2C DRIVER INSTALLATION FAILED!");
        }
        return false;
    }
    return true;
}
///////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
// Initialize ADC calibration (copied from sample)
///////////////////////////////////////////////////////////////////////////////////////////
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("ADC_CAL", "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI("ADC_CAL", "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI("ADC_CAL", "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW("ADC_CAL", "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE("ADC_CAL", "Invalid arg or no memory");
    }

    return calibrated;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Deinit ADC calibration
///////////////////////////////////////////////////////////////////////////////////////////
void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI("ADC_CAL", "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}
///////////////////////////////////////////////////////////////////////////////////////////