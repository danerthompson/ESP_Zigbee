#include "zigbee_defs.h"
#include "periph_funcs.h"
#include "hdc1080.h"
#include "opt3001.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_pm.h"
#include "esp_private/esp_clk.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "math.h"

///////////////////////////////////////////////////////////////////////////////////////////
// Constant definitions (including pin numbers)
///////////////////////////////////////////////////////////////////////////////////////////
#define MAJOR_SOFTWARE_VER 0
#define MINOR_SOFTWARE_VER 1

#define PIR_GPIO GPIO_NUM_3
#define LEDP_GPIO GPIO_NUM_23
#define LED0_GPIO GPIO_NUM_21
#define LED1_GPIO GPIO_NUM_22
#define IDENTIFY_LED_GPIO LED0_GPIO
#define NETWORK_LED_GPIO LED1_GPIO
#define BUTT0_GPIO GPIO_NUM_7
#define BAT_ADC_EN_GPIO GPIO_NUM_20
#define SENSOR_UPDATE_PERIOD_SEC 300
#define ZIGBEE_SLEEP_TIME_MS 9000
#define ADC_DELAY_MS 0  // Might be needed if using cap on ADC pin

// I2C stuff for HDC1080 and OPT3001
#define I2C_SCL GPIO_NUM_11
#define I2C_SDA GPIO_NUM_10
#define I2C_PORT I2C_NUM_0
#define HDC1080_ADDR 0x40
#define OPT3001_ADDR 0x45

// ADC configuration
#define BAT_ADC         ADC_UNIT_1
#define BAT_ADC_CH      ADC_CHANNEL_2   // GPIO 2
#define BAT_ADC_ATTEN   ADC_ATTEN_DB_0  // No attenuation needed if using AA battery and 1/2 voltage divider

#define ESP_INTR_FLAG_DEFAULT 0
#define LONG_PRESS_TIME_MS 1000
#define LED_TURNOFF_TIMEOUT_MS 30000
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// ADC variables
///////////////////////////////////////////////////////////////////////////////////////////
adc_oneshot_unit_handle_t bat_adc_handle;
adc_oneshot_unit_init_cfg_t bat_adc_init_config = {
    .unit_id = BAT_ADC,
};
adc_oneshot_chan_cfg_t bat_adc_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = BAT_ADC_ATTEN,
};
adc_cali_handle_t bat_adc_cali_handle = NULL;
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// I2C variables
///////////////////////////////////////////////////////////////////////////////////////////
i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .scl_io_num = I2C_SCL,
    .sda_io_num = I2C_SDA,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};
i2c_master_bus_handle_t i2c_bus_handle;

i2c_master_dev_handle_t hdc1080_dev_handle;
hdc1080_settings_t hdc1080_settings;
i2c_device_config_t hdc1080_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = HDC1080_ADDR,
    .scl_speed_hz = 100000,
    .flags.disable_ack_check = true,
};

i2c_master_dev_handle_t opt3001_dev_handle;
opt3001_settings_t opt3001_settings;
i2c_device_config_t opt3001_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = OPT3001_ADDR,
    .scl_speed_hz = 100000,
    .flags.disable_ack_check = true,
};
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Zigbee variables
///////////////////////////////////////////////////////////////////////////////////////////
uint8_t zb_started_flag = 0;            // Flag is set after Zigbee stack is initialized
uint8_t first_sensor_sample_flag = 0;   // Flag is set after first sensor sample occurs (stops Zigbee from reporting 0s on restart)
uint8_t hdc1080_done_flag = 0;          // Set after reading callback has been called
int zb_network_status = 0;              // 0 for unjoined, 1 for joined, -1 for left
uint8_t zb_unavailable_alarm_flag = 0;  // 0 if alarm has not been set, 1 if alarm is set
uint8_t zb_steering_alarm_flag = 0;     // 0 if alarm has not been set, 1 if alarm is set

int16_t zb_bat_voltage, zb_temperature;
uint16_t zb_humidity, zb_illuminance;
uint16_t zb_identify_time = 0;
uint8_t zb_currently_identifying = 0;

uint8_t PIR_interrupt_type, BUTT0_interrupt_type;
uint8_t leds_enabled = 0;       // When set, allow network LED to turn on
uint8_t long_press_alarm_scheduled = 0;
uint8_t led_turnoff_alarm_scheduled = 0;
///////////////////////////////////////////////////////////////////////////////////////////

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

static const char *TAG = "ESP_STICKBEE";

///////////////////////////////////////////////////////////////////////////////////////////
// Update PIR sensor attribute when interrupt is called
///////////////////////////////////////////////////////////////////////////////////////////
TaskHandle_t pir_sensor_update_handle = NULL;
void pir_sensor_update(void *arg) {
    //uint32_t io_num;
    uint8_t zb_occupancy;
    while (1) {
        zb_occupancy = gpio_get_level(PIR_GPIO);
        PIR_interrupt_type = zb_occupancy? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;

        // Change interrupt level
        ESP_ERROR_CHECK(gpio_set_intr_type(PIR_GPIO, PIR_interrupt_type));
        // Change wakeup level on GPIO (also changes the button wakeup level, but this isn't as critical as PIR)
        // If looking for LOW wakeup, add BUTT0 to wakeup, if HIGH, don't include BUTT0
        ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup((1ULL << PIR_GPIO) | ((zb_occupancy? 1ULL : 0) << BUTT0_GPIO), zb_occupancy? ESP_EXT1_WAKEUP_ANY_LOW : ESP_EXT1_WAKEUP_ANY_HIGH));

        //ESP_ERROR_CHECK(gpio_wakeup_disable(PIR_GPIO));
        //ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO));
        //ESP_ERROR_CHECK(gpio_wakeup_enable(PIR_GPIO, PIR_interrupt_type));   
        //ESP_ERROR_CHECK(gpio_wakeup_enable(BUTT0_GPIO, BUTT0_interrupt_type));    // Make sure BUTT0 wakeup stays enabled
        //ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

        // If Zigbee has started, update the attribute
        if (zb_started_flag == 1) {
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &zb_occupancy, false);
            esp_zb_lock_release();
        }
        ESP_LOGI(TAG, "PIR sensor: %d", zb_occupancy);
        vTaskSuspend(NULL); // Suspend task until resumed by ISR
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Timer callback for LED turnoff
///////////////////////////////////////////////////////////////////////////////////////////
static void led_turnoff_timer_callback(uint8_t input) {
    ESP_LOGI(TAG, "LED turnoff timer callback");
    led_turnoff_alarm_scheduled = 0;
    // Turn off LEDs
    leds_enabled = 0;
    gpio_hold_dis(IDENTIFY_LED_GPIO);
    gpio_set_direction(IDENTIFY_LED_GPIO, GPIO_MODE_DISABLE);
    gpio_hold_dis(NETWORK_LED_GPIO);
    gpio_set_direction(NETWORK_LED_GPIO, GPIO_MODE_DISABLE);
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Timer callback for button long press
///////////////////////////////////////////////////////////////////////////////////////////
static void long_press_timer_callback(uint8_t input) {
    ESP_LOGI(TAG, "Long press timer callback");
    long_press_alarm_scheduled = 0;
    // If button is still pressed
    if (gpio_get_level(BUTT0_GPIO) == 0) {
        //i2c_master_bus_rm_device(hdc1080_dev_handle);  // Should be redundant if deleting the bus
        //i2c_master_bus_rm_device(opt3001_dev_handle);  // Should be redundant if deleting the bus
        //i2c_del_master_bus(i2c_bus_handle);
        gpio_uninstall_isr_service();
        gpio_hold_dis(LED0_GPIO);
        gpio_hold_dis(LED1_GPIO);
        gpio_hold_dis(LEDP_GPIO);
        ESP_LOGI(TAG, "Factory resetting");
        vTaskDelay(pdMS_TO_TICKS(100));
        esp_zb_factory_reset();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Handle button press/release
///////////////////////////////////////////////////////////////////////////////////////////
TaskHandle_t button_handler_handle = NULL;
void button_handler(void *arg) {
    uint8_t button_level;
    while(1) {
        button_level = gpio_get_level(BUTT0_GPIO);
        BUTT0_interrupt_type = button_level? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;

        // Wait for Zigbee stack to be started
        if (zb_started_flag) {
            // Button pressed
            if (button_level == 0) {
                // Cancel current alarm
                if (long_press_alarm_scheduled) {
                    esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)long_press_timer_callback, 0);
                    long_press_alarm_scheduled = 0;
                }
                // Begin recording for long press
                esp_zb_scheduler_alarm((esp_zb_callback_t)long_press_timer_callback, 0, LONG_PRESS_TIME_MS);
                long_press_alarm_scheduled = 1;

                if (leds_enabled == 0) {
                    // Turn on identify LED
                    gpio_set_direction(IDENTIFY_LED_GPIO, GPIO_MODE_OUTPUT);
                    gpio_hold_dis(IDENTIFY_LED_GPIO);
                    gpio_set_level(IDENTIFY_LED_GPIO, 0);
                    gpio_hold_en(IDENTIFY_LED_GPIO);
                    // Set network LED
                    if (esp_zb_bdb_dev_joined()) {
                        gpio_set_direction(NETWORK_LED_GPIO, GPIO_MODE_OUTPUT);
                        gpio_hold_dis(NETWORK_LED_GPIO);
                        gpio_set_level(NETWORK_LED_GPIO, 0);
                        gpio_hold_en(NETWORK_LED_GPIO);
                    }
                    leds_enabled = 1;

                    // Cancel current alarm
                    if (led_turnoff_alarm_scheduled) {
                        esp_zb_lock_acquire(portMAX_DELAY);
                        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)led_turnoff_timer_callback, 0);
                        esp_zb_lock_release();
                        led_turnoff_alarm_scheduled = 0;
                    }
                    // Begin timer to turn off LEDs
                    esp_zb_lock_acquire(portMAX_DELAY);
                    esp_zb_scheduler_alarm((esp_zb_callback_t)led_turnoff_timer_callback, 0, LED_TURNOFF_TIMEOUT_MS);
                    esp_zb_lock_release();
                    led_turnoff_alarm_scheduled = 1;
                    
                }
                else {
                    // Turn off LEDs
                    leds_enabled = 0;
                    gpio_hold_dis(IDENTIFY_LED_GPIO);
                    gpio_set_direction(IDENTIFY_LED_GPIO, GPIO_MODE_DISABLE);
                    gpio_hold_dis(NETWORK_LED_GPIO);
                    gpio_set_direction(NETWORK_LED_GPIO, GPIO_MODE_DISABLE);

                    // Cancel current alarm
                    if (led_turnoff_alarm_scheduled) {
                        esp_zb_lock_acquire(portMAX_DELAY);
                        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)led_turnoff_timer_callback, 0);
                        esp_zb_lock_release();
                        led_turnoff_alarm_scheduled = 0;
                    }

                }
                
            }
        }

        // Change interrupt level
        ESP_ERROR_CHECK(gpio_set_intr_type(BUTT0_GPIO, BUTT0_interrupt_type));
        // Change wakeup level on GPIO
        //ESP_ERROR_CHECK(gpio_wakeup_disable(BUTT0_GPIO));
        //ESP_ERROR_CHECK(esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_GPIO));
        //ESP_ERROR_CHECK(gpio_wakeup_enable(BUTT0_GPIO, BUTT0_interrupt_type));   
        //ESP_ERROR_CHECK(gpio_wakeup_enable(PIR_GPIO, PIR_interrupt_type));  // Make sure PIR wakeup stays enabled
        //ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());

        ESP_LOGI(TAG, "Button %s", button_level? "released" : "pressed");
        vTaskSuspend(NULL);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Set up PIR sensor interrupt
///////////////////////////////////////////////////////////////////////////////////////////
static QueueHandle_t gpio_isr_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    switch (gpio_num) {
        case PIR_GPIO:
            // Disable interrupt (will be changed to occur on opposite logic level in task)
            gpio_set_intr_type(PIR_GPIO, GPIO_INTR_DISABLE);
            xHigherPriorityTaskWoken = xTaskResumeFromISR(pir_sensor_update_handle);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;

        case BUTT0_GPIO:
            // Disable interrupt (will be changed to occur on opposite logic level in task)
            gpio_set_intr_type(BUTT0_GPIO, GPIO_INTR_DISABLE);
            xHigherPriorityTaskWoken = xTaskResumeFromISR(button_handler_handle);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            break;
    }

}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Function to deinit peripherals (they aren't reset on software restart) and reset ESP
///////////////////////////////////////////////////////////////////////////////////////////
static void deinit_and_restart() {
    //i2c_master_bus_rm_device(hdc1080_dev_handle);  // Should be redundant if deleting the bus
    //i2c_master_bus_rm_device(opt3001_dev_handle);  // Should be redundant if deleting the bus
    //i2c_del_master_bus(i2c_bus_handle);
    //adc_calibration_deinit(bat_adc_cali_handle);
    //adc_oneshot_del_unit(bat_adc_handle);
    gpio_uninstall_isr_service();
    gpio_hold_dis(LED0_GPIO);
    gpio_hold_dis(LED1_GPIO);
    gpio_hold_dis(LEDP_GPIO);
    vTaskDelay(pdMS_TO_TICKS(100));
    esp_restart();
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Function to start top level Zigbee commissioning
///////////////////////////////////////////////////////////////////////////////////////////
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Function to handle when Zigbee network is unavilable
///////////////////////////////////////////////////////////////////////////////////////////
static void handle_zigbee_unavilable(uint8_t input) {
    zb_unavailable_alarm_flag = 0;
    if (zb_network_status != 1) {   // If the device hasn't joined a zigbee network
        //esp_zb_factory_reset();
        deinit_and_restart();
        //bdb_start_top_level_commissioning_cb(ESP_ZB_BDB_MODE_NETWORK_STEERING);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Configure ADC
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t adc_init(void) {
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&bat_adc_init_config, &bat_adc_handle), TAG, "Failed to create ADC handle");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(bat_adc_handle, BAT_ADC_CH, &bat_adc_config), TAG, "Failed to config ADC channel");

    if (adc_calibration_init(BAT_ADC, BAT_ADC_CH, BAT_ADC_ATTEN, &bat_adc_cali_handle)!= true) {
        return !ESP_OK;
    }
    return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Function to update all sensors (HDC1080 and ADC)
///////////////////////////////////////////////////////////////////////////////////////////
static void update_all_sensors(void *arg) {
    int adc_raw, voltage;
    float temperature, humidity, lux;

    while (1) {

        ESP_LOGI(TAG, "UPDATE ALL SENSORS:");

        // Acquire lock to make sure Zigbee stuff isn't called during
        if (zb_started_flag == 1) {
            esp_zb_lock_acquire(portMAX_DELAY);
        }

        // Lock power management to stop light sleep from occurring during delay
        esp_pm_lock_handle_t pm_lock;
        esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "no_light_sleep", &pm_lock);
        esp_pm_lock_acquire(pm_lock);

        // Reset I2C bus pins
        gpio_reset_pin(I2C_SCL);
        gpio_reset_pin(I2C_SDA);

        // Initialize I2C bus
        i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle);

        // Add HDC1080 to I2C bus driver
        i2c_master_bus_add_device(i2c_bus_handle, &hdc1080_dev_cfg, &hdc1080_dev_handle);
        hdc1080_settings.device_handle = hdc1080_dev_handle;    // Update device handle
        hdc1080_settings.configuration = HDC1080_CONFIG_TEMP_14BIT_RES_MASK | HDC1080_CONFIG_HUMIDITY_14BIT_RES_MASK | 
                                         HDC1080_CONFIG_HEATER_DISABLE_MASK | HDC1080_CONFIG_TEMP_AND_HUMIDITY_MODE_MASK;
        // Add OPT3001 to I2C bus driver
        i2c_master_bus_add_device(i2c_bus_handle, &opt3001_dev_cfg, &opt3001_dev_handle);
        opt3001_settings.device_handle = opt3001_dev_handle;    // Update device handle
        opt3001_settings.configuration = OPT3001_CONFIG_AUTO_FULLSCALE_RANGE_MASK | OPT3001_CONFIG_CONVERSION_TIME_100MS_MASK | 
                                         OPT3001_CONFIG_CONV_MODE_SINGLESHOT_MASK | OPT3001_CONFIG_LATCH_LATCHED_MASK;
        
        // Read HDC1080
        HDC1080_Write_Configuration(&hdc1080_settings); // Don't actually have to do this every time, but it shouldn't hurt
        if (HDC1080_Read_Vals(&hdc1080_settings, &temperature, &humidity) == ESP_OK) {

            zb_temperature = (int16_t) ((temperature+0.005)*100.0);   // Add 0.005 to do rounding instead of truncating
            zb_humidity = (uint16_t) ((humidity+0.005)*100.0);         // Add 0.005 to do rounding instead of truncating

            ESP_LOGI("SENSOR_DATA", "TEMPERATURE: %.2f°C", temperature);
            ESP_LOGI("SENSOR_DATA", "HUMIDITY: %.2f%%", humidity);

            // If Zigbee stack has been started, update attributes
            if (zb_started_flag == 1) {
                esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                    ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &zb_temperature, false);

                esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                    ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &zb_humidity, false);
            }
        }
        else {
            ESP_LOGE(TAG, "Error reading HDC1080");
        }

        // Read OPT3001
        OPT3001_Write_Configuration(&opt3001_settings); // Must write configuration to trigger single shot conversion
        // Wait for conversion to finish
        vTaskDelay(pdMS_TO_TICKS(opt3001_settings.configuration & OPT3001_CONFIG_CONVERSION_TIME_800MS_MASK? 1000 : 200));
        if (OPT3001_Read_Lux(&opt3001_settings, &lux) == ESP_OK) {

            ESP_LOGI("SENSOR_DATA", "LUX: %.2f", lux);

            if (lux < 1) {  // Minimum value in ZCL definition
                zb_illuminance = 0;
            }
            else {
                zb_illuminance = (uint16_t) (10000.0 * log10f(lux)) + 1;
            }

            // If Zigbee stack has been started, update attribute
            if (zb_started_flag == 1) {
                esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                    ESP_ZB_ZCL_CLUSTER_ID_ILLUMINANCE_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_ILLUMINANCE_MEASUREMENT_MEASURED_VALUE_ID, &zb_illuminance, false);
            }           
        }
        else {
            ESP_LOGE(TAG, "Error reading OPT3001");
        }

        // Delete I2C bus and reset pins
        ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus_handle));
        gpio_reset_pin(I2C_SCL);
        gpio_reset_pin(I2C_SDA);

        // Enable voltage divider
        gpio_set_direction(BAT_ADC_EN_GPIO, GPIO_MODE_OUTPUT);
        gpio_set_level(BAT_ADC_EN_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(ADC_DELAY_MS));

        // Try initializing and deinitializing ADC before each sample (hoping to fix constant 4081 issue)
        adc_init();

        // Read ADC
        if(adc_oneshot_read(bat_adc_handle, BAT_ADC_CH, &adc_raw) == ESP_OK) {
            ESP_LOGI("ADC", "ADC value: %d", adc_raw);
        }
        if (adc_cali_raw_to_voltage(bat_adc_cali_handle, adc_raw, &voltage) == ESP_OK) {
            ESP_LOGI("ADC", "ADC voltage: %d", voltage);
        }

        // Release power management
        esp_pm_lock_release(pm_lock);
        if (zb_started_flag == 1) {
            esp_zb_lock_release();
        }

        // Disable voltage divider pin
        gpio_set_direction(BAT_ADC_EN_GPIO, GPIO_MODE_DISABLE);

        adc_calibration_deinit(bat_adc_cali_handle);
        adc_oneshot_del_unit(bat_adc_handle);

        zb_bat_voltage = (int16_t)voltage * 2;  // Consider the voltage divider

        // If Zigbee has been started, update attribute
        if (zb_started_flag == 1) {
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                CUSTOM_ELECTRICAL_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                CUSTOM_ELECTRICAL_CLUSTER_BATTERY_ATTR_ID, &zb_bat_voltage, false);
            esp_zb_lock_release();
        }
        
        // Set flag first time this is sampled
        if (first_sensor_sample_flag == 0) {
            first_sensor_sample_flag = 1;
        }

        vTaskDelay(pdMS_TO_TICKS(SENSOR_UPDATE_PERIOD_SEC * 1000));
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Configure HDC1080 and start multisense sampling loop
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t multisensor_init(void)
{
    /*
    gpio_reset_pin(I2C_SCL);
    gpio_reset_pin(I2C_SDA);

    //i2c_master_bus_rm_device(hdc1080_dev_handle);  // Should be redundant if deleting the bus
    //i2c_master_bus_rm_device(opt3001_dev_handle);  // Should be redundant if deleting the bus
    //i2c_del_master_bus(i2c_bus_handle);


    // Initialize I2C bus
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &i2c_bus_handle));
    
    // Reset bus to avoid errors
    //ESP_ERROR_CHECK(i2c_master_bus_reset(i2c_bus_handle));
        
    // Add HDC1080 to I2C bus driver
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &hdc1080_dev_cfg, &hdc1080_dev_handle));

    // Configure HDC1080 settings
    hdc1080_settings.device_handle = hdc1080_dev_handle;
    hdc1080_settings.configuration = HDC1080_CONFIG_TEMP_14BIT_RES_MASK | HDC1080_CONFIG_HUMIDITY_14BIT_RES_MASK | 
                                     HDC1080_CONFIG_HEATER_DISABLE_MASK | HDC1080_CONFIG_TEMP_AND_HUMIDITY_MODE_MASK;
        
    // Config HDC1080
    ESP_ERROR_CHECK(HDC1080_Write_Configuration(&hdc1080_settings));

    // Add OPT3001 to I2C bus driver
    ESP_ERROR_CHECK(i2c_master_bus_add_device(i2c_bus_handle, &opt3001_dev_cfg, &opt3001_dev_handle));

    // Configure OPT3001 settings
    opt3001_settings.device_handle = opt3001_dev_handle;
    opt3001_settings.configuration = OPT3001_CONFIG_AUTO_FULLSCALE_RANGE_MASK | OPT3001_CONFIG_CONVERSION_TIME_100MS_MASK | 
                                     OPT3001_CONFIG_CONV_MODE_SINGLESHOT_MASK | OPT3001_CONFIG_LATCH_LATCHED_MASK;
        
    // Config OPT3001
    ESP_ERROR_CHECK(OPT3001_Write_Configuration(&opt3001_settings));

    // Disable I2C bus (needed for light sleep)
    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(hdc1080_dev_handle));  // Should be redundant if deleting the bus
    //ESP_ERROR_CHECK(i2c_master_bus_rm_device(opt3001_dev_handle));  // Should be redundant if deleting the bus
    ESP_ERROR_CHECK(i2c_del_master_bus(i2c_bus_handle));
    gpio_reset_pin(I2C_SCL);
    gpio_reset_pin(I2C_SDA);
    */
    
    return (xTaskCreate(update_all_sensors, "update_all_sensors", 8192, NULL, 10, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Configure GPIO
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t gpio_init(void) { 
    gpio_config_t pir_io_conf = {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    }; 
    gpio_config_t led0_io_conf = {
        .pin_bit_mask = (1ULL << LED0_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t led1_io_conf = {
        .pin_bit_mask = (1ULL << LED1_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t ledp_io_conf = {
        .pin_bit_mask = (1ULL << LEDP_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t bat_adc_en_io_conf = {
        .pin_bit_mask = (1ULL << BAT_ADC_EN_GPIO),
        .mode = GPIO_MODE_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config_t butt0_io_conf = {
        .pin_bit_mask = (1ULL << BUTT0_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,   
    };

    ESP_RETURN_ON_ERROR(gpio_config(&pir_io_conf), TAG, "Failed to configure PIR GPIO");
    ESP_RETURN_ON_ERROR(gpio_config(&led0_io_conf), TAG, "Failed to configure LED0 GPIO");
    ESP_RETURN_ON_ERROR(gpio_config(&led1_io_conf), TAG, "Failed to configure LED1 GPIO");
    ESP_RETURN_ON_ERROR(gpio_config(&ledp_io_conf), TAG, "Failed to configure PIR LED GPIO");
    ESP_RETURN_ON_ERROR(gpio_config(&bat_adc_en_io_conf), TAG, "Failed to configure BAT ADC enable GPIO");
    ESP_RETURN_ON_ERROR(gpio_config(&butt0_io_conf), TAG, "Failed to configure BUTT0 GPIO");
    //ESP_RETURN_ON_ERROR(gpio_sleep_set_direction(PIR_GPIO, GPIO_MODE_INPUT), TAG, "Failed to set sleep direction of PIR IO");
    //ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT), TAG, "Failed to install GPIO ISR service");
    // Do this to avoid race conditions on boot from PIR GPIO
    uint8_t pir_level = gpio_get_level(PIR_GPIO);
    PIR_interrupt_type = pir_level? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(PIR_GPIO, PIR_interrupt_type), TAG, "Failed to set PIR interrupt type");
    ESP_RETURN_ON_ERROR(gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT), TAG, "Failed to install ISR service");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PIR_GPIO, gpio_isr_handler, (void*) PIR_GPIO), TAG, "Failed to initialize PIR pin interrupt");
    //ESP_ERROR_CHECK(gpio_wakeup_enable(PIR_GPIO, PIR_interrupt_type));    

    //ESP_RETURN_ON_ERROR(gpio_sleep_set_direction(BUTT0_GPIO, GPIO_MODE_INPUT), TAG, "Failed to set sleep direction of BUTT0 IO");
    // Do this to avoid race conditions on boot from BUTT0 GPIO
    BUTT0_interrupt_type = gpio_get_level(BUTT0_GPIO)? GPIO_INTR_LOW_LEVEL : GPIO_INTR_HIGH_LEVEL;
    ESP_RETURN_ON_ERROR(gpio_set_intr_type(BUTT0_GPIO, BUTT0_interrupt_type), TAG, "Failed to set BUTT0 interrupt type");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(BUTT0_GPIO, gpio_isr_handler, (void*) BUTT0_GPIO), TAG, "Failed to initialize BUTT0 pin interrupt");
    //ESP_ERROR_CHECK(gpio_wakeup_enable(BUTT0_GPIO, BUTT0_interrupt_type));    

    //ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    // If looking for LOW wakeup, add BUTT0 to wakeup, if HIGH, don't include BUTT0
    ESP_RETURN_ON_ERROR(esp_sleep_enable_ext1_wakeup((1ULL << PIR_GPIO) | ((pir_level? 1ULL : 0) << BUTT0_GPIO), pir_level? ESP_EXT1_WAKEUP_ANY_LOW : ESP_EXT1_WAKEUP_ANY_HIGH), TAG, "Failed to enable EXT1 wakeup");
    gpio_isr_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    esp_err_t return_err = (xTaskCreate(button_handler, "button_handler", 4096, NULL, 10, &button_handler_handle) == pdTRUE) ? ESP_OK : ESP_FAIL;
    vTaskSuspend(button_handler_handle);  // Suspend the task initially
    return_err = (xTaskCreate(pir_sensor_update, "pir_update", 4096, NULL, 10, &pir_sensor_update_handle) == pdTRUE) ? ESP_OK : ESP_FAIL;
    vTaskSuspend(pir_sensor_update_handle);  // Suspend the task initially
    return return_err;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Initialize drivers
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t init_all_drivers(void)
{
    //ESP_RETURN_ON_ERROR(adc_init(), TAG, "Failed to initialize ADC");
    ESP_RETURN_ON_ERROR(gpio_init(), TAG, "Failed to set up GPIO and interrupt");
    ESP_RETURN_ON_ERROR(multisensor_init(), TAG, "Failed to initialize DHT driver");
    return ESP_OK;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Zigbee signal handler
///////////////////////////////////////////////////////////////////////////////////////////
void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p     = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Initialize Zigbee stack");
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
                esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            } else {
                ESP_LOGI(TAG, "Device rebooted");
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            // This is caused when the parent node can't be found. Keep resetting until parent node is found: https://github.com/espressif/esp-zigbee-sdk/issues/169#issuecomment-1849540638
            deinit_and_restart();
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        zb_steering_alarm_flag = 0;
        if (err_status == ESP_OK) {
            zb_network_status = 1;
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    // CUSTOM CODE:
    // Handle leaving the network signal
    case ESP_ZB_ZDO_SIGNAL_LEAVE:
        esp_zb_zdo_signal_leave_params_t *leave_params = (esp_zb_zdo_signal_leave_params_t *)esp_zb_app_signal_get_params(p_sg_p);
        if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_RESET) {
            ESP_LOGI(TAG, "Leave signal received (RESET)");
        }
        else if (leave_params->leave_type == ESP_ZB_NWK_LEAVE_TYPE_REJOIN) {
            ESP_LOGI(TAG, "Leave signal received (REJOIN)");
        }
        ESP_LOGI(TAG, "Restarting network steering");

        zb_network_status = 0;

        // Restart commissioning to rejoin a network
        esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);

        // Set alarm to handle if network isn't rejoined
        if (zb_unavailable_alarm_flag == 0) {
            zb_unavailable_alarm_flag = 1;
            ESP_LOGI(TAG, "Zigbee unavailable alarm set");
            esp_zb_scheduler_alarm((esp_zb_callback_t)handle_zigbee_unavilable, 0, 60*1000);
        }
        //deinit_and_restart();
        break;
    // Handle unavailable signal
    case ESP_ZB_ZDO_DEVICE_UNAVAILABLE:
        zb_network_status = 0;
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type, esp_err_to_name(err_status));
        // Set alarm to handle zigbee unavailable error if it hasn't been set yet
        if (zb_unavailable_alarm_flag == 0) {
            zb_unavailable_alarm_flag = 1;
            ESP_LOGI(TAG, "Zigbee unavailable alarm set");
            esp_zb_scheduler_alarm((esp_zb_callback_t)handle_zigbee_unavilable, 0, 60*1000);
        }
        break;
    // Handle being able to sleep
    case ESP_ZB_COMMON_SIGNAL_CAN_SLEEP:
        ESP_LOGI(TAG, "Zigbee can sleep");
        esp_zb_sleep_now();
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }

    // Handle network LED
    if (leds_enabled == 1) {
        if (esp_zb_bdb_dev_joined()) {
            gpio_set_direction(NETWORK_LED_GPIO, GPIO_MODE_OUTPUT);
            gpio_hold_dis(NETWORK_LED_GPIO);
            gpio_set_level(NETWORK_LED_GPIO, 0);
            gpio_hold_en(NETWORK_LED_GPIO);
        }
        else {
            gpio_hold_dis(NETWORK_LED_GPIO);
            gpio_set_direction(NETWORK_LED_GPIO, GPIO_MODE_DISABLE);
        }
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Define custom Zigbee cluster
///////////////////////////////////////////////////////////////////////////////////////////
esp_zb_cluster_list_t *multisensor_cluster_create(esp_zb_multisensor_cfg_t *multisensor)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&(multisensor->basic_cfg));

    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&(multisensor->identify_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(cluster_list, esp_zb_temperature_meas_cluster_create(&(multisensor->temp_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_humidity_meas_cluster(cluster_list, esp_zb_humidity_meas_cluster_create(&(multisensor->humidity_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_illuminance_meas_cluster(cluster_list, esp_zb_illuminance_meas_cluster_create(&(multisensor->light_meas_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_occupancy_sensing_cluster(cluster_list, esp_zb_occupancy_sensing_cluster_create(&(multisensor->pir_sens_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster_create(&(multisensor->pir_led_on_off_cfg)), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    
    // Create custom attributes
    esp_zb_attribute_list_t *custom_electrical_attr_list = esp_zb_zcl_attr_list_create(CUSTOM_ELECTRICAL_CLUSTER_ID);
    ESP_ERROR_CHECK(esp_zb_custom_cluster_add_custom_attr(custom_electrical_attr_list, CUSTOM_ELECTRICAL_CLUSTER_BATTERY_ATTR_ID, ESP_ZB_ZCL_ATTR_TYPE_S16, ESP_ZB_ZCL_ATTR_ACCESS_READ_ONLY |
                                                                                                                           ESP_ZB_ZCL_ATTR_ACCESS_REPORTING, &zb_bat_voltage));

    // Create custom clusters
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_custom_cluster(cluster_list, custom_electrical_attr_list, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    return cluster_list;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Create Zigbee device endpoint
///////////////////////////////////////////////////////////////////////////////////////////
esp_zb_ep_list_t *multisensor_ep_create(uint8_t endpoint_id, esp_zb_multisensor_cfg_t *multisensor)
{
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = endpoint_id,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_CUSTOM_ATTR_DEVICE_ID,
        .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, multisensor_cluster_create(multisensor), endpoint_config);
    return ep_list;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Configure some sleep stuff
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t esp_zb_power_save_init(void)
{
    esp_err_t rc = ESP_OK;
#ifdef CONFIG_PM_ENABLE
    int cur_cpu_freq_mhz = CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ;
    esp_pm_config_t pm_config = {
        .max_freq_mhz = cur_cpu_freq_mhz,
        .min_freq_mhz = cur_cpu_freq_mhz,
#if CONFIG_FREERTOS_USE_TICKLESS_IDLE
        .light_sleep_enable = true
#endif
    };
    rc = esp_pm_configure(&pm_config);
#endif
    return rc;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Toggle identify LED
///////////////////////////////////////////////////////////////////////////////////////////
static void identify_led_toggler(uint8_t led_state) {
    gpio_hold_dis(IDENTIFY_LED_GPIO);
    gpio_set_level(IDENTIFY_LED_GPIO, led_state);
    gpio_hold_en(IDENTIFY_LED_GPIO);

    if (zb_currently_identifying == 1) {
        esp_zb_scheduler_alarm((esp_zb_callback_t)identify_led_toggler, !led_state, 100);
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Handle identify cluster
///////////////////////////////////////////////////////////////////////////////////////////
static void identify_cluster_handler(uint8_t last_alarm_call) {

    // Update identify time cluster
    zb_identify_time--;
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
        ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID, &zb_identify_time, false);
    esp_zb_lock_release();

    // For the last alarm call to this function, disable LED pin
    if (last_alarm_call) {
        gpio_hold_dis(IDENTIFY_LED_GPIO);
        gpio_set_direction(IDENTIFY_LED_GPIO, GPIO_MODE_DISABLE);
        zb_currently_identifying = 0;
        esp_zb_scheduler_alarm_cancel((esp_zb_callback_t)identify_led_toggler, 0);  // Cancel alarm just in case
    }
    else if (zb_identify_time > 0) {
        // Schedule next alarm normally
        esp_zb_scheduler_alarm((esp_zb_callback_t)identify_cluster_handler, (zb_identify_time==1? 1 : 0), 1000);
    }
    
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Handle settings attributes (light on/off)
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)", message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (message->info.dst_endpoint == HA_ESP_SENSOR_ENDPOINT) {
        // Handle on off cluster
        if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_ON_OFF) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL) {
                light_state = message->attribute.data.value ? *(bool *)message->attribute.data.value : light_state;
                ESP_LOGI(TAG, "PIR LED set to %s", light_state ? "on" : "off");
                if (light_state == 0) {
                    gpio_hold_dis(LEDP_GPIO);
                    gpio_set_direction(LEDP_GPIO, GPIO_MODE_DISABLE);
                }
                else {
                    gpio_set_direction(LEDP_GPIO, GPIO_MODE_OUTPUT);
                    gpio_set_level(LEDP_GPIO, 0);
                    gpio_hold_en(LEDP_GPIO);
                }
            }
        }
        // Handle identify cluster
        else if (message->info.cluster == ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY) {
            if (message->attribute.id == ESP_ZB_ZCL_ATTR_IDENTIFY_IDENTIFY_TIME_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                // Make sure not currently in identify mode
                if (zb_currently_identifying == 0) {
                   zb_identify_time = *(uint16_t*)(message->attribute.data.value);
                    ESP_LOGI(TAG, "Identify time set to %d seconds", zb_identify_time);

                    // Turn on identify LED initially
                    gpio_set_direction(IDENTIFY_LED_GPIO, GPIO_MODE_OUTPUT);
                    gpio_hold_dis(IDENTIFY_LED_GPIO);
                    gpio_set_level(IDENTIFY_LED_GPIO, 0);
                    gpio_hold_en(IDENTIFY_LED_GPIO);

                    zb_currently_identifying = 1;
                    esp_zb_scheduler_alarm((esp_zb_callback_t)identify_led_toggler, 1, 100);
                    esp_zb_scheduler_alarm((esp_zb_callback_t)identify_cluster_handler, (zb_identify_time==1? 1 : 0), 1000);
                }
            }
        }
    }
    return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Handle attribute set command
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
        ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *)message);
        break;
    default:
        ESP_LOGI(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Zigbee task (called once, basically the setup)
///////////////////////////////////////////////////////////////////////////////////////////
void esp_zb_task(void *pvParameters)
{
    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_sleep_enable(true);
    //esp_zb_sleep_set_threshold(ZIGBEE_SLEEP_TIME_MS); // Unused
    esp_zb_init(&zb_nwk_cfg);

    // Create customized temperature sensor endpoint
    esp_zb_multisensor_cfg_t sensor_cfg;

    // Set some default values
    sensor_cfg.pir_sens_cfg.occupancy = 0;                                  // Unoccupied
    sensor_cfg.pir_sens_cfg.sensor_type = 0;                                // PIR
    sensor_cfg.basic_cfg.power_source = 0x03;                               // For battery
    sensor_cfg.temp_meas_cfg.measured_value = zb_temperature;
    sensor_cfg.humidity_meas_cfg.measured_value = zb_humidity;
    sensor_cfg.light_meas_cfg.measured_value = zb_illuminance;
    sensor_cfg.electrical_meas_cfg.measured_battery_value = zb_bat_voltage;
    sensor_cfg.pir_led_on_off_cfg.on_off = 0;

    esp_zb_ep_list_t *esp_zb_sensor_ep = multisensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &sensor_cfg);

    // Register the device
    esp_zb_device_register(esp_zb_sensor_ep);

    // Register action handler function
    esp_zb_core_action_handler_register(zb_action_handler);

    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));

    zb_started_flag = 1;    // Set zigbee started flag

    esp_zb_main_loop_iteration();
}
///////////////////////////////////////////////////////////////////////////////////////////


///////////////////////////////////////////////////////////////////////////////////////////
// App_main, really only called once. Init drivers here
///////////////////////////////////////////////////////////////////////////////////////////
void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_power_save_init());
    
    //ESP_ERROR_CHECK(esp_sleep_pd_config(ESP_PD_DOMAIN_TOP, ESP_PD_OPTION_ON));
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

    // Report software version
    ESP_LOGI(TAG, "ESP_STICKBEE SOFTWARE VERSION: %d.%d", MAJOR_SOFTWARE_VER, MINOR_SOFTWARE_VER);

    // Initialize all drivers/peripherals
    ESP_LOGI(TAG, "All driver initialization %s", init_all_drivers() ? "failed" : "successful");

    // Wait for first sample to occur with a timeout
    const TickType_t timeout_ticks = pdMS_TO_TICKS(1000);  // Timeout of 10 seconds
    TickType_t start_ticks = xTaskGetTickCount();
    while (first_sensor_sample_flag == 0) {
        if (xTaskGetTickCount() - start_ticks >= timeout_ticks) {
            ESP_LOGE(TAG, "First sensor sample reading timed out");
            break;
        }
    }

    // Start Zigbee stack task
    xTaskCreate(esp_zb_task, "Zigbee_main", 4096, NULL, 5, NULL);
}
///////////////////////////////////////////////////////////////////////////////////////////
