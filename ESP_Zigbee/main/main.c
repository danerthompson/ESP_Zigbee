#include "switch_driver.h"
#include "zigbee_defs.h"
#include "periph_funcs.h"
#include "hdc1080.h"

#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

/*
#include "ha/esp_zigbee_ha_standard.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

//#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "hdc1080.h"
*/

#define PIR_GPIO GPIO_NUM_7
#define SENSOR_UPDATE_PERIOD_SEC 300

adc_oneshot_unit_handle_t bat_adc_handle;
adc_oneshot_unit_init_cfg_t bat_adc_init_config = {
    .unit_id = BAT_ADC,
};
adc_oneshot_chan_cfg_t bat_adc_config = {
    .bitwidth = ADC_BITWIDTH_DEFAULT,
    .atten = BAT_ADC_ATTEN,
};
adc_cali_handle_t bat_adc_cali_handle = NULL;

uint8_t zb_started_flag = 0;            // Flag is set after Zigbee stack is initialized
uint8_t first_sensor_sample_flag = 0;   // Flag is set after first sensor sample occurs (stops Zigbee from reporting 0s on restart)
int zb_network_status = 0;              // 0 for unjoined, 1 for joined, -1 for left
uint8_t zb_unavailable_alarm_flag = 0;  // 0 if alarm has not been set, 1 if alarm is set

int16_t zb_bat_voltage, zb_temperature, zb_humidity;

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile sensor (End Device) source code.
#endif

static const char *TAG = "ESP_STICKBEE";

///////////////////////////////////////////////////////////////////////////////////////////
// Set up PIR sensor interrupt
static QueueHandle_t gpio_isr_evt_queue = NULL;
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_isr_evt_queue, &gpio_num, NULL);
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Update PIR sensor attribute when interrupt is called
///////////////////////////////////////////////////////////////////////////////////////////
static void pir_sensor_update(void *arg) {
    uint32_t io_num;
    uint8_t zb_occupancy;
    while (1) {
        if (xQueueReceive(gpio_isr_evt_queue, &io_num, portMAX_DELAY)) {    // True if item received from queue
            zb_occupancy = gpio_get_level(io_num);

            // If Zigbee has started, update the attribute
            if (zb_started_flag == 1) {
                esp_zb_lock_acquire(portMAX_DELAY);
                esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                    ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                    ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID, &zb_occupancy, false);
                esp_zb_lock_release();
            }
        }
        vTaskDelay(pdMS_TO_TICKS(250));
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Handle push buttons
///////////////////////////////////////////////////////////////////////////////////////////
static switch_func_pair_t button_func_pair[] = {
    {GPIO_INPUT_IO_TOGGLE_SWITCH, SWITCH_ONOFF_TOGGLE_CONTROL}
};
static void esp_app_buttons_handler(switch_func_pair_t *button_func_pair)
{
    if (button_func_pair->func == SWITCH_ONOFF_TOGGLE_CONTROL) {
        // Send report attributes command
        esp_zb_zcl_report_attr_cmd_t report_attr_cmd;
        report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
        report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID;
        report_attr_cmd.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
        report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT;
        report_attr_cmd.zcl_basic_cmd.src_endpoint = HA_ESP_SENSOR_ENDPOINT;

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_report_attr_cmd_req(&report_attr_cmd);
        esp_zb_lock_release();
        ESP_EARLY_LOGI(TAG, "Send 'report attributes' command");
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Function to deinit peripherals (they aren't reset on software restart) and reset ESP
///////////////////////////////////////////////////////////////////////////////////////////
static void deinit_and_restart() {
    i2c_driver_delete(I2C_PORT);
    adc_calibration_deinit(bat_adc_cali_handle);
    adc_oneshot_del_unit(bat_adc_handle);
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
// Callback with new HDC1080 data
///////////////////////////////////////////////////////////////////////////////////////////
void hdc1080_readings_callback(hdc1080_sensor_readings_t sens_readings) {

    // Set flag first time this is sampled
    if (first_sensor_sample_flag == 0) {
        first_sensor_sample_flag = 1;
    }
    
    zb_temperature = (int16_t) ((sens_readings.temperature+0.005)*100.0);   // Add 0.005 to do rounding instead of truncating
    zb_humidity = (int16_t) ((sens_readings.humidity+0.005)*100.0);         // Add 0.005 to do rounding instead of truncating
    float temp_in_f = CEL2FAH(sens_readings.temperature);
    ESP_LOGI("SENSOR_DATA", "TEMPERATURE: %.2f°F", temp_in_f);
    ESP_LOGI("SENSOR_DATA", "HUMIDITY: %.2f%%", sens_readings.humidity);

    // If Zigbee has started, update attributes
    if (zb_started_flag == 1) {
        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID, &zb_temperature, false);
        esp_zb_lock_release();

        esp_zb_lock_acquire(portMAX_DELAY);
        esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_REL_HUMIDITY_MEASUREMENT, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_REL_HUMIDITY_MEASUREMENT_VALUE_ID, &zb_humidity, false);
        esp_zb_lock_release();
    }
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Function to update all sensors (HDC1080 and ADC)
///////////////////////////////////////////////////////////////////////////////////////////
static void update_all_sensors(void *arg) {
    int adc_raw, voltage;

    while (1) {
        
        // Request readings from HDC1080
        if(hdc1080_request_readings() == ESP_OK){
            //ESP_LOGI("MAIN", "READINGS WERE REQUESTED");
        }

        // Read ADC
        if(adc_oneshot_read(bat_adc_handle, BAT_ADC_CH, &adc_raw) == ESP_OK) {
            ESP_LOGI("ADC", "ADC value: %d", adc_raw);
        }

        if (adc_cali_raw_to_voltage(bat_adc_cali_handle, adc_raw, &voltage) == ESP_OK) {
            ESP_LOGI("ADC", "ADC voltage: %d", voltage);
        }

        zb_bat_voltage = (int16_t)voltage;

        // If Zigbee has been started, update attribute
        if (zb_started_flag == 1) {
            esp_zb_lock_acquire(portMAX_DELAY);
            esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
                CUSTOM_ELECTRICAL_CLUSTER_ID, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                CUSTOM_ELECTRICAL_CLUSTER_BATTERY_ATTR_ID, &zb_bat_voltage, false);
            esp_zb_lock_release();
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
    if(i2c_init()) {
        hdc1080_settings_t hdc_settings = {
        .i2c_address = HDC1080_I2C_ADDRESS,
        .i2c_port_number = I2C_PORT,
        .timeout_length = I2C_READ_TIMEOUT_PERIOD,
        .callback = hdc1080_readings_callback
        };
        
        // SETUP YOUR HDC REGISTER CONFIGURATION
        hdc1080_config_t hdc_config = {
        .humidity_measurement_resolution = HDC1080_HUMIDITY_RESOLUTION_14BIT,
        .temperature_measurement_resolution = HDC1080_TEMPERATURE_RESOLUTION_14BIT,
        .mode_of_acquisition = HDC1080_ACQUISITION_HUMIDITY_AND_TEMPERATURE,
        .heater = HDC1080_HEATER_DISABLED
        };

        // SETUP AND CONFIGURE THE SENSOR AND ABSTRACTION
        if(hdc1080_configure(&hdc_settings, hdc_config) == ESP_OK){
            ESP_LOGI("SENSOR_FUNCS", "HDC1080 CONFIGURATION SUCCESSFUL");
            // DO A REQUEST FOR THE SENSOR READINGS 
            // THIS WILL CALLBACK TO void temperature_readings_callback(hdc1080_sensor_readings_t sens_readings)
            // AS SET IN THE hdc_settings
            //if(hdc1080_request_readings() == ESP_OK){
            //    ESP_LOGI("SENSOR_FUNCS", "READINGS WERE REQUESTED");
            //}
        }
    }
    return (xTaskCreate(update_all_sensors, "update_all_sensors", 4096, NULL, 10, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
}
///////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////
// Configure GPIO
///////////////////////////////////////////////////////////////////////////////////////////
static esp_err_t gpio_init(void) {  
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIR_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_ANYEDGE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "Failed to configure GPIO");
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(PIR_GPIO, gpio_isr_handler, (void*) PIR_GPIO), TAG, "Failed to initialize pin interrupt");
    gpio_isr_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    return (xTaskCreate(pir_sensor_update, "pir_update", 4096, NULL, 10, NULL) == pdTRUE) ? ESP_OK : ESP_FAIL;
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

static esp_err_t init_all_drivers(void)
{
    ESP_RETURN_ON_FALSE(switch_driver_init(button_func_pair, PAIR_SIZE(button_func_pair), esp_app_buttons_handler), ESP_FAIL, TAG, "Failed to initialize switch driver");
    ESP_RETURN_ON_ERROR(adc_init(), TAG, "Failed to initialize ADC");
    ESP_RETURN_ON_ERROR(gpio_init(), TAG, "Failed to set up GPIO and interrupt");
    ESP_RETURN_ON_ERROR(multisensor_init(), TAG, "Failed to initialize DHT driver");
    return ESP_OK;
}

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
        }
        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
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
        ESP_LOGI(TAG, "Restartig network steering");
        //esp_zb_factory_reset();
        deinit_and_restart();
        //bdb_start_top_level_commissioning_cb(ESP_ZB_BDB_MODE_NETWORK_STEERING);
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
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
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
// Zigbee task (called once, basically the setup)
///////////////////////////////////////////////////////////////////////////////////////////
void esp_zb_task(void *pvParameters)
{
    // Initialize Zigbee stack
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    // Create customized temperature sensor endpoint
    esp_zb_multisensor_cfg_t sensor_cfg;

    // Set some default values
    sensor_cfg.pir_sens_cfg.occupancy = 0;        // Unoccupied
    sensor_cfg.pir_sens_cfg.sensor_type = 0;      // PIR
    sensor_cfg.basic_cfg.power_source = 0x03;     // For battery
    sensor_cfg.temp_meas_cfg.measured_value = zb_temperature;
    sensor_cfg.humidity_meas_cfg.measured_value = zb_humidity;
    sensor_cfg.electrical_meas_cfg.measured_battery_value = zb_bat_voltage;

    esp_zb_ep_list_t *esp_zb_sensor_ep = multisensor_ep_create(HA_ESP_SENSOR_ENDPOINT, &sensor_cfg);

    // Register the device
    esp_zb_device_register(esp_zb_sensor_ep);

    /*
    // Update power source in basic cluster
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(HA_ESP_SENSOR_ENDPOINT,
            ESP_ZB_ZCL_CLUSTER_ID_BASIC, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            ESP_ZB_ZCL_ATTR_BASIC_POWER_SOURCE_ID, &zcl_basic_power_src, false);
    esp_zb_lock_release();
    */

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
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));

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
