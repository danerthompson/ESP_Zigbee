#pragma once

#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zigbee_core.h"

/* Zigbee configuration */
#define INSTALLCODE_POLICY_ENABLE       false                                   // enable the install code policy for security
#define ED_AGING_TIMEOUT                ESP_ZB_ED_AGING_TIMEOUT_64MIN
#define ED_KEEP_ALIVE                   7500                                    // milliseconds
#define HA_ESP_SENSOR_ENDPOINT          10                                      // esp sensor device endpoint
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    // Zigbee primary channel mask use in the example


/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x16""Stickman Solutions LLC"
#define MODEL_IDENTIFIER                "\x0C""ESP_Stickbee"

#define ESP_ZB_ZED_CONFIG()                                         \
    {                                                               \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ED,                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,           \
        .nwk_cfg.zed_cfg = {                                        \
            .ed_timeout = ED_AGING_TIMEOUT,                         \
            .keep_alive = ED_KEEP_ALIVE,                            \
        },                                                          \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }  
                                                         
// Custom attribute info
#define CUSTOM_ELECTRICAL_CLUSTER_ID 0xFC69
#define CUSTOM_ELECTRICAL_CLUSTER_BATTERY_ATTR_ID 0x0020

typedef struct esp_zb_custom_electrical_meas_cluster_cfg_s {
    int16_t measured_battery_value;                 
} esp_zb_custom_electrical_meas_cluster_cfg_t;

typedef struct esp_zb_multisensor_cfg_s {
    esp_zb_basic_cluster_cfg_t basic_cfg;              
    esp_zb_identify_cluster_cfg_t identify_cfg;         
    esp_zb_temperature_meas_cluster_cfg_t temp_meas_cfg; 
    esp_zb_humidity_meas_cluster_cfg_t humidity_meas_cfg;
    esp_zb_illuminance_meas_cluster_cfg_t light_meas_cfg;
    esp_zb_occupancy_sensing_cluster_cfg_t pir_sens_cfg;
    esp_zb_on_off_cluster_cfg_t pir_led_on_off_cfg;
    esp_zb_custom_electrical_meas_cluster_cfg_t electrical_meas_cfg;
} esp_zb_multisensor_cfg_t;

