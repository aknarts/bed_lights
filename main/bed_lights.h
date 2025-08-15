//
// Created by stastny on 25.9.24.
//

#ifndef BED_LIGHTS_H
#define BED_LIGHTS_H

#include "esp_zigbee_core.h"
#include "light_driver.h"

/* Zigbee configuration */
#define MAX_CHILDREN                      10                                    /* the max amount of connected devices */
#define INSTALLCODE_POLICY_ENABLE       false   /* enable the install code policy for security */
#define HA_ESP_SENSOR_ENDPOINT          1
#define ESP_ZB_PRIMARY_CHANNEL_MASK     ESP_ZB_TRANSCEIVER_ALL_CHANNELS_MASK    /* Zigbee primary channel mask use in the example */

/* Attribute values in ZCL string format
 * The string should be started with the length of its own.
 */
#define MANUFACTURER_NAME               "\x06""Acheta"
#define MODEL_IDENTIFIER                "\x0C""Bed.Lights"

#define BASE_LIGHT_ENDPOINT              1
#define STAIRS_LED_COUNT                12   // number of individual stair lights (channels)
#define BED_STRIP_COUNT                 2    // number of bed side strips (channels)
#define BED_STRIP_LED_LENGTH            60    // assumed length per bed side strip (adjust)
// NOTE: ESP32-C6 RMT channel count may limit how many strips can be driven simultaneously.
#define TOTAL_LIGHT_CHANNELS            (STAIRS_LED_COUNT + BED_STRIP_COUNT)

#define BOARD_TEMP_ENDPOINT             (BASE_LIGHT_ENDPOINT + TOTAL_LIGHT_CHANNELS)
#define BOARD_TEMP_UPDATE_INTERVAL_S    5    // seconds between measurements
#define BOARD_TEMP_MIN_C               -10
#define BOARD_TEMP_MAX_C                85

#define ESP_ZB_ZR_CONFIG()                                                              \
    {                                                                                   \
        .esp_zb_role = ESP_ZB_DEVICE_TYPE_ROUTER,                                       \
        .install_code_policy = INSTALLCODE_POLICY_ENABLE,                               \
        .nwk_cfg.zczr_cfg = {                                                           \
            .max_children = MAX_CHILDREN,                                               \
        },                                                                              \
    }

#define ESP_ZB_DEFAULT_RADIO_CONFIG()                           \
    {                                                           \
        .radio_mode = ZB_RADIO_MODE_NATIVE,                     \
    }

#define ESP_ZB_DEFAULT_HOST_CONFIG()                            \
    {                                                           \
        .host_connection_mode = ZB_HOST_CONNECTION_MODE_NONE,   \
    }

#endif //BED_LIGHTS_H
