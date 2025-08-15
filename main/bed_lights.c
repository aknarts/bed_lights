#include <sys/cdefs.h>
#include <stdio.h>
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "bed_lights.h"
#include "esp_check.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "temp_sensor_driver.h"

static const char *TAG = "ESP_ZB_LIGHT";

static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_RETURN_ON_FALSE(esp_zb_bdb_start_top_level_commissioning(mode_mask) == ESP_OK, ,
                        TAG, "Failed to start Zigbee bdb commissioning");
}

static inline bool endpoint_is_light(uint8_t ep) {
    return ep >= BASE_LIGHT_ENDPOINT && ep < BASE_LIGHT_ENDPOINT + TOTAL_LIGHT_CHANNELS;
}
static inline size_t endpoint_to_channel(uint8_t ep) { return (size_t)(ep - BASE_LIGHT_ENDPOINT); }

static int16_t zb_temperature_encode(float celsius) { return (int16_t)(celsius * 100); }

static void board_temp_update_cb(float temperature)
{
    int16_t measured_value = zb_temperature_encode(temperature);
    esp_zb_lock_acquire(portMAX_DELAY);
    esp_zb_zcl_set_attribute_val(BOARD_TEMP_ENDPOINT,
                                 ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
                                 ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                 ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
                                 &measured_value, false);
    esp_zb_lock_release();
}

static esp_err_t deferred_driver_init(void)
{
    // Light endpoints off state
    size_t chs = light_driver_channel_count();
    esp_zb_lock_acquire(portMAX_DELAY);
    for (size_t i = 0; i < chs; ++i) {
        uint8_t ep = BASE_LIGHT_ENDPOINT + i;
        bool off = false;
        uint8_t startup_off = 0;
        esp_zb_zcl_set_attribute_val(ep, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID, &off, false);
        esp_zb_zcl_set_attribute_val(ep, ESP_ZB_ZCL_CLUSTER_ID_ON_OFF, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                                     ESP_ZB_ZCL_ATTR_ON_OFF_START_UP_ON_OFF, &startup_off, false);
    }
    esp_zb_lock_release();
    // Temperature sensor init
    temperature_sensor_config_t tcfg = TEMPERATURE_SENSOR_CONFIG_DEFAULT(BOARD_TEMP_MIN_C, BOARD_TEMP_MAX_C);
    esp_err_t err = temp_sensor_driver_init(&tcfg, BOARD_TEMP_UPDATE_INTERVAL_S, board_temp_update_cb);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Temp sensor init failed: %s", esp_err_to_name(err));
    }
    return ESP_OK;
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type)
    {
        case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
            ESP_LOGI(TAG, "Initialize Zigbee stack");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
            break;
        case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
        case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
            if (err_status == ESP_OK)
            {
                ESP_LOGI(TAG, "Deferred driver initialization %s", deferred_driver_init() ? "failed" : "successful");
                ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
                if (esp_zb_bdb_is_factory_new())
                {
                    ESP_LOGI(TAG, "Start network steering");
                    esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
                } else
                {
                    ESP_LOGI(TAG, "Device rebooted");
                }
            } else
            {
                ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
            }
            break;
        case ESP_ZB_BDB_SIGNAL_STEERING:
            if (err_status == ESP_OK)
            {
                esp_zb_ieee_addr_t extended_pan_id;
                esp_zb_get_extended_pan_id(extended_pan_id);
                ESP_LOGI(TAG,
                         "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                         extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                         extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                         esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());
            } else
            {
                ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
                esp_zb_scheduler_alarm((esp_zb_callback_t) bdb_start_top_level_commissioning_cb,
                                       ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
            }
            break;
        case ESP_ZB_ZDO_SIGNAL_LEAVE:
            ESP_LOGI(TAG, "Leaving old network");
            esp_zb_nvram_erase_at_start(true);
            ESP_LOGI(TAG, "Start network steering");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);
            break;
        default:
            ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                     esp_err_to_name(err_status));
            break;
    }
}

static void esp_zb_identify(void *pvParameters)
{
    bool light_state = false;
    for (int i = 0; i < 10; ++i) // blink fewer times for identify
    {
        light_state = !light_state;
        light_driver_set_power(light_state);
        vTaskDelay(pdMS_TO_TICKS(500));
    };
    light_driver_set_power(false);
    vTaskDelay(pdMS_TO_TICKS(100));
    vTaskDelete(NULL);
}

static esp_err_t zb_attribute_handler(const esp_zb_zcl_set_attr_value_message_t *message)
{
    esp_err_t ret = ESP_OK;
    bool light_state = 0;
    uint8_t light_level = 0;
    uint16_t light_color_x = 0;
    uint16_t light_color_y = 0;
    uint16_t light_temp_mired = 0;
    uint8_t hue = 0;
    uint8_t sat = 0;
    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->info.status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG,
                        "Received message: error status(%d)",
                        message->info.status);
    ESP_LOGI(TAG, "Received message: endpoint(%d), cluster(0x%x), attribute(0x%x), data size(%d)",
             message->info.dst_endpoint, message->info.cluster,
             message->attribute.id, message->attribute.data.size);
    if (endpoint_is_light(message->info.dst_endpoint))
    {
        size_t ch = endpoint_to_channel(message->info.dst_endpoint);
        switch (message->info.cluster)
        {
            case ESP_ZB_ZCL_CLUSTER_ID_ON_OFF:
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID &&
                    message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_BOOL)
                {
                    light_state = message->attribute.data.value ? *(bool *) message->attribute.data.value : light_state;
                    ESP_LOGI(TAG, "EP %d -> channel %d set power %s", message->info.dst_endpoint, (int)ch, light_state ? "On" : "Off");
                    light_driver_set_power_ch(ch, light_state);
                } else {
                    ESP_LOGW(TAG, "On/Off cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                }
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_COLOR_CONTROL:
                // same logic but call channel-specific setters
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    light_color_x = message->attribute.data.value ? *(uint16_t *) message->attribute.data.value : light_color_x;
                    light_color_y = *(uint16_t *) esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID)->data_p;
                    ESP_LOGI(TAG, "EP %d color x -> 0x%x", message->info.dst_endpoint, light_color_x);
                    light_driver_set_color_xy_ch(ch, light_color_x, light_color_y);
                } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_Y_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    light_color_y = message->attribute.data.value ? *(uint16_t *) message->attribute.data.value : light_color_y;
                    light_color_x = *(uint16_t *) esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_X_ID)->data_p;
                    ESP_LOGI(TAG, "EP %d color y -> 0x%x", message->info.dst_endpoint, light_color_y);
                    light_driver_set_color_xy_ch(ch, light_color_x, light_color_y);
                } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U16) {
                    light_temp_mired = message->attribute.data.value ? *(uint16_t *) message->attribute.data.value : light_temp_mired;
                    ESP_LOGI(TAG, "EP %d color temp mired -> %u", message->info.dst_endpoint, light_temp_mired);
                    light_driver_set_color_temperature_mired_ch(ch, light_temp_mired);
                } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    hue = message->attribute.data.value ? *(uint8_t *) message->attribute.data.value : hue;
                    sat = *(uint8_t *) esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID)->data_p;
                    ESP_LOGI(TAG, "EP %d hue -> %u", message->info.dst_endpoint, hue);
                    light_driver_set_color_hue_sat_ch(ch, hue, sat);
                } else if (message->attribute.id == ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    sat = message->attribute.data.value ? *(uint8_t *) message->attribute.data.value : sat;
                    hue = *(uint8_t *) esp_zb_zcl_get_attribute(message->info.dst_endpoint, message->info.cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID)->data_p;
                    ESP_LOGI(TAG, "EP %d saturation -> %u", message->info.dst_endpoint, sat);
                    light_driver_set_color_hue_sat_ch(ch, hue, sat);
                } else {
                    ESP_LOGW(TAG, "Color control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                }
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL:
                if (message->attribute.id == ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_U8) {
                    light_level = message->attribute.data.value ? *(uint8_t *) message->attribute.data.value : light_level;
                    ESP_LOGI(TAG, "EP %d level -> %u", message->info.dst_endpoint, light_level);
                    light_driver_set_level_ch(ch, light_level);
                } else {
                    ESP_LOGW(TAG, "Level Control cluster data: attribute(0x%x), type(0x%x)", message->attribute.id, message->attribute.data.type);
                }
                break;
            case ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY:
                switch (message->attribute.id) {
                    case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_BLINK: light_driver_effect_start_ch(ch, LIGHT_EFFECT_BLINK); break;
                    case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_BREATHE: light_driver_effect_start_ch(ch, LIGHT_EFFECT_BREATHE); break;
                    case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_OKAY: light_driver_effect_start_ch(ch, LIGHT_EFFECT_ICU); break;
                    case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_CHANNEL_CHANGE: light_driver_effect_start_ch(ch, LIGHT_EFFECT_RANDOM_COLOR); break;
                    case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_FINISH_EFFECT:
                    case ESP_ZB_ZCL_IDENTIFY_EFFECT_ID_STOP: light_driver_effect_stop_ch(ch); break;
                    default: ESP_LOGI(TAG, "Identify effect not supported attr:0x%x", message->attribute.id); break;
                }
                break;
            default:
                ESP_LOGI(TAG, "EP %d cluster 0x%x attr 0x%x", message->info.dst_endpoint, message->info.cluster, message->attribute.id);
        }
    }
    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id)
    {
        case ESP_ZB_CORE_SET_ATTR_VALUE_CB_ID:
            ESP_LOGI(TAG, "Set attribute value callback");
            ret = zb_attribute_handler((esp_zb_zcl_set_attr_value_message_t *) message);
            break;
        case ESP_ZB_CORE_IDENTIFY_EFFECT_CB_ID:
            ESP_LOGI(TAG, "Identify effect callback");
            break;
        default:
            ESP_LOGI(TAG, "Zigbee action(0x%x) callback", callback_id);
            break;
    }
    return ret;
}

static esp_zb_cluster_list_t *
custom_light_clusters_create(esp_zb_color_dimmable_light_cfg_t *light)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();

    esp_zb_attribute_list_t *basic_cluster = esp_zb_basic_cluster_create(&light->basic_cfg);
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID,
                                                  MANUFACTURER_NAME));
    ESP_ERROR_CHECK(esp_zb_basic_cluster_add_attr(basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID,
                                                  MODEL_IDENTIFIER));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_basic_cluster(cluster_list, basic_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_identify_cluster_create(&light->identify_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_identify_cluster(cluster_list, esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_IDENTIFY), ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_on_off_cluster(cluster_list, esp_zb_on_off_cluster_create(&light->on_off_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    esp_zb_attribute_list_t *color_cluster = esp_zb_color_control_cluster_create(&light->color_cfg);
    // Add extended color attributes
    static uint16_t color_temp = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMPERATURE_DEF_VALUE;
    static uint16_t color_temp_min = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_DEFAULT_VALUE;
    static uint16_t color_temp_max = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_DEFAULT_VALUE;
    static uint8_t current_hue = 0x00;
    static uint8_t current_sat = 0x00;
    static uint16_t enhanced_hue = 0x0000;
    esp_zb_color_control_cluster_add_attr(color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMPERATURE_ID, &color_temp);
    esp_zb_color_control_cluster_add_attr(color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MIN_MIREDS_ID, &color_temp_min);
    esp_zb_color_control_cluster_add_attr(color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_COLOR_TEMP_PHYSICAL_MAX_MIREDS_ID, &color_temp_max);
    esp_zb_color_control_cluster_add_attr(color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_HUE_ID, &current_hue);
    esp_zb_color_control_cluster_add_attr(color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_CURRENT_SATURATION_ID, &current_sat);
    esp_zb_color_control_cluster_add_attr(color_cluster, ESP_ZB_ZCL_ATTR_COLOR_CONTROL_ENHANCED_CURRENT_HUE_ID, &enhanced_hue);
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_color_control_cluster(cluster_list, color_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));

    ESP_ERROR_CHECK(esp_zb_cluster_list_add_scenes_cluster(cluster_list, esp_zb_scenes_cluster_create(&light->scenes_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_level_cluster(cluster_list, esp_zb_level_cluster_create(&light->level_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_groups_cluster(cluster_list, esp_zb_groups_cluster_create(&light->groups_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static esp_zb_ep_list_t *
custom_light_ep_create(esp_zb_color_dimmable_light_cfg_t *light)
{
    // Deprecated single endpoint creator retained for reference; not used in multi-endpoint mode
    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
            .endpoint = BASE_LIGHT_ENDPOINT,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
            .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_light_clusters_create(light), endpoint_config);
    return ep_list;
}

static esp_zb_cluster_list_t *
custom_temp_clusters_create(void)
{
    esp_zb_cluster_list_t *cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_temperature_meas_cluster_cfg_t temp_cfg = {
            .measured_value = zb_temperature_encode(25.0f),
            .min_value = zb_temperature_encode(BOARD_TEMP_MIN_C),
            .max_value = zb_temperature_encode(BOARD_TEMP_MAX_C),
    };
    ESP_ERROR_CHECK(esp_zb_cluster_list_add_temperature_meas_cluster(
            cluster_list, esp_zb_temperature_meas_cluster_create(&temp_cfg), ESP_ZB_ZCL_CLUSTER_SERVER_ROLE));
    return cluster_list;
}

static void esp_zb_task(void *pvParameters)
{
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZR_CONFIG();
    esp_zb_init(&zb_nwk_cfg);
    esp_zb_color_dimmable_light_cfg_t light_cfg = {
            .basic_cfg = { .zcl_version = ESP_ZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE, .power_source = ESP_ZB_ZCL_BASIC_POWER_SOURCE_DEFAULT_VALUE, },
            .on_off_cfg = { .on_off = false, },
            .color_cfg = { .current_x = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_X_DEF_VALUE, .current_y = ESP_ZB_ZCL_COLOR_CONTROL_CURRENT_Y_DEF_VALUE, .color_mode = ESP_ZB_ZCL_COLOR_CONTROL_COLOR_MODE_DEFAULT_VALUE, .options = ESP_ZB_ZCL_COLOR_CONTROL_OPTIONS_DEFAULT_VALUE, .enhanced_color_mode = ESP_ZB_ZCL_COLOR_CONTROL_ENHANCED_COLOR_MODE_DEFAULT_VALUE, .color_capabilities = 0x001F, },
            .level_cfg = { .current_level = ESP_ZB_ZCL_LEVEL_CONTROL_CURRENT_LEVEL_DEFAULT_VALUE, },
            .scenes_cfg = { .scenes_count = ESP_ZB_ZCL_SCENES_SCENE_COUNT_DEFAULT_VALUE, .current_scene = ESP_ZB_ZCL_SCENES_CURRENT_SCENE_DEFAULT_VALUE, .current_group = ESP_ZB_ZCL_SCENES_CURRENT_GROUP_DEFAULT_VALUE, .scene_valid = ESP_ZB_ZCL_SCENES_SCENE_VALID_DEFAULT_VALUE, .name_support = ESP_ZB_ZCL_SCENES_NAME_SUPPORT_DEFAULT_VALUE, },
            .groups_cfg = { .groups_name_support_id = ESP_ZB_ZCL_GROUPS_NAME_SUPPORT_DEFAULT_VALUE, },
            .identify_cfg = { .identify_time = ESP_ZB_ZCL_IDENTIFY_IDENTIFY_TIME_DEFAULT_VALUE, }, };

    esp_zb_ep_list_t *ep_list = esp_zb_ep_list_create();
    for (size_t ch = 0; ch < TOTAL_LIGHT_CHANNELS; ++ch) {
        esp_zb_endpoint_config_t endpoint_config = {
                .endpoint = (uint8_t)(BASE_LIGHT_ENDPOINT + ch),
                .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                .app_device_id = ESP_ZB_HA_COLOR_DIMMABLE_LIGHT_DEVICE_ID,
                .app_device_version = 0
        };
        esp_zb_cluster_list_t *clusters = custom_light_clusters_create(&light_cfg);
        esp_zb_ep_list_add_ep(ep_list, clusters, endpoint_config);
    }
    // Add board temperature endpoint
    esp_zb_endpoint_config_t temp_endpoint_cfg = {
            .endpoint = BOARD_TEMP_ENDPOINT,
            .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .app_device_id = ESP_ZB_HA_TEMPERATURE_SENSOR_DEVICE_ID,
            .app_device_version = 0
    };
    esp_zb_ep_list_add_ep(ep_list, custom_temp_clusters_create(), temp_endpoint_cfg);
    esp_zb_device_register(ep_list);

    // Configure reporting for each endpoint
    for (size_t ch = 0; ch < TOTAL_LIGHT_CHANNELS; ++ch) {
        uint8_t ep = (uint8_t)(BASE_LIGHT_ENDPOINT + ch);
        esp_zb_zcl_reporting_info_t onoff_reporting = {
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                .ep = ep,
                .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_ON_OFF,
                .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                .u.send_info.min_interval = 1,
                .u.send_info.max_interval = 300,
                .u.send_info.def_min_interval = 1,
                .u.send_info.def_max_interval = 300,
                .u.send_info.delta.u8 = 0,
                .attr_id = ESP_ZB_ZCL_ATTR_ON_OFF_ON_OFF_ID,
                .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_zb_zcl_update_reporting_info(&onoff_reporting);
        esp_zb_zcl_reporting_info_t level_reporting = {
                .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
                .ep = ep,
                .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_LEVEL_CONTROL,
                .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
                .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
                .u.send_info.min_interval = 1,
                .u.send_info.max_interval = 300,
                .u.send_info.def_min_interval = 1,
                .u.send_info.def_max_interval = 300,
                .u.send_info.delta.u8 = 1,
                .attr_id = ESP_ZB_ZCL_ATTR_LEVEL_CONTROL_CURRENT_LEVEL_ID,
                .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
        };
        esp_zb_zcl_update_reporting_info(&level_reporting);
    }
    // Add reporting for temperature
    esp_zb_zcl_reporting_info_t temp_reporting = {
            .direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV,
            .ep = BOARD_TEMP_ENDPOINT,
            .cluster_id = ESP_ZB_ZCL_CLUSTER_ID_TEMP_MEASUREMENT,
            .cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
            .dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID,
            .u.send_info.min_interval = 5,
            .u.send_info.max_interval = 300,
            .u.send_info.def_min_interval = 5,
            .u.send_info.def_max_interval = 300,
            .u.send_info.delta.s16 = 50, // 0.50C change
            .attr_id = ESP_ZB_ZCL_ATTR_TEMP_MEASUREMENT_VALUE_ID,
            .manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC,
    };
    esp_zb_zcl_update_reporting_info(&temp_reporting);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    // Multi-channel hardware configuration (ASSUMED GPIOs – adjust to your wiring!)
    static const light_channel_config_t channel_cfg[TOTAL_LIGHT_CHANNELS] = {
        // 12 stair LEDs (single pixel each) – example GPIO sequence (replace with real pins)
        { .gpio = 2, .led_count = 1 }, { .gpio = 3, .led_count = 1 }, { .gpio = 4, .led_count = 1 }, { .gpio = 5, .led_count = 1 },
        { .gpio = 6, .led_count = 1 }, { .gpio = 7, .led_count = 1 }, { .gpio = 8, .led_count = 1 }, { .gpio = 9, .led_count = 1 },
        { .gpio = 10, .led_count = 1 }, { .gpio = 11, .led_count = 1 }, { .gpio = 12, .led_count = 1 }, { .gpio = 13, .led_count = 1 },
        // 2 bed side strips
        { .gpio = 14, .led_count = BED_STRIP_LED_LENGTH }, { .gpio = 15, .led_count = BED_STRIP_LED_LENGTH },
    };
    light_driver_init_channels(channel_cfg, TOTAL_LIGHT_CHANNELS, LIGHT_DEFAULT_OFF);

    esp_zb_platform_config_t config = { .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(), .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(), };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    xTaskCreate(esp_zb_task, "Zigbee_main", 6144, NULL, 5, NULL);
}
