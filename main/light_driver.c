/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee light driver example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */


#include "esp_log.h"
#include "led_strip.h"
#include "light_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_random.h"

static const char *LD_TAG = "light_drv";

// Remove single-channel globals and replace with per-channel state
// (retain CONFIG_EXAMPLE_* macros from header for backward compatible wrappers)
// #undef CONFIG_EXAMPLE_STRIP_LED_NUMBER
// #undef CONFIG_EXAMPLE_STRIP_LED_GPIO

#define MAX_LIGHT_CHANNELS 16

typedef struct {
    led_strip_handle_t handle;
    uint16_t led_count;
    uint8_t r, g, b, level;
    bool power;
    light_effect_t effect;
    TaskHandle_t effect_task;
    bool effect_stop;
} light_channel_state_t;

static light_channel_state_t s_channels[MAX_LIGHT_CHANNELS];
static size_t s_channel_count = 0;
static SemaphoreHandle_t s_driver_lock;

static inline bool ch_valid(size_t ch) { return ch < s_channel_count; }

static inline void apply_output_ch(light_channel_state_t *ch)
{
    if (!ch || !ch->handle) return;
    if (ch->power) {
        float ratio = (float) ch->level / 255.0f;
        uint8_t rr = (uint8_t) ((float) ch->r * ratio);
        uint8_t gg = (uint8_t) ((float) ch->g * ratio);
        uint8_t bb = (uint8_t) ((float) ch->b * ratio);
        for (uint16_t i = 0; i < ch->led_count; ++i) {
            led_strip_set_pixel(ch->handle, i, rr, gg, bb);
        }
    } else {
        for (uint16_t i = 0; i < ch->led_count; ++i) {
            led_strip_set_pixel(ch->handle, i, 0, 0, 0);
        }
    }
    led_strip_refresh(ch->handle);
}

static void color_temp_to_rgb(uint16_t mired, uint8_t *r, uint8_t *g, uint8_t *b)
{
    // mired = 1,000,000 / K. Clamp typical range 153 (6500K) - 500 (2000K)
    if (mired < 153) mired = 153;
    if (mired > 500) mired = 500;
    float kelvin = 1000000.0f / (float)mired; // ~2000 - 6500
    float temp = kelvin / 100.0f;
    float rr, gg, bb;
    if (temp <= 66) {
        rr = 255.0f;
    } else {
        rr = 329.698727446f * powf(temp - 60.0f, -0.1332047592f);
        if (rr < 0) rr = 0;
        if (rr > 255) rr = 255;
    }
    if (temp <= 66) {
        gg = 99.4708025861f * logf(temp) - 161.1195681661f;
    } else {
        gg = 288.1221695283f * powf(temp - 60.0f, -0.0755148492f);
    }
    if (gg < 0) gg = 0;
    if (gg > 255) gg = 255;
    if (temp >= 66) {
        bb = 255.0f;
    } else if (temp <= 19) {
        bb = 0;
    } else {
        bb = 138.5177312231f * logf(temp - 10.0f) - 305.0447927307f;
        if (bb < 0) bb = 0;
        if (bb > 255) bb = 255;
    }
    *r = (uint8_t)rr; *g = (uint8_t)gg; *b = (uint8_t)bb;
}

// Effect task per channel
static void effect_task_ch(void *arg)
{
    size_t ch_index = (size_t) arg;
    if (!ch_valid(ch_index)) { vTaskDelete(NULL); return; }
    light_channel_state_t *st = &s_channels[ch_index];
    uint8_t breathe_level = st->level ? st->level : 1;
    int breathe_dir = 1;
    while (!st->effect_stop) {
        switch (st->effect) {
            case LIGHT_EFFECT_BLINK: {
                st->power = !st->power; apply_output_ch(st); vTaskDelay(pdMS_TO_TICKS(500)); break; }
            case LIGHT_EFFECT_BREATHE: {
                breathe_level += breathe_dir * 5;
                if (breathe_level >= (int) st->level) { breathe_level = st->level; breathe_dir = -1; }
                if (breathe_level <= 5) { breathe_level = 5; breathe_dir = 1; }
                if (st->power) {
                    float ratio = (float) breathe_level / 255.0f;
                    uint8_t rr = (uint8_t) ((float) st->r * ratio);
                    uint8_t gg = (uint8_t) ((float) st->g * ratio);
                    uint8_t bb = (uint8_t) ((float) st->b * ratio);
                    for (uint16_t i=0;i<st->led_count;++i) led_strip_set_pixel(st->handle, i, rr, gg, bb);
                    led_strip_refresh(st->handle);
                }
                vTaskDelay(pdMS_TO_TICKS(40));
                break; }
            case LIGHT_EFFECT_ICU: {
                st->power = true; apply_output_ch(st); vTaskDelay(pdMS_TO_TICKS(120));
                st->power = false; apply_output_ch(st); vTaskDelay(pdMS_TO_TICKS(120));
                st->power = true; apply_output_ch(st); vTaskDelay(pdMS_TO_TICKS(120));
                st->power = false; apply_output_ch(st); vTaskDelay(pdMS_TO_TICKS(500));
                break; }
            case LIGHT_EFFECT_RANDOM_COLOR: {
                st->r = (uint8_t) (esp_random() & 0xFF);
                st->g = (uint8_t) (esp_random() >> 8);
                st->b = (uint8_t) (esp_random() >> 16);
                apply_output_ch(st); vTaskDelay(pdMS_TO_TICKS(700));
                break; }
            case LIGHT_EFFECT_STATIC:
            case LIGHT_EFFECT_NONE:
            default:
                vTaskDelay(pdMS_TO_TICKS(200));
                break;
        }
    }
    st->effect_task = NULL;
    vTaskDelete(NULL);
}

void light_driver_init_channels(const light_channel_config_t *channels, size_t count, bool power_default)
{
    if (!channels || count == 0) return;
    if (count > MAX_LIGHT_CHANNELS) {
        ESP_LOGW(LD_TAG, "Requested %u channels, limiting to %d", (unsigned)count, MAX_LIGHT_CHANNELS);
        count = MAX_LIGHT_CHANNELS;
    }
    if (!s_driver_lock) s_driver_lock = xSemaphoreCreateMutex();
    xSemaphoreTake(s_driver_lock, portMAX_DELAY);
    if (s_channel_count) { // already initialized
        xSemaphoreGive(s_driver_lock); return; }
    for (size_t i = 0; i < count; ++i) {
        led_strip_config_t cfg = { .strip_gpio_num = channels[i].gpio, .max_leds = channels[i].led_count };
        led_strip_rmt_config_t rmt = { .resolution_hz = 10 * 1000 * 1000 };
        led_strip_handle_t handle;
        esp_err_t err = led_strip_new_rmt_device(&cfg, &rmt, &handle);
        if (err == ESP_OK && handle) {
            s_channels[i].handle = handle;
            s_channels[i].led_count = channels[i].led_count;
            s_channels[i].r = 255; s_channels[i].g = 255; s_channels[i].b = 255;
            s_channels[i].level = 255; s_channels[i].power = power_default;
            s_channels[i].effect = LIGHT_EFFECT_NONE; s_channels[i].effect_task = NULL; s_channels[i].effect_stop = false;
            apply_output_ch(&s_channels[i]);
            ESP_LOGI(LD_TAG, "Channel %u init OK (GPIO %d, leds %u)", (unsigned)i, channels[i].gpio, channels[i].led_count);
        } else {
            ESP_LOGE(LD_TAG, "Channel %u init FAILED (GPIO %d, err %s)", (unsigned)i, channels[i].gpio, esp_err_to_name(err));
            s_channels[i].handle = NULL;
        }
    }
    s_channel_count = count;
    xSemaphoreGive(s_driver_lock);
}

size_t light_driver_channel_count(void) { return s_channel_count; }

static void set_color_xy_internal(light_channel_state_t *st, uint16_t x, uint16_t y)
{
    float red_f = 0, green_f = 0, blue_f = 0, cx, cy;
    cx = (float) x / 65535.0f;
    cy = (float) y / 65535.0f;
    if (cy < 0.00001f) return;
    float X = cx / cy;
    float Z = (1.0f - cx - cy) / cy;
    XYZ_to_RGB(X, 1.0f, Z, red_f, green_f, blue_f);
    if (red_f < 0) red_f = 0;
    if (green_f < 0) green_f = 0;
    if (blue_f < 0) blue_f = 0;
    if (red_f > 1) red_f = 1;
    if (green_f > 1) green_f = 1;
    if (blue_f > 1) blue_f = 1;
    st->r = (uint8_t) (red_f * 255.0f);
    st->g = (uint8_t) (green_f * 255.0f);
    st->b = (uint8_t) (blue_f * 255.0f);
    if (st->power) apply_output_ch(st);
}

void light_driver_set_power_ch(size_t ch, bool power) { if (!ch_valid(ch)) return; s_channels[ch].power = power; apply_output_ch(&s_channels[ch]); }
void light_driver_set_level_ch(size_t ch, uint8_t level) { if (!ch_valid(ch)) return; s_channels[ch].level = level; if (s_channels[ch].power) apply_output_ch(&s_channels[ch]); }
void light_driver_set_color_RGB_ch(size_t ch, uint8_t red, uint8_t green, uint8_t blue) { if (!ch_valid(ch)) return; s_channels[ch].r=red; s_channels[ch].g=green; s_channels[ch].b=blue; if (s_channels[ch].power) apply_output_ch(&s_channels[ch]); }
void light_driver_set_color_xy_ch(size_t ch, uint16_t x, uint16_t y) { if (!ch_valid(ch)) return; set_color_xy_internal(&s_channels[ch], x, y); }
void light_driver_set_color_hue_sat_ch(size_t ch, uint8_t hue, uint8_t sat) { if (!ch_valid(ch)) return; float rf,gf,bf; HSV_to_RGB(hue,sat,UINT8_MAX,rf,gf,bf); s_channels[ch].r=(uint8_t)rf; s_channels[ch].g=(uint8_t)gf; s_channels[ch].b=(uint8_t)bf; if (s_channels[ch].power) apply_output_ch(&s_channels[ch]); }
void light_driver_set_color_temperature_mired_ch(size_t ch, uint16_t mired) { if (!ch_valid(ch)) return; color_temp_to_rgb(mired,&s_channels[ch].r,&s_channels[ch].g,&s_channels[ch].b); if (s_channels[ch].power) apply_output_ch(&s_channels[ch]); }

void light_driver_effect_start_ch(size_t ch, light_effect_t effect)
{ if (!ch_valid(ch)) return; light_channel_state_t *st=&s_channels[ch]; if (effect==LIGHT_EFFECT_NONE){ light_driver_effect_stop_ch(ch); return;} st->effect=effect; st->effect_stop=false; if(!st->effect_task){ xTaskCreate(effect_task_ch,"fx_ch",2048,(void*)ch,4,&st->effect_task);} }
void light_driver_effect_stop_ch(size_t ch)
{ if (!ch_valid(ch)) return; light_channel_state_t *st=&s_channels[ch]; if(st->effect_task){ st->effect_stop=true; for(int i=0;i<10 && st->effect_task;++i) vTaskDelay(pdMS_TO_TICKS(20)); } st->effect=LIGHT_EFFECT_NONE; st->power=true; apply_output_ch(st); }

// Single-channel backward compatible wrappers operate on channel 0
void light_driver_init(bool power) { light_channel_config_t def={ .gpio=CONFIG_EXAMPLE_STRIP_LED_GPIO, .led_count=CONFIG_EXAMPLE_STRIP_LED_NUMBER }; light_driver_init_channels(&def,1,power); }
void light_driver_set_power(bool power) { light_driver_set_power_ch(0,power); }
void light_driver_set_level(uint8_t level) { light_driver_set_level_ch(0,level); }
void light_driver_set_color_RGB(uint8_t r,uint8_t g,uint8_t b){ light_driver_set_color_RGB_ch(0,r,g,b);}
void light_driver_set_color_xy(uint16_t x,uint16_t y){ light_driver_set_color_xy_ch(0,x,y);}
void light_driver_set_color_hue_sat(uint8_t h,uint8_t s){ light_driver_set_color_hue_sat_ch(0,h,s);}
void light_driver_set_color_temperature_mired(uint16_t m){ light_driver_set_color_temperature_mired_ch(0,m);}
void light_driver_effect_start(light_effect_t e){ light_driver_effect_start_ch(0,e);}
void light_driver_effect_stop(void){ light_driver_effect_stop_ch(0);}
