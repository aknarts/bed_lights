#include "ws2812fx_stub.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "light_driver.h"
#include "esp_log.h"

static const char *TAG = "WS2812FX";
static TaskHandle_t fx_task_handle = NULL;

static void fx_task(void *arg)
{
    // Simple breathing effect placeholder
    uint8_t level = 0;
    int dir = 1;
    while (1) {
        light_driver_set_level(level);
        level = (uint8_t)(level + dir * 5);
        if (level >= 250) { level = 250; dir = -1; }
        if (level <= 5) { level = 5; dir = 1; }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

void ws2812fx_init(void)
{
    ESP_LOGI(TAG, "FX init");
}

void ws2812fx_start(void)
{
    if (!fx_task_handle) {
        xTaskCreate(fx_task, "fx_task", 2048, NULL, 4, &fx_task_handle);
    }
}

