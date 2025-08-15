/**
 * @file ultrasonic.c
 *
 * ESP-IDF driver for ultrasonic range meters, e.g. HC-SR04, HY-SRF05 and so on
 *
 * Ported from esp-open-rtos
 * Copyright (C) 2016, 2018 Ruslan V. Uss <unclerus@gmail.com>
 * BSD Licensed as described in the file LICENSE
 */
#include "ultrasonic.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_timer.h>
#include <esp_log.h>

#define TRIGGER_LOW_DELAY_US   4      // us
#define TRIGGER_HIGH_DELAY_US  10     // us (10us pulse is standard)
#define BLANKING_DELAY_US      200    // us, allow transducer ring-down before listening
#define WAIT_FOR_ECHO_HIGH_US  8000   // us, time to see echo rising edge
#define ROUNDTRIP_US_PER_CM    58     // us for round trip ~ 58us/cm (speed of sound ~343m/s)

static inline uint64_t now_us(void)
{
    return (uint64_t)esp_timer_get_time(); // monotonic, microseconds
}

static inline bool expired_since(uint64_t start_us, uint64_t limit_us)
{
    return (now_us() - start_us) >= limit_us;
}

void ultrasonic_init(const ultrasonic_sensor_t *dev)
{
    // Ensure clean, deterministic GPIO state
    gpio_reset_pin(dev->trigger_pin);
    gpio_reset_pin(dev->echo_pin);

    // Trigger as push-pull output, idle low
    gpio_set_direction(dev->trigger_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(dev->trigger_pin, 0);

    // Echo as input with pulldown to keep stable LOW when sensor idle
    gpio_set_direction(dev->echo_pin, GPIO_MODE_INPUT);
    gpio_set_pull_mode(dev->echo_pin, GPIO_PULLDOWN_ONLY);
}

esp_err_t ultrasonic_measure_cm(const ultrasonic_sensor_t *dev, uint32_t max_distance, int32_t *distance)
{
    if (!distance) return ESP_ERR_INVALID_ARG;

    // Calculate the maximum high time we expect on the echo line for the given max_distance
    const uint32_t max_echo_high_us = max_distance * ROUNDTRIP_US_PER_CM;

    // 1) Ensure echo is LOW before we start a new ping (previous ping ended)
    uint64_t t0 = now_us();
    while (gpio_get_level(dev->echo_pin))
    {
        if (expired_since(t0, WAIT_FOR_ECHO_HIGH_US)) // reuse as a "settle low" timeout
            return ESP_ERR_ULTRASONIC_PING; // still high -> device/bus stuck or previous ping not finished
    }

    // 2) Send trigger pulse
    gpio_set_level(dev->trigger_pin, 0);
    esp_rom_delay_us(TRIGGER_LOW_DELAY_US);
    gpio_set_level(dev->trigger_pin, 1);
    esp_rom_delay_us(TRIGGER_HIGH_DELAY_US);
    gpio_set_level(dev->trigger_pin, 0);

    // Optional: small blanking time to let the waterproof transducer ring down
    esp_rom_delay_us(BLANKING_DELAY_US);

    // 3) Wait for echo to go HIGH (start of measurement)
    t0 = now_us();
    while (!gpio_get_level(dev->echo_pin))
    {
        if (expired_since(t0, WAIT_FOR_ECHO_HIGH_US))
            return ESP_ERR_ULTRASONIC_PING_TIMEOUT; // no rising edge seen
    }

    // 4) Measure how long echo stays HIGH
    const uint64_t echo_start = now_us();
    uint64_t echo_end = echo_start;

    while (gpio_get_level(dev->echo_pin))
    {
        echo_end = now_us();
        // Add timeout so we don't hang if line gets stuck HIGH
        if (expired_since(echo_start, max_echo_high_us + 2000 /* small margin */))
            break;
    }

    // Compute duration
    const uint64_t dt_us = (echo_end > echo_start) ? (echo_end - echo_start) : 0;

    // 5) Convert to cm
    const uint32_t cm = (uint32_t)(dt_us / ROUNDTRIP_US_PER_CM);

    if (cm == 0 && dt_us == 0)
        return ESP_ERR_ULTRASONIC_PING_TIMEOUT;

    if (cm > max_distance)
        return ESP_ERR_ULTRASONIC_ECHO_TIMEOUT;

    *distance = (int32_t)cm;
    return ESP_OK;
}