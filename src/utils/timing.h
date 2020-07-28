#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void delay(unsigned int duration_ms)
{
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
}

unsigned long int IRAM_ATTR micros()
{
    return (unsigned long)(esp_timer_get_time());
}

unsigned long int IRAM_ATTR millis()
{
    return (unsigned long)(esp_timer_get_time() / 1000ULL);
}

unsigned long millisSince(unsigned long time)
{
    return millis() - time;
}
