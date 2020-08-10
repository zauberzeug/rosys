#pragma once

#include "freertos/FreeRTOS.h"

void delay(unsigned int duration_ms);

unsigned long int IRAM_ATTR micros();

unsigned long int IRAM_ATTR millis();

unsigned long millisSince(unsigned long time);
