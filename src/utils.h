#pragma once

#include <string>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void delay(unsigned int duration_ms)
{
    vTaskDelay(duration_ms / portTICK_PERIOD_MS);
}

std::string cut_first_word(std::string &msg, char delimiter = ' ')
{
    int space = msg.find(' ');
    std::string word = space < 0 ? msg : msg.substr(0, space);
    msg = space < 0 ? std::string() : msg.substr(space + 1);
    return word;
}