#pragma once

#include "driver/gpio.h"

class Port
{
public:
    gpio_num_t number;

    Port(gpio_num_t number)
    {
        this->number = number;
    }
};
