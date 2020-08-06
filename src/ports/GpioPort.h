#pragma once

#include "driver/gpio.h"

#include "Port.h"

class GpioPort : public Port
{
private:
    gpio_num_t number;

public:
    GpioPort(gpio_num_t number)
    {
        this->number = number;
    }

    void setup(bool input, int pull=0)
    {
        gpio_reset_pin(number);
        gpio_set_direction(number, input ? GPIO_MODE_INPUT : GPIO_MODE_OUTPUT);
        gpio_set_pull_mode(number, pull > 0 ? GPIO_PULLUP_ONLY : pull < 0 ? GPIO_PULLDOWN_ONLY : GPIO_FLOATING);
    }

    void set_level(int level)
    {
        gpio_set_level(number, level);
    }

    int get_level()
    {
        return gpio_get_level(number);
    }
};