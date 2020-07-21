#pragma once

#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "Port.h"

class Led : public Module
{
public:
    Port *port;

    enum State
    {
        ON,
        OFF,
        PULSE,
    } state;

    double interval = 1.0;

    Led(Port *port)
    {
        this->port = port;
        this->state = OFF;
    }

    void setup()
    {
        gpio_reset_pin(port->number);
        gpio_set_direction(port->number, GPIO_MODE_OUTPUT);
    }

    void loop()
    {
        switch (state)
        {
        case ON:
            gpio_set_level(port->number, 1);
            break;
        case OFF:
            gpio_set_level(port->number, 0);
            break;
        case PULSE:
            gpio_set_level(port->number, sin(esp_timer_get_time() / interval * 2e-6 * M_PI) > 0 ? 1 : 0);
            break;
        default:
            printf("Invalid state: %d\n", state);
        }
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "on")
        {
            state = ON;
        }
        else if (command == "off")
        {
            state = OFF;
        }
        else if (command == "pulse")
        {
            state = PULSE;
        }
        else if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            double value = atof(msg.c_str());
            if (key == "interval")
            {
                interval = value;
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
