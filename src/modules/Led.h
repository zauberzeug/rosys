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
            gpio_set_level(port->number, sin(esp_timer_get_time() / 1000.0 * 3.1415 / interval));
            break;
        default:
            printf("Invalid state: %d\n", state);
        }
    }

    void handleMsg(std::string msg)
    {
        if (msg == "on")
        {
            state = ON;
        }
        else if (msg == "off")
        {
            state = OFF;
        }
        else if (msg == "pulse")
        {
            state = PULSE;
        }
        else if (msg.substr(0, 4) == "set ")
        {
            int space = msg.find(' ');
            int equal = msg.find('=');
            std::string key = msg.substr(space + 1, equal);
            double value = atof(msg.substr(equal + 1).c_str());
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
            printf("Unknown command: %s\n", msg.c_str());
        }
    }
};
