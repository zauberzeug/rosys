#pragma once

#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "../Port.h"
#include "../utils/strings.h"

class Button : public Module
{
public:
    Port *port;

    bool output = false;
    bool pullup = false;

    Button(Port *port)
    {
        this->port = port;
    }

    void setup()
    {
        gpio_reset_pin(port->number);
        gpio_set_direction(port->number, GPIO_MODE_INPUT);
        gpio_set_pull_mode(port->number, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
    }

    void print_state()
    {
        printf("%s %d\n", this->name.c_str(), gpio_get_level(port->number));
    }

    void loop()
    {
        if (output)
            print_state();
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
            {
                output = msg == "on";
            }
            else if (key == "pullup")
            {
                pullup = msg == "on";
                gpio_set_pull_mode(port->number, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else if (command == "get")
        {
            print_state();
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
