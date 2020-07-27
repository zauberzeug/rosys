#pragma once

#include <string>
#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "../Port.h"
#include "../utils/strings.h"

class Button : public Module
{
private:
    bool output = false;
    bool pullup = false;

    Port *port;

public:
    Button(std::string name, std::string parameters) : Module(name)
    {
        this->port = new Port((gpio_num_t)atoi(parameters.c_str()));
    }

    void setup()
    {
        gpio_reset_pin(port->number);
        gpio_set_direction(port->number, GPIO_MODE_INPUT);
        gpio_set_pull_mode(port->number, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
    }

    void loop()
    {
        if (output)
            printf("%s %d\n", this->name.c_str(), gpio_get_level(port->number));
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
            {
                output = msg == "1";
            }
            else if (key == "pullup")
            {
                pullup = msg == "1";
                gpio_set_pull_mode(port->number, pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING);
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else if (command == "get")
        {
            printf("%s get %d\n", this->name.c_str(), gpio_get_level(port->number));
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
