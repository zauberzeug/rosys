#pragma once

#include <string>
#include "driver/gpio.h"

#include "Module.h"
#include "../Port.h"
#include "../utils/strings.h"

class DualMotor : public Module
{
public:
    Port *rx_port;
    Port *tx_port;

    bool output = false;

    DualMotor(std::string name, std::string parameters) : Module(name)
    {
        this->rx_port = new Port((gpio_num_t) atoi(cut_first_word(parameters, ',').c_str()));
        this->tx_port = new Port((gpio_num_t) atoi(cut_first_word(parameters, ',').c_str()));
    }

    void setup()
    {
    }

    void loop()
    {
        if (output)
            printf("%s ...\n", this->name.c_str());
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
