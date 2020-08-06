#pragma once

#include <string>
#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"

class Button : public Module
{
private:
    bool output = false;
    bool pullup = false;

    Port *port;

public:
    Button(std::string name, std::string port) : Module(name)
    {
        this->port = Port::fromString(port.c_str());
    }

    void setup()
    {
        this->port->setup(true, pullup);
    }

    void loop()
    {
        if (output)
            printf("%s %d\n", this->name.c_str(), port->get_level());
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
                port->setup(true, atoi(msg.c_str()));
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else if (command == "get")
        {
            printf("%s get %d\n", this->name.c_str(), port->get_level());
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
