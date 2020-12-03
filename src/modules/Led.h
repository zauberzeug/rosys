#pragma once

#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"

class Led : public Module
{
private:
    bool output = false;
    double interval = 1.0;

    Port *port;

    enum State
    {
        ON,
        OFF,
        PULSE,
    } state;

public:
    Led(std::string name, std::string port) : Module(name)
    {
        this->port = Port::fromString(port);
        this->state = OFF;
    }

    void setup()
    {
        port->setup(false);
    }

    int level()
    {
        if (state == ON)
            return 1;
        if (state == OFF)
            return 0;
        if (state == PULSE)
            return sin(esp_timer_get_time() / interval * 2e-6 * M_PI) > 0 ? 1 : 0;
        return -1;
    }

    void loop()
    {
        port->set_level(level());
        if (output)
            printf("%s %d\n", this->name.c_str(), level());
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
            if (key == "output")
            {
                output = msg == "1";
            }
            else if (key == "interval")
            {
                interval = value;
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else if (command == "get")
        {
            printf("%s get %d\n", this->name.c_str(), level());
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }

    void stop()
    {
        this->state = OFF;
    }
};
