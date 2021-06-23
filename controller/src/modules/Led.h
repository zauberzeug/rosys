#pragma once

#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Led : public Module
{
private:
    double interval = 1.0;

    Port *port;

    enum State
    {
        OFF = 0,
        ON = 1,
        PULSE = 2,
    };

    int level = 0;
    unsigned long int lastChange = 0;

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

    void loop()
    {
        if (state == OFF)
            level = 0;
        if (state == ON)
            level = 1;
        if (state == PULSE and millisSince(lastChange) > interval / 2.0 * 1000.0) {
            level = 1 - level;
            lastChange = millis();
        }

        port->set_level(level);

        Module::loop();
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d", this->level);
        return buffer;
    }

    void handleMsg(std::string command, std::string parameters)
    {
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
        else
        {
            Module::handleMsg(command, parameters);
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "interval")
        {
            interval = atof(value.c_str());
        }
        else
        {
            Module::set(key, value);
        }
    }

    void stop()
    {
        this->state = OFF;
    }
};
