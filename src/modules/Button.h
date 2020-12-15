#pragma once

#include <string>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Button : public Module
{
private:
    bool invert = false;
    int pullup = 0;

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
        this->state = invert ? 1 - port->get_level() : port->get_level();

        Module::loop();
    }

    std::string output()
    {
        char buffer[256];
        std::sprintf(buffer, "%d", this->state);
        return buffer;
    }

    void set(std::string key, std::string value)
    {
        if (key == "invert")
        {
            invert = value == "1";
        }
        else if (key == "pullup")
        {
            pullup = atoi(value.c_str());
            port->setup(true, pullup);
        }
        else
        {
            Module::set(key, value);
        }
    }
};
