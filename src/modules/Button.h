#pragma once

#include <string>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"

class Button : public Module
{
private:
    bool output = false;
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

        if (output)
            printf("%s %d\n", this->name.c_str(), this->state);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "get")
        {
            printf("%s get %d\n", this->name.c_str(), this->state);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
        }
        else if (key == "invert")
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
            printf("Unknown setting: %s\n", key.c_str());
        }
    }
};
