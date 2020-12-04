#pragma once

#include <string>
#include <math.h>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"

class Motor : public Module
{
private:
    Port *dir_port;
    Port *pwm_port;

public:
    Motor(std::string name, std::string parameters) : Module(name)
    {
        this->dir_port = Port::fromString(cut_first_word(parameters, ','));
        this->pwm_port = Port::fromString(cut_first_word(parameters, ','));
    }

    void setup()
    {
        this->dir_port->setup(false);
        this->pwm_port->setup(false);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "pw")
        {
            double pw = atof(msg.c_str());
            this->dir_port->set_level(pw > 0);
            this->pwm_port->set_level(fabs(pw));
            printf("%s %s completed\n", name.c_str(), command.c_str());
        }
        else if (command == "up")
        {
            // NOTE: deprecated
            handleMsg("pw 1");
            printf("%s %s completed\n", name.c_str(), command.c_str());
        }
        else if (command == "down")
        {
            // NOTE: deprecated
            handleMsg("pw -1");
            printf("%s %s completed\n", name.c_str(), command.c_str());
        }
        else if (command == "stop")
        {
            stop();
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }

    void stop()
    {
        this->pwm_port->set_level(0);
    }
};