#pragma once

#include <string>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Motor : public Module
{
private:
    Port *dir_port;
    Port *pwm_port;

    enum State
    {
        STOP = 0,
        MOVE_FORWARD = 1,
        MOVE_BACKWARD = 2,
    };

public:
    Motor(std::string name, std::string parameters) : Module(name)
    {
        this->dir_port = Port::fromString(cut_first_word(parameters, ','));
        this->pwm_port = Port::fromString(cut_first_word(parameters, ','));
        this->state = STOP;
    }

    void setup()
    {
        this->dir_port->setup(false);
        this->pwm_port->setup(false);
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d", this->state);
        return buffer;
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "pw")
        {
            double pw = atof(parameters.c_str());
            this->dir_port->set_level(pw > 0);
            this->pwm_port->set_level(fabs(pw));
            this->state = pw > 0 ? MOVE_FORWARD : MOVE_BACKWARD;
        }
        else if (command == "up")
        {
            // NOTE: deprecated
            handleMsg("pw", "1");
        }
        else if (command == "down")
        {
            // NOTE: deprecated
            handleMsg("pw", "-1");
        }
        else if (command == "stop")
        {
            stop();
        }
        else
        {
            Module::handleMsg(command, parameters);
        }
    }

    void stop()
    {
        this->pwm_port->set_level(0);
        this->state = STOP;
    }
};
