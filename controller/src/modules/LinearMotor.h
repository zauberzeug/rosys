#pragma once

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class LinearMotor : public Module
{
private:
    Port *portMoveIn;
    Port *portMoveOut;
    Port *portStopIn;
    Port *portStopOut;

    enum State
    {
        STOP = 0,
        STOP_IN = 1,
        STOP_OUT = 2,
        MOVE_IN = 3,
        MOVE_OUT = 4,
    };

public:
    LinearMotor(std::string name, std::string parameters) : Module(name)
    {
        this->portMoveIn = Port::fromString(cut_first_word(parameters, ','));
        this->portMoveOut = Port::fromString(cut_first_word(parameters, ','));
        this->portStopIn = Port::fromString(cut_first_word(parameters, ','));
        this->portStopOut = Port::fromString(cut_first_word(parameters, ','));
        this->state = STOP;
    }

    void setup()
    {
        this->portMoveIn->setup(false);
        this->portMoveOut->setup(false);
        if (this->portStopIn)
            this->portStopIn->setup(true);
        if (this->portStopOut)
            this->portStopOut->setup(true);
    }

    void loop()
    {
        if ((this->state == MOVE_IN or this->state == STOP) and this->portStopIn and this->portStopIn->get_level() == 0)
            this->state = STOP_IN;
        if ((this->state == MOVE_OUT or this->state == STOP) and this->portStopOut and this->portStopOut->get_level() == 0)
            this->state = STOP_OUT;

        portMoveIn->set_level(this->state == MOVE_IN ? 1 : 0);
        portMoveOut->set_level(this->state == MOVE_OUT ? 1 : 0);

        Module::loop();
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d", this->state);
        return buffer;
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "in")
        {
            state = MOVE_IN;
        }
        else if (command == "out")
        {
            state = MOVE_OUT;
        }
        else if (command == "stop")
        {
            this->stop();
        }
        else
        {
            Module::handleMsg(command, parameters);
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
        }
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
        }
    }

    void stop()
    {
        if (this->state == MOVE_IN or this->state == MOVE_OUT)
            this->state = STOP;
    }
};
