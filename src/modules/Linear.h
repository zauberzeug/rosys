#pragma once

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"

class Linear : public Module
{
private:
    bool output = false;

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
    Linear(std::string name, std::string parameters) : Module(name)
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
        this->portStopIn->setup(true);
        this->portStopOut->setup(true);
    }

    void loop()
    {
        if (this->state == MOVE_IN and this->portStopIn->get_level() == 0)
            this->state = STOP_IN;
        if (this->state == MOVE_OUT and this->portStopOut->get_level() == 0)
            this->state = STOP_OUT;

        portMoveIn->set_level(this->state == MOVE_IN ? 1 : 0);
        portMoveOut->set_level(this->state == MOVE_OUT ? 1 : 0);

        if (output)
            printf("%s %d\n", this->name.c_str(), this->state);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

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
        else if (command == "get")
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
        else
        {
            printf("Unknown setting: %s\n", key.c_str());
        }
    }

    void stop()
    {
        if (this->state == MOVE_IN or this->state == MOVE_OUT)
            this->state = STOP;
    }
};
