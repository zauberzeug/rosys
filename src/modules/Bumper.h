#pragma once

#include <string>

#include "Button.h"
#include "../utils/timing.h"

class Bumper : public Button
{
private:
    unsigned long int debounce = 1000;

    bool state = false;
    unsigned long int timeOfLastBump = 0;

public:
    Bumper(std::string name, std::string parameters) : Button(name, parameters)
    {
    }

    void loop()
    {
        if (gpio_get_level(port->number) == 0)
            timeOfLastBump = millis();
        unsigned long millisSinceLastBump = millis() - timeOfLastBump;

        bool newState = millisSinceLastBump <= debounce;
        if (newState != state and output)
            printf("%s %s\n", name.c_str(), newState ? "start" : "stop");

        state = newState;
    }

    void handleMsg(std::string msg)
    {
        std::string original_msg(msg);

        std::string command = cut_first_word(msg);
        std::string key = cut_first_word(msg, '=');

        if (command == "set" and key == "debounce")
        {
            debounce = atoi(msg.c_str());
            return;
        }

        Button::handleMsg(original_msg);
    }
};
