#pragma once

#include "Module.h"
#include "../configuration.h"

class Esp : public Module
{
    void setup()
    {
    }

    void loop()
    {
    }

    void handleMsg(std::string msg)
    {
        if (msg == "restart")
        {
            esp_restart();
        }
        else if (msg == "erase")
        {
            configuration::erase();
        }
        else
        {
            printf("Unknown command: %s\n", msg.c_str());
        }
    }
};
