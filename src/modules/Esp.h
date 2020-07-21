#pragma once

#include "Module.h"

class Esp : public Module
{
    void handleMsg(std::string msg)
    {
        if (msg == "restart")
        {
            esp_restart();
        }
        else
        {
            printf("Unknown command: %s\n", msg.c_str());
        }
    }
};
