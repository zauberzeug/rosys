#pragma once

#include "Module.h"
#include "../storage.h"
#include "../utils/strings.h"

class Esp : public Module
{
public:
    Esp() : Module("esp")
    {
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "restart")
        {
            esp_restart();
        }
        else if (command == "print")
        {
            printf("%s\n", msg.c_str());
        }
        else if (command == "erase")
        {
            storage::erase();
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
