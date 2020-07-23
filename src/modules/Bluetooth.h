#pragma once

#include <string>

#include "Module.h"
#include "bluetooth/ble.h"
#include "../utils/strings.h"

class Bluetooth : public Module
{
public:
    Bluetooth(std::string name, std::string device_name, void (*globalMessageHandler)(std::string)) : Module(name)
    {
        Ble::init(device_name, globalMessageHandler);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "send")
        {
            printf("Sending...\n");
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
