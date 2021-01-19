#pragma once

#include <string>

#include "Module.h"
#include "bluetooth/ble.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Bluetooth : public Module
{
public:
    Bluetooth(std::string name, std::string device_name, void (*globalMessageHandler)(std::string)) : Module(name)
    {
        Ble::init(device_name, globalMessageHandler);
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "send")
        {
            cprintln("Sending...");
        }
        else
        {
            Module::handleMsg(command, parameters);
        }
    }
};
