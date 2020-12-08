#pragma once

#include "Module.h"
#include "../storage.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Esp : public Module
{
private:
    std::map<std::string, Module *> *modules;

public:
    Esp(std::map<std::string, Module *> *modules) : Module("esp")
    {
        this->modules = modules;
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
            cprintln("%s", msg.c_str());
        }
        else if (command == "erase")
        {
            storage::put("");
        }
        else if (command == "stop")
        {
            stopAll();
        }
        else
        {
            cprintln("Unknown command: %s", command.c_str());
        }
    }

    void stopAll()
    {
        for (auto const &item : *modules)
        {
            item.second->stop();
        }
    }
};
