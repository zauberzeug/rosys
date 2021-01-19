#pragma once

#include <vector>

#include "Module.h"
#include "../storage.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Esp : public Module
{
private:
    std::map<std::string, Module *> *modules;
    std::vector<std::string> *outputModules = new std::vector<std::string>();

public:
    Esp(std::map<std::string, Module *> *modules) : Module("esp")
    {
        this->modules = modules;
    }

    std::string getOutput()
    {
        std::string result = "";
        for (const auto name : *(this->outputModules)) {
            if (not result.empty())
                result += " ";
            result += (*(this->modules))[name]->getOutput();
        }
        return result;
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "restart")
        {
            esp_restart();
        }
        else if (command == "print")
        {
            cprintln("%s", parameters.c_str());
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
            Module::handleMsg(command, parameters);
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "outputModules")
        {
            this->outputModules->clear();
            while (not value.empty())
            {
                this->outputModules->push_back(cut_first_word(value, ','));
            }
        }
        else
        {
            Module::set(key, value);
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
