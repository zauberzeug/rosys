#pragma once

#include <vector>
#include <string>

#include "Module.h"
#include "../ports/Port.h"
#include "../ports/GpioPort.h"
#include "../storage.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"
#include "../utils/timing.h"

class Esp : public Module
{
private:
    std::map<std::string, Module *> *modules;
    std::vector<std::string> *outputModules = new std::vector<std::string>();
    bool ready = false;
    bool en24v = false;
    Port *readyPort;
    Port *en24vPort;

public:
    Esp(std::map<std::string, Module *> *modules) : Module("esp")
    {
        this->modules = modules;
        this->readyPort = new GpioPort(GPIO_NUM_15);
        this->en24vPort = new GpioPort(GPIO_NUM_12);
    }

    void setup()
    {
        this->readyPort->setup(false);
        this->en24vPort->setup(false);
    }

    void loop()
    {
        printf("%d %d\n", ready ? 1 : 0, en24v ? 1 : 0);
        this->readyPort->set_level(ready ? 1 : 0);
        this->en24vPort->set_level(en24v ? 1 : 0);

        Module::loop();
    }

    std::string getOutput()
    {
        std::string result = std::to_string(millis());
        for (const auto name : *(this->outputModules))
        {
            if (not result.empty())
                result += " ";
            result += (*(this->modules))[name]->getOutput();
        }
        if (ready)
        {
            result += " RDY";
        }
        if (en24v)
        {
            result += " 24V";
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
        else if (command == "ping")
        {
            // do nothing
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
        else if (key == "ready")
        {
            this->ready = value == "1";
        }
        else if (key == "24v")
        {
            this->en24v = value == "1";
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
