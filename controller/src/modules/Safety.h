#pragma once

#include <map>
#include <string>

#include "Module.h"
#include "Condition.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"
#include "../utils/timing.h"

class Safety : public Module
{
private:
    bool active = true;
    unsigned long keep_alive_interval_ms = 0;
    unsigned long last_keep_alive_signal = 0;

    void (*handleCommand)(std::string);

    std::map<std::string, Module *> *modules;
    std::vector<Condition *> conditions;
    std::vector<std::pair<std::string, std::string>> shadows;

public:
    Safety(std::map<std::string, Module *> *modules, void (*handleCommand)(std::string)) : Module("safety")
    {
        this->modules = modules;
        this->handleCommand = handleCommand;
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "list")
        {
            this->list();
        }
        else {
            Module::handleMsg(command, parameters);
        }
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d", this->active);
        return buffer;
    }

    void addCondition(std::string msg)
    {
        conditions.push_back(new Condition(msg));
    }

    void addShadow(std::string msg)
    {
        std::string trigger = cut_first_word(msg);
        bool twoway = cut_first_word(msg) == "<>";
        std::string target = cut_first_word(msg);
        shadows.push_back(std::make_pair(trigger, target));
        if (twoway)
            shadows.push_back(std::make_pair(target, trigger));
    }

    void set(std::string key, std::string value)
    {
        if (key == "active")
        {
            active = value == "1";
        }
        else if (key == "keep_alive_interval")
        {
            keep_alive_interval_ms = atof(value.c_str()) * 1000;
        }
        else
        {
            Module::set(key, value);
        }
    }

    void keep_alive()
    {
        last_keep_alive_signal = millis();
    }

    bool is_alive()
    {
        return keep_alive_interval_ms == 0 or millisSince(last_keep_alive_signal) < keep_alive_interval_ms;
    }

    void applyConditions()
    {
        if (not this->active)
            return;

        for (auto const &condition : conditions)
        {
            if (condition->is_true(modules))
            {
                handleCommand(condition->msg);
            }
        }
    }

    void applyShadow(std::string trigger, std::string command, std::string parameters)
    {
        if (not this->active)
            return;

        for (auto const &item : shadows)
        {
            if (item.first == trigger) {
                (*modules)[item.second]->handleMsg(command, parameters);
            }
        }
    }

    void list()
    {
        cprintln(this->active ? "ACTIVE" : "INACTIVE");
        
        for (auto const &condition : conditions)
        {
            std::string result = condition->is_true(modules) ? " --> triggered" : "";
            cprintln((condition->to_string() + result).c_str());
        }

        for (auto const &item : shadows)
        {
            cprintln("shadow %s > %s", item.first.c_str(), item.second.c_str());
        }
    }
};
