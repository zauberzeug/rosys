#pragma once

#include <map>
#include <string>

#include "Module.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Safety : public Module
{
private:
    bool active = true;

    std::map<std::string, Module *> *modules;
    std::map<std::string, std::tuple<std::string, std::string, int>> conditions;
    std::map<std::string, std::pair<std::string, std::string>> shadows;

public:
    Safety(std::map<std::string, Module *> *modules) : Module("safety")
    {
        this->modules = modules;
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

    void set(std::string key, std::string value)
    {
        if (starts_with(key, "condition_"))
        {
            cut_first_word(key, '_');
            std::string module = cut_first_word(value, ',');
            std::string trigger = cut_first_word(value, ',');
            int state = atoi(value.c_str());
            conditions[key] = std::make_tuple(module, trigger, state);
        }
        else if (starts_with(key, "shadow_"))
        {
            cut_first_word(key, '_');
            std::string trigger = cut_first_word(value, ',');
            std::string shadow = cut_first_word(value, ',');
            shadows[key] = std::make_pair(trigger, shadow);
        }
        else if (key == "active")
        {
            active = value == "1";
        }
        else
        {
            Module::set(key, value);
        }
    }

    bool check(Module *module)
    {
        if (not this->active)
            return true;

        for (auto const &item : conditions)
        {
            std::string name = std::get<0>(item.second);
            std::string trigger = std::get<1>(item.second);
            int state = std::get<2>(item.second);
            if ((name == "*" || name == module->name) && state != (*modules)[trigger]->state)
                return false;
        }
        return true;
    }

    void applyShadow(std::string trigger, std::string command, std::string parameters)
    {
        if (not this->active)
            return;

        for (auto const &item : shadows)
        {
            if (item.second.first == trigger) {
                (*modules)[item.second.second]->handleMsg(command, parameters);
            }
        }
    }

    void list()
    {
        cprintln(this->active ? "ACTIVE" : "INACTIVE");
        
        for (auto const &item : conditions)
        {
            std::string name = std::get<0>(item.second);
            std::string trigger = std::get<1>(item.second);
            int state = std::get<2>(item.second);
            const char* result = state == (*modules)[trigger]->state ? "ok" : "violated";
            cprintln("condition \"%s\" %s,%s,%d: %s", item.first.c_str(), name.c_str(), trigger.c_str(), state, result);
        }

        for (auto const &item : shadows)
        {
            std::string trigger = item.second.first;
            std::string shadow = item.second.second;
            cprintln("shadow \"%s\" %s->%s", item.first.c_str(), trigger.c_str(), shadow.c_str());
        }
    }
};
