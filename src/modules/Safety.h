#pragma once

#include <map>
#include <string>

#include "Module.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Safety : public Module
{
private:
    std::map<std::string, Module *> *modules;
    std::map<std::string, std::tuple<std::string, std::string, int>> conditions;

public:
    Safety(std::map<std::string, Module *> *modules) : Module("safety")
    {
        this->modules = modules;
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "list")
        {
            this->list();
        }
        else {
            cprintln("Unknown command: %s", command.c_str());
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
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
        }
    }

    bool check(Module *module)
    {
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

    void list()
    {
        for (auto const &item : conditions)
        {
            std::string name = std::get<0>(item.second);
            std::string trigger = std::get<1>(item.second);
            int state = std::get<2>(item.second);
            const char* result = state == (*modules)[trigger]->state ? "ok" : "violated";
            cprintln("\"%s\" %s,%s,%d: %s", item.first.c_str(), name.c_str(), trigger.c_str(), state, result);
        }
    }
};
