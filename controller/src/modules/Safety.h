#pragma once

#include <map>
#include <string>

#include "Module.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"
#include "../utils/timing.h"

class Safety : public Module
{
private:
    bool active = true;
    unsigned long keep_alive_interval_ms = 0;
    unsigned long last_keep_alive_signal = 0;

    std::map<std::string, Module *> *modules;
    std::vector<std::tuple<std::string, bool, int, std::string, std::string>> conditions;
    std::vector<std::pair<std::string, std::string>> shadows;

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

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d", this->active);
        return buffer;
    }

    void addCondition(std::string msg)
    {
        std::string trigger = cut_first_word(msg);
        bool equality = cut_first_word(msg) == "==";
        int state = atoi(cut_first_word(msg).c_str());
        std::string target = cut_first_word(msg);
        conditions.push_back(std::make_tuple(trigger, equality, state, target, msg));
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

    void applyConditions(Module *module)
    {
        if (not this->active)
            return;

        for (auto const &item : conditions)
        {
            std::string trigger = std::get<0>(item);
            bool equality = std::get<1>(item);
            int state = std::get<2>(item);
            std::string target = std::get<3>(item);
            std::string msg = std::get<4>(item);
            if (target != module->name)
            {
                continue;
            }
            if ((state == (*modules)[trigger]->state) == equality)
            {
                std::string command = cut_first_word(msg);
                (*modules)[target]->handleMsg(command, msg);
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
        
        for (auto const &item : conditions)
        {
            std::string trigger = std::get<0>(item);
            bool equality = std::get<1>(item);
            int state = std::get<2>(item);
            std::string target = std::get<3>(item);
            std::string msg = std::get<4>(item);
            const char* result = (state == (*modules)[trigger]->state) == equality ? "--> triggered" : "";
            cprintln("if %s %s %d %s %s %s", trigger.c_str(), equality ? "==" : "!=", state, target.c_str(), msg.c_str(), result);
        }

        for (auto const &item : shadows)
        {
            cprintln("shadow %s > %s", item.first.c_str(), item.second.c_str());
        }
    }
};
