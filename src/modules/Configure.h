#pragma once

#include <string>
#include <map>

#include "../utils/strings.h"
#include "Module.h"
#include "Bluetooth.h"
#include "Led.h"
#include "Button.h"
#include "../Port.h"
#include "../storage.h"

class Configure : public Module
{
private:
    std::map<std::string, Module *> modules;
    void (*globalMessageHandler)(std::string);

public:
    Configure(std::map<std::string, Module *> &modules, void (*globalMessageHandler)(std::string)) : Module("configure")
    {
        this->modules = modules;
        this->globalMessageHandler = globalMessageHandler;

        nvs_iterator_t it;
        nvs_entry_info_t info;

        printf("Reading modules from persistent storage...\n");
        it = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY);
        while (it != NULL)
        {
            nvs_entry_info(it, &info);
            it = nvs_entry_next(it);

            std::string namespace_ = std::string(info.namespace_name);
            if (namespace_ != "configure")
                continue;

            std::string key = std::string(info.key);
            std::string value = storage::get(namespace_, key);
            printf("+ %s: %s\n", key.c_str(), value.c_str());

            std::string type = cut_first_word(value, ':');
            if (type == "bluetooth")
                modules[key] = new Bluetooth(key, value, globalMessageHandler);
            else if (type == "led")
                modules[key] = new Led(key, new Port((gpio_num_t)atoi(value.c_str())));
            else if (type == "button")
                modules[key] = new Button(key, new Port((gpio_num_t)atoi(value.c_str())));
            else
                printf("Unknown module type: %s\n", type.c_str());
        };

        printf("Reading settings from persistent storage...\n");
        it = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY);
        while (it != NULL)
        {
            nvs_entry_info(it, &info);
            it = nvs_entry_next(it);

            std::string namespace_ = std::string(info.namespace_name);
            if (namespace_ == "configure")
                continue;

            // TODO: DEBUG HERE
            // std::string key = std::string(info.key);
            // std::string value = storage::get(namespace_, key);
            // printf("+ Setting %s.%s=%s\n", namespace_.c_str(), key.c_str(), value.c_str());
            // modules[namespace_]->handleMsg("set " + key + "=" + value);
        };
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "new")
        {
            std::string type = cut_first_word(msg);
            std::string name = cut_first_word(msg);
            if (name.empty())
            {
                printf("A name is required.\n");
                return;
            }
            storage::put("configure", name, type + ':' + msg);
        }
        else if (command == "has")
        {
            printf("configure has %s %d\n", msg.c_str(), modules.count(msg) > 0 ? 1 : 0);
        }
        else if (command == "preset")
        {
            std::string name = cut_first_word(msg);
            std::string key = cut_first_word(msg, '=');
            storage::put(name, key, msg);
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