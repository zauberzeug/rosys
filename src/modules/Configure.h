#pragma once

#include <string>
#include <map>

#include "utils/strings.h"
#include "Module.h"
#include "Led.h"
#include "Button.h"
#include "../Port.h"
#include "../storage.h"

extern std::map<std::string, Module *> modules;

class Configure : public Module
{
public:
    Configure()
    {
        printf("Reading configuration from persistent storage...\n");

        nvs_iterator_t it = nvs_entry_find("nvs", NULL, NVS_TYPE_ANY);
        nvs_entry_info_t info;
        while (it != NULL)
        {
            nvs_entry_info(it, &info);
            it = nvs_entry_next(it);

            std::string namespace_ = std::string(info.namespace_name);
            std::string key = std::string(info.key);
            std::string value = storage::get(namespace_, key);
            if (namespace_ == "configure")
            {
                printf("+ Module %s: %s\n", key.c_str(), value.c_str());
                std::string type = cut_first_word(value, ':');
                if (type == "led")
                {
                    modules[key] = new Led(new Port((gpio_num_t)atoi(value.c_str())));
                }
                else if (type == "button")
                {
                    modules[key] = new Button(new Port((gpio_num_t)atoi(value.c_str())));
                }
                else
                {
                    printf("Unknown module type: %s\n", type.c_str());
                    continue;
                }
            }
            else
            {
                printf("+ Setting %s.%s=%s\n", namespace_.c_str(), key.c_str(), value.c_str());
                modules[namespace_]->handleMsg("set " + key + "=" + value);
            }
        };
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "new")
        {
            std::string type = cut_first_word(msg);
            std::string name = cut_first_word(msg);
            storage::put("configure", name, type + ':' + msg);
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