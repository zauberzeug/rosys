#pragma once

#include <string>

#include "utils.h"
#include "Module.h"
#include "Led.h"
#include "../Port.h"
#include "../storage.h"

class Configure : public Module
{
    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "new")
        {
            std::string type = cut_first_word(msg);
            storage::put("configure", type, msg);
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
    }

    Module *create(std::string msg)
    {
        std::string module_name = cut_first_word(msg);

        Module *module;
        if (module_name.substr(0, 3) == "led")
        {
            module = new Led(new Port((gpio_num_t)atoi(msg.c_str())));
        }
        else
        {
            printf("Unknown module: %s\n", module_name.c_str());
            return NULL;
        }
        module->name = module_name;

        return module;
    }
};