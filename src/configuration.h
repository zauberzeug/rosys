#pragma once

#include "nvs_flash.h"

#include "modules/Module.h"
#include "modules/Led.h"
#include "Port.h"

namespace configuration
{

    void erase()
    {
        nvs_flash_erase();
    }

    Module *create(std::string msg)
    {
        int space = msg.find(' ');
        if (space < 0)
            space = msg.length();
        std::string module_name = msg.substr(0, space);
        std::string values = msg.substr(space + 1);

        Module *module;
        if (module_name.substr(0, 3) == "led")
        {
            module = new Led(new Port((gpio_num_t)atoi(values.c_str())));
        }
        else
        {
            printf("Unknown module: %s\n", module_name.c_str());
            return NULL;
        }
        module->name = module_name;

        return module;
    }

}