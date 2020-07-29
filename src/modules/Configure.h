#pragma once

#include <string>
#include <map>

#include "../utils/strings.h"
#include "../storage.h"
#include "Module.h"
#include "Bluetooth.h"
#include "Led.h"
#include "Button.h"
#include "Bumper.h"
#include "Drive.h"

class Configure : public Module
{
private:
    const char *NAMESPACE = "MODULES";
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
        it = nvs_entry_find("nvs", NAMESPACE, NVS_TYPE_ANY);
        while (it != NULL)
        {
            nvs_entry_info(it, &info);
            it = nvs_entry_next(it);

            std::string name = std::string(info.key);
            std::string lines = storage::get(NAMESPACE, name);
            std::string line = cut_first_word(lines, '\n');
            std::string type = cut_first_word(line, ':');
            printf("+ %s: %s(%s)\n", name.c_str(), type.c_str(), line.c_str());
            if (type == "bluetooth")
                modules[name] = new Bluetooth(name, line, globalMessageHandler);
            else if (type == "led")
                modules[name] = new Led(name, line);
            else if (type == "button")
                modules[name] = new Button(name, line);
            else if (type == "bumper")
                modules[name] = new Bumper(name, line);
            else if (type == "drive")
                modules[name] = new Drive(name, line);
            else
                printf("Unknown module type: %s\n", type.c_str());

            while (true)
            {
                line = cut_first_word(lines, '\n');
                if (line.empty())
                    break;

                std::string key = cut_first_word(line, '=');
                printf("  - %s=%s\n", key.c_str(), line.c_str());
                modules[name]->handleMsg("set " + key + "=" + line);
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
            if (name.empty())
            {
                printf("A name is required.\n");
                return;
            }
            storage::put(NAMESPACE, name, type + ':' + msg + '\n');
        }
        else if (command == "has")
        {
            printf("configure has %s %d\n", msg.c_str(), modules.count(msg) > 0 ? 1 : 0);
        }
        else if (command == "print")
        {
            storage::print(NAMESPACE, msg);
        }
        else if (command == "preset")
        {
            std::string name = cut_first_word(msg);
            std::string key = cut_first_word(msg, '=');
            std::string value = msg;
            std::string configuration = storage::get(NAMESPACE, name);

            while (true)
            {
                int start_pos = configuration.find('\n' + key + '=');
                if (start_pos == std::string::npos)
                    break;

                int end_pos = configuration.find('\n', start_pos + 1);
                configuration.replace(start_pos, end_pos - start_pos + 1, "\n");
            }

            configuration += key + '=' + value + '\n';

            storage::put(NAMESPACE, name, configuration);
        }
        else if (command == "erase")
        {
            storage::erase(NAMESPACE, msg);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};