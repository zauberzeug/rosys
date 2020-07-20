#include <map>
#include "esp_system.h"

#include "modules/Module.h"
#include "modules/Esp.h"
#include "modules/Led.h"
#include "configuration.h"
#include "serial.h"
#include "simple_arduino.h"

std::map<std::string, Module *> modules;

void handleMsg(std::string msg)
{
    int space = msg.find(' ');
    std::string module_name = space < 0 ? msg : msg.substr(0, space);
    std::string remainder = space < 0 ? std::string() : msg.substr(space + 1);

    if (module_name == "configure")
    {
        Module *module = configuration::create(remainder);
        modules[module->name] = module;
    }
    else if (modules.count(module_name))
    {
        modules[module_name]->handleMsg(remainder);
    }
    else
    {
        printf("Unknown module_name: %s\n", module_name.c_str());
    }
}

void setup()
{
    serial::begin(115200);

    modules["esp"] = new Esp();
    modules["led"] = new Led(new Port(GPIO_NUM_13));

    for (auto const &item : modules)
    {
        item.second->setup();
    }
}

void loop()
{
    while (true)
    {
        std::string line = serial::readStringUntil('\n');
        if (line.empty())
            break;

        handleMsg(line);
    }

    for (auto const &item : modules)
    {
        item.second->loop();
    }

    delay(10);
}

extern "C"
{
    void app_main();
}
void app_main()
{
    setup();

    while (true)
    {
        loop();
    }
}
