#include <map>
#include "esp_system.h"

#include "modules/Module.h"
#include "modules/Esp.h"
#include "modules/Configure.h"
#include "modules/Led.h"
#include "serial.h"
#include "utils.h"

std::map<std::string, Module *> modules;

void setup()
{
    serial::begin(115200);

    storage::init();

    modules["esp"] = new Esp();
    modules["configure"] = new Configure();
    modules["led"] = new Led(new Port(GPIO_NUM_13));

    for (auto const &item : modules)
        item.second->setup();
}

void loop()
{
    while (true)
    {
        std::string line = serial::readStringUntil('\n');
        if (line.empty())
            break;

        std::string module_name = cut_first_word(line);
        if (modules.count(module_name))
            modules[module_name]->handleMsg(line);
        else
            printf("Unknown module_name: %s\n", module_name.c_str());
    }

    for (auto const &item : modules)
        item.second->loop();

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
        loop();
}
