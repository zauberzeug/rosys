#include <map>
#include "esp_system.h"
#include "nvs_flash.h"

#include "modules/Module.h"
#include "modules/Esp.h"
#include "modules/Configure.h"
#include "modules/Led.h"
#include "utils/serial.h"
#include "utils/delay.h"
#include "utils/strings.h"

std::map<std::string, Module *> modules;

void setup()
{
    serial::begin(115200);
    delay(500);

    storage::init();

    modules["esp"] = new Esp();
    modules["configure"] = new Configure();

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
            printf("Unknown module name: %s\n", module_name.c_str());
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
