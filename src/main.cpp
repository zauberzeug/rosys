#include <map>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "storage.h"
#include "mcp.h"
#include "modules/Module.h"
#include "modules/Esp.h"
#include "modules/Configure.h"
#include "utils/Serial.h"
#include "utils/timing.h"
#include "utils/strings.h"

#define EN_24V GPIO_NUM_12

std::map<std::string, Module *> modules;

Serial *serial;

void handleMsg(std::string msg)
{
    std::string module_name = cut_first_word(msg);
    if (modules.count(module_name))
        modules[module_name]->handleMsg(msg);
    else if (module_name == "stop")
        for (auto const &item : modules)
            item.second->stop();
    else if (module_name == "left") // DEPRICATED
        handleMsg(std::string("drive left ") + msg);
    else if (module_name == "right") // DEPRICATED
        handleMsg(std::string("drive right ") + msg);
    else if (module_name == "pw") // DEPRICATED
        handleMsg(std::string("drive pw ") + msg);
    else if (module_name == "ros") // DEPRICATED
        handleMsg(std::string("esp print ") + msg);
    else
        printf("Unknown module name: %s\n", module_name.c_str());
}

void setup()
{
    serial = new Serial(115200);
    delay(500);

    mcp::init();

    storage::init();

    modules["esp"] = new Esp();
    modules["configure"] = new Configure(modules, handleMsg);

    for (auto const &item : modules)
        item.second->setup();

    gpio_reset_pin(EN_24V);
    gpio_set_direction(EN_24V, GPIO_MODE_OUTPUT);
    gpio_set_level(EN_24V, 1);
}

void loop()
{
    while (true)
    {
        std::string line = serial->readStringUntil('\n');
        if (line.empty())
            break;
        handleMsg(line);
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
