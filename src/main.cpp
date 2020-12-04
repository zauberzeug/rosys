#include <map>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "storage.h"
#include "mcp.h"
#include "modules/Module.h"
#include "modules/Esp.h"
#include "modules/Safety.h"
#include "modules/Bluetooth.h"
#include "modules/Led.h"
#include "modules/Button.h"
#include "modules/Drive.h"
#include "modules/Motor.h"
#include "modules/DualMotor.h"
#include "modules/Imu.h"
#include "modules/Can.h"
#include "modules/RmdMotor.h"
#include "utils/Serial.h"
#include "utils/timing.h"
#include "utils/strings.h"

#define EN_24V GPIO_NUM_12

std::map<std::string, Module *> modules;

Serial *serial;

Esp *esp;
Safety *safety;

Module *createModule(std::string type, std::string name, std::string parameters);

void handleMsg(std::string msg)
{
    if (msg[0] == '+')
    {
        storage::append(msg.substr(1));
        return;
    }
    if (msg[0] == '-')
    {
        storage::remove(msg.substr(1));
        return;
    }
    if (msg[0] == '?')
    {
        storage::print(msg.substr(1));
        return;
    }

    std::string word = cut_first_word(msg);
    if (word == "new")
    {
        std::string type = cut_first_word(msg);
        std::string name = cut_first_word(msg);
        modules[name] = createModule(type, name, msg);
        modules[name]->setup();
    }
    else if (word == "set")
    {
        std::string name = cut_first_word(msg, '.');
        std::string key = cut_first_word(msg, '=');
        modules[name]->set(key, msg);
    }
    else if (modules.count(word))
    {
        modules[word]->handleMsg(msg);
    }
    else
    {
        // DEPRICATED
        if (word == "stop")
            esp->stopAll();
        else if (word == "left")
            handleMsg(std::string("drive left ") + msg);
        else if (word == "right")
            handleMsg(std::string("drive right ") + msg);
        else if (word == "pw")
            handleMsg(std::string("drive pw ") + msg);
        else if (word == "ros")
            handleMsg(std::string("esp print ") + msg);
        else
            printf("Unknown module name: %s\n", word.c_str());
    }
}

void setup()
{
    serial = new Serial(38400);
    delay(500);

    mcp::init(); // TODO

    storage::init();

    modules["esp"] = esp = new Esp(&modules);
    modules["safety"] = safety = new Safety(&modules);

    printf("Reading configuration...\n");
    std::string content = storage::get();
    while (!content.empty())
    {
        std::string line = cut_first_word(content, '\n');
        printf(">> %s\n", line.c_str());
        handleMsg(line);
    }

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
    {
        if (!safety->check(item.second))
            item.second->stop();
        item.second->loop();
    }

    delay(10);
}

Module *createModule(std::string type, std::string name, std::string parameters)
{
    if (type == "bluetooth")
        return new Bluetooth(name, parameters, handleMsg);
    else if (type == "led")
        return new Led(name, parameters);
    else if (type == "button")
        return new Button(name, parameters);
    else if (type == "drive")
        return new Drive(name, parameters);
    else if (type == "motor")
        return new Motor(name, parameters);
    else if (type == "dualmotor")
        return new DualMotor(name, parameters);
    else if (type == "imu")
        return new Imu(name);
    else if (type == "can")
        return new Can(name, parameters);
    else if (type == "rmdmotor")
        return new RmdMotor(name, (Can *)modules[parameters]);
    else
    {
        printf("Unknown module type: %s\n", type.c_str());
        return NULL;
    }
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
