#include <map>
#include "esp_system.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include "storage.h"
#include "mcp.h"
#include "modules/Bluetooth.h"
#include "modules/Button.h"
#include "modules/Can.h"
#include "modules/DcMotor.h"
#include "modules/Esp.h"
#include "modules/Imu.h"
#include "modules/Led.h"
#include "modules/LinearMotor.h"
#include "modules/Module.h"
#include "modules/ODriveMotor.h"
#include "modules/ODriveWheels.h"
#include "modules/RmdMotor.h"
#include "modules/RoboClawMotors.h"
#include "modules/RoboClawWheels.h"
#include "modules/Safety.h"
#include "utils/checksum.h"
#include "utils/timing.h"
#include "utils/Serial.h"
#include "utils/strings.h"

std::map<std::string, Module *> modules;

Serial *serial;

Esp *esp;
Safety *safety;

Module *createModule(std::string type, std::string name, std::string parameters);
std::string check(std::string line);

void handleMsg(std::string multiMsg)
{
    if (multiMsg[0] == '+')
    {
        storage::append(multiMsg.substr(1));
        return;
    }
    if (multiMsg[0] == '-')
    {
        storage::remove(multiMsg.substr(1));
        return;
    }
    if (multiMsg[0] == '?')
    {
        storage::print(multiMsg.substr(1));
        return;
    }

    while (not multiMsg.empty())
    {
        std::string msg = cut_first_word(multiMsg, ';');
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
        else if (word == "if")
        {
            safety->addCondition(std::string("if ") + msg + ";" + multiMsg);
        }
        else if (word == "shadow")
        {
            safety->addShadow(msg);
        }
        else if (modules.count(word))
        {
            std::string command = cut_first_word(msg);
            modules[word]->handleMsg(command, msg);
            safety->applyShadow(word, command, msg);
        }
        else if (word == "stop")
        {
            esp->stopAll();
        }
        else if (word == "pw") // DEPRICATED
        {
            handleMsg("wheels " + word + " " + msg);
        }
        else
        {
            cprintln("Unknown module name: %s", word.c_str());
        }
    }
}

void handleBluetoothMsg(std::string multiMsg)
{
    handleMsg(multiMsg);
    safety->keep_alive();
}

void setup()
{
    serial = new Serial(115200);
    delay(500);

    mcp::init();

    storage::init();

    modules["esp"] = esp = new Esp(&modules);
    modules["safety"] = safety = new Safety(&modules, handleMsg);

    esp->setup();
    safety->setup();

    std::string boot = storage::read("TEMP", "BOOT");
    int boot_count = boot == "" ? 1 : stoi(boot) + 1;
    storage::write("TEMP", "BOOT", std::to_string(boot_count));
    if (boot_count > 3)
    {
        cprintln("Boot loop detected. Ignoring configuration this time.");
    }
    else
    {
        cprintln("Reading configuration...");
        std::string content = storage::get();
        while (!content.empty())
        {
            std::string line = cut_first_word(content, '\n');
            cprintln(">> %s", line.c_str());
            handleMsg(line);
        }
    }
    storage::write("TEMP", "BOOT", "");
}

void loop()
{
    while (true)
    {
        std::string line = serial->readStringUntil('\n');
        if (line.empty())
            break;
        std::string msg = check(line);
        if (msg.empty())
            continue;
        handleMsg(msg);
        safety->keep_alive();
    }

    safety->applyConditions();

    for (auto const &item : modules)
    {
        if (!safety->is_alive()) {
            item.second->stop();
        }
        item.second->loop();
    }

    delay(10);
}

Module *createModule(std::string type, std::string name, std::string parameters)
{
    if (type == "bluetooth")
        return new Bluetooth(name, parameters, handleBluetoothMsg);
    else if (type == "button")
        return new Button(name, parameters);
    else if (type == "can")
        return new Can(name, parameters);
    else if (type == "dcmotor")
        return new DcMotor(name, parameters);
    else if (type == "imu")
        return new Imu(name);
    else if (type == "led")
        return new Led(name, parameters);
    else if (type == "linearmotor")
        return new LinearMotor(name, parameters);
    else if (type == "odrivemotor")
    {
        std::string switch_name = cut_first_word(parameters, ',');
        std::string can_name = cut_first_word(parameters, ',');
        Button *home_switch = switch_name == "0" ? nullptr : (Button *)modules[switch_name];
        return new ODriveMotor(name, home_switch, (Can *)modules[can_name], parameters);
    }
    else if (type == "odrivewheels")
    {
        ODriveMotor *left = (ODriveMotor *)modules[cut_first_word(parameters, ',').c_str()];
        ODriveMotor *right = (ODriveMotor *)modules[cut_first_word(parameters, ',').c_str()];
        return new ODriveWheels(name, left, right);
    }
    else if (type == "rmdmotor")
        return new RmdMotor(name, (Can *)modules[parameters]);
    else if (type == "roboclawmotors")
        return new RoboClawMotors(name, parameters);
    else if (type == "roboclawwheels")
        return new RoboClawWheels(name, parameters);
    else
    {
        cprintln("Unknown module type: %s", type.c_str());
        return nullptr;
    }
}

std::string check(std::string line)
{
    std::string msg = cut_first_word(line, '^');
    if (line.empty())
        return msg;

    unsigned int checksum = 0;
    for (int i = 0; i < msg.size(); ++i)
        checksum ^= msg[i];
    if (checksum != atoi(line.c_str()))
    {
        cprintln("Warning: Checksum mismatch for \"%s^%s\" (%d expected)",
                 msg.c_str(), line.c_str(), checksum);
        return "";
    }

    return msg;
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
