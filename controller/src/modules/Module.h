#pragma once

#include <string>

class Module
{
public:
    std::string name;
    int state = 0;
    bool output = false;

    Module(std::string name)
    {
        this->name = name;
    }

    virtual void setup()
    {
    }

    virtual void loop()
    {
        if (this->output) {
            std::string str = this->getOutput();
            if (not str.empty()) {
                cprintln("%s %s", this->name.c_str(), str.c_str());
            }
        }
    }

    virtual void handleMsg(std::string command, std::string parameters)
    {
        if (command == "get")
        {
            cprintln("%s get %s", this->name.c_str(), this->getOutput().c_str());
        }
        else
        {
            cprintln("Unknown command: %s", command.c_str());
        }
    }

    virtual void handleCanMsg(uint16_t id, uint8_t d[8])
    {
    }

    virtual void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
        }
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
        }
    }

    virtual std::string getOutput()
    {
        return "";
    }

    virtual void stop()
    {
    }
};
