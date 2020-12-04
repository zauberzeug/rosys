#pragma once

#include <string>

class Module
{
public:
    std::string name;
    int state = 0;

    Module(std::string name)
    {
        this->name = name;
    }

    virtual void setup(){};

    virtual void loop(){};

    virtual void handleMsg(std::string msg) = 0;

    virtual void set(std::string key, std::string value){};

    virtual void stop(){};
};
