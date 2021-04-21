#pragma once

#include <string>
#include "driver/gpio.h"

class Port
{
public:
    static Port *fromString(std::string string);

    virtual void setup(bool input) = 0;

    virtual void set_pull(int pull) = 0;

    virtual void set_level(int level) = 0;

    virtual int get_level() = 0;
};
