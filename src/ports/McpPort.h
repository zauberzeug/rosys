#pragma once

#include "driver/gpio.h"

#include "Port.h"
#include "../mcp.h"
#include "../mcp23017.h"

class McpPort : public Port
{
private:
    int number;
    int bank;

public:
    McpPort(int bank, int number)
    {
        this->number = number;
        this->bank = bank;
    }

    void setup(bool input, int pull=0)
    {
    }

    void set_level(int level)
    {
        mcp::write_bit(this->bank, this->number, level);
    }

    int get_level()
    {
        return 0;
    }
};