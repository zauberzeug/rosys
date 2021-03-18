#pragma once

#include "driver/gpio.h"

#include "Port.h"
#include "../mcp.h"
#include "../utils/checksum.h"

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

    void setup(bool input, int pull = 0)
    {
        mcp::set_mode(this->bank, this->number, input);
        if (pull < 0)
            cprintln("Error: Pull-down is not supported.");
        mcp::set_pullup(this->bank, this->number, pull > 0);
    }

    void set_level(int level)
    {
        mcp::write_bit(this->bank, this->number, level);
    }

    int get_level()
    {
        return mcp::read_bit(this->bank, this->number);
    }
};