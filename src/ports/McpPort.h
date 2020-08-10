#pragma once

#include "driver/gpio.h"

#include "Port.h"
#include "../mcp.h"

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
        if (this->bank == 0 && not input)
            printf("Error: Bank A is input only.\n");
        if (this->bank == 1 && input)
            printf("Error: Bank B is output only.\n");
        if (pull < 0)
            printf("Error: Pull-down is not supported.\n");
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