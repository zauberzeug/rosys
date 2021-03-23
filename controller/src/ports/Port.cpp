#include "Port.h"

#include "GpioPort.h"
#include "McpPort.h"
#include "../utils/strings.h"

Port *Port::fromString(std::string string)
{
    if (starts_with(string, "MCP_"))
    {
        int bank = starts_with(string, "MCP_A") ? 0 : 1;
        int number = atoi(string.substr(5).c_str());
        return new McpPort(bank, number);
    }
    else if (string.empty())
    {
        return nullptr;
    }
    else
    {
        gpio_num_t number = (gpio_num_t)atoi(string.c_str());
        return new GpioPort(number);
    }
}