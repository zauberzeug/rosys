#include "mcp.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <driver/gpio.h>
#include <mcp23x17.h>

mcp23x17_t config = {};

void mcp::init()
{
    gpio_reset_pin(GPIO_NUM_14);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_14, 1);

    i2cdev_init();
    mcp23x17_init_desc(&config, (i2c_port_t)0, MCP23X17_ADDR_BASE, GPIO_NUM_21, GPIO_NUM_22);
    config.cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    config.cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
}

void mcp::set_mode(int bank, int number, bool input)
{
    mcp23x17_set_mode(&config, 8 * bank + number, input ? MCP23X17_GPIO_INPUT : MCP23X17_GPIO_OUTPUT);
}

void mcp::set_pullup(int bank, int number, bool value)
{
    mcp23x17_set_pullup(&config, 8 * bank + number, value);
}

void mcp::write_bit(int bank, int number, int level)
{
    mcp23x17_set_level(&config, 8 * bank + number, level > 0);
}

int mcp::read_bit(int bank, int number)
{
    uint32_t value;
    mcp23x17_get_level(&config, 8 * bank + number, &value);
    return value;
}
