#include "mcp.h"

#include "mcp23017.h"

mcp23017_t config;
uint8_t value_A = 0;
uint8_t value_B = 0;

void mcp::init()
{
    config ={
        .i2c_addr = 0x20,
        .port = I2C_NUM_0,
        .sda_pin = 21,
        .scl_pin = 22,
    };

    gpio_reset_pin(GPIO_NUM_14);
    gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_14, 1);

    ESP_ERROR_CHECK(mcp23017_init(&config));
}

void mcp::write_bit(int bank, int number, int level)
{
    uint8_t new_value;
    if (bank == 0) {
        if (level == 0)
            new_value = value_A & ~(1 << number);
        else
            new_value = value_A | (1 << number);
        if (new_value == value_A)
            return;
        value_A = new_value;
        mcp23017_write_register(&config, MCP23017_GPIO, GPIOA, value_A);
    }
    else {
        if (level == 0)
            new_value = value_B & ~(1 << number);
        else
            new_value = value_B | (1 << number);
        if (new_value == value_B)
            return;
        value_B = new_value;
        mcp23017_write_register(&config, MCP23017_GPIO, GPIOB, value_B);
    }
}