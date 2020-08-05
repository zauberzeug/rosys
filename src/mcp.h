#pragma once

#include "mcp23017.h"

namespace mcp {

    mcp23017_t config = {
        .i2c_addr = 0x20,
        .port = I2C_NUM_0,
        .sda_pin = 21,
        .scl_pin = 22,
    };

    void init() {
        gpio_reset_pin(GPIO_NUM_14);
        gpio_set_direction(GPIO_NUM_14, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_14, 1);

        ESP_ERROR_CHECK(mcp23017_init(&config));
    }

}