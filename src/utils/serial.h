#pragma once

#include <string>
#include "driver/uart.h"

class Serial
{
private:
    const int BUFFER_SIZE = 1024;

public:
    Serial(int baud_rate)
    {
        uart_config_t uart_config = {
            .baud_rate = baud_rate,
            .data_bits = UART_DATA_8_BITS,
            .parity = UART_PARITY_DISABLE,
            .stop_bits = UART_STOP_BITS_1,
            .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
            .rx_flow_ctrl_thresh = 0,
            .use_ref_tick = false,
        };
        uart_param_config(UART_NUM_0, &uart_config);
        uart_driver_install(UART_NUM_0, BUFFER_SIZE * 2, 0, 0, NULL, 0);
    }

    uint8_t read()
    {
        uint8_t data = 0;
        uart_read_bytes(UART_NUM_0, &data, 1, 0);
        return data;
    }

    std::string readStringUntil(char delimiter)
    {
        static uint8_t *buffer = (uint8_t *)malloc(BUFFER_SIZE);
        static int offset = 0;

        uint8_t c = read();
        while (c)
        {
            if (c == delimiter)
            {
                std::string result((const char *)buffer, offset);
                offset = 0;
                return result;
            }
            buffer[offset++] = c;
            c = read();
        }
        return std::string();
    }
};