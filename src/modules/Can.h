#pragma once

#include <string>
#include "driver/can.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"

class Can : public Module
{
private:
    bool output = false;

public:
    Can(std::string name, std::string parameters) : Module(name)
    {
        gpio_num_t rx = (gpio_num_t)atoi(cut_first_word(parameters, ',').c_str());
        gpio_num_t tx = (gpio_num_t)atoi(cut_first_word(parameters, ',').c_str());

        can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(tx, rx, CAN_MODE_NORMAL);
        can_timing_config_t t_config = CAN_TIMING_CONFIG_1MBITS();
        can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

        ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
        ESP_ERROR_CHECK(can_start());
    }

    void loop()
    {
        can_message_t message;
        while (can_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK)
        {
            printf("ID is %d\n", message.identifier);
            if (!(message.flags & CAN_MSG_FLAG_RTR))
            {
                for (int i = 0; i < message.data_length_code; i++)
                {
                    printf("Data byte %d = %d\n", i, message.data[i]);
                }
            }
        }
    }

    void send(uint16_t id, uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7)
    {
        can_message_t message;
        message.identifier = id;
        message.flags = CAN_MSG_FLAG_NONE;
        message.data_length_code = 8;
        message.data[0] = d0;
        message.data[1] = d1;
        message.data[2] = d2;
        message.data[3] = d3;
        message.data[4] = d4;
        message.data[5] = d5;
        message.data[6] = d6;
        message.data[7] = d7;
        if (can_transmit(&message, pdMS_TO_TICKS(0)) != ESP_OK)
        {
            printf("Could not send message\n");
        }
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
            {
                output = msg == "1";
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else if (command == "send")
        {
            send(std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16),
                 std::stoi(cut_first_word(msg, ','), nullptr, 16));
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
