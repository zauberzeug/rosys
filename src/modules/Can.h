#pragma once

#include <string>
#include "driver/can.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Can : public Module
{
private:
    bool output = false;

    std::map<uint16_t, Module *> subscribers;

public:
    Can(std::string name, std::string parameters) : Module(name)
    {
        gpio_num_t rx = (gpio_num_t)atoi(cut_first_word(parameters, ',').c_str());
        gpio_num_t tx = (gpio_num_t)atoi(cut_first_word(parameters, ',').c_str());

        can_general_config_t g_config = CAN_GENERAL_CONFIG_DEFAULT(tx, rx, CAN_MODE_NORMAL);
        can_timing_config_t t_config = CAN_TIMING_CONFIG_1MBITS();
        can_filter_config_t f_config = CAN_FILTER_CONFIG_ACCEPT_ALL();

        g_config.rx_queue_len = 20;

        ESP_ERROR_CHECK(can_driver_install(&g_config, &t_config, &f_config));
        ESP_ERROR_CHECK(can_start());
    }

    void loop()
    {
        can_message_t message;
        while (can_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK)
        {
            if (subscribers.count(message.identifier))
            {
                subscribers[message.identifier]->handleCanMsg(message.identifier, message.data);
            }

            if (output)
            {
                cprint("can %03x", message.identifier);
                if (!(message.flags & CAN_MSG_FLAG_RTR))
                {
                    for (int i = 0; i < message.data_length_code; ++i)
                    {
                        cprint(",%02x", message.data[i]);
                    }
                }
                cprintln("");
            }
        }
    }

    void send(uint16_t id, uint8_t data[8], bool rtr=false)
    {
        can_message_t message;
        message.identifier = id;
        message.flags = rtr? CAN_MSG_FLAG_RTR : CAN_MSG_FLAG_NONE;
        message.data_length_code = 8;
        for (int i = 0; i < 8; ++i)
        {
            message.data[i] = data[i];
        }
        if (can_transmit(&message, pdMS_TO_TICKS(0)) != ESP_OK)
        {
            cprintln("Could not send message with ID 0x%03x", id);
        }
    }

    void send(uint16_t id,
        uint8_t d0, uint8_t d1, uint8_t d2, uint8_t d3,
        uint8_t d4, uint8_t d5, uint8_t d6, uint8_t d7,
        bool rtr=false)
    {
        uint8_t data[8] = { d0, d1, d2, d3, d4, d5, d6, d7 };
        this->send(id, data, rtr);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "send" or command == "request")
        {
            uint16_t id = std::stoi(cut_first_word(msg, ','), nullptr, 16);
            uint8_t data[8];
            for (int i = 0; i < 8; ++i)
            {
                std::string word = cut_first_word(msg, ',');
                data[i] = word.empty() ? 0 : std::stoi(word, nullptr, 16);
            }
            bool rtr = command=="request";
            send(id, data, rtr);
        }
        else
        {
            cprintln("Unknown command: %s", command.c_str());
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
        }
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
        }
    }

    void subscribe(uint16_t id, Module *module)
    {
        this->subscribers[id] = module;
    }
};
