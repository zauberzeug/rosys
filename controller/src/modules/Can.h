#pragma once

#include <string>
#include "driver/twai.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class Can : public Module
{
private:
    std::map<uint16_t, Module *> subscribers;

public:
    Can(std::string name, std::string parameters) : Module(name)
    {
        gpio_num_t rx = (gpio_num_t)atoi(cut_first_word(parameters, ',').c_str());
        gpio_num_t tx = (gpio_num_t)atoi(cut_first_word(parameters, ',').c_str());

        twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx, rx, TWAI_MODE_NORMAL);
        twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
        twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

        g_config.rx_queue_len = 20;
        g_config.tx_queue_len = 20;

        ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
        ESP_ERROR_CHECK(twai_start());
    }

    void loop()
    {
        twai_message_t message;
        while (twai_receive(&message, pdMS_TO_TICKS(0)) == ESP_OK)
        {
            if (subscribers.count(message.identifier))
            {
                subscribers[message.identifier]->handleCanMsg(message.identifier, message.data);
            }

            if (output)
            {
                cprint("can %03x", message.identifier);
                if (!(message.flags & TWAI_MSG_FLAG_RTR))
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
        twai_message_t message;
        message.identifier = id;
        message.flags = rtr? TWAI_MSG_FLAG_RTR : TWAI_MSG_FLAG_NONE;
        message.data_length_code = 8;
        for (int i = 0; i < 8; ++i)
        {
            message.data[i] = data[i];
        }
        if (twai_transmit(&message, pdMS_TO_TICKS(0)) != ESP_OK)
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

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "send" or command == "request")
        {
            uint16_t id = std::stoi(cut_first_word(parameters, ','), nullptr, 16);
            uint8_t data[8];
            for (int i = 0; i < 8; ++i)
            {
                std::string word = cut_first_word(parameters, ',');
                data[i] = word.empty() ? 0 : std::stoi(word, nullptr, 16);
            }
            bool rtr = command == "request";
            send(id, data, rtr);
        }
        else
        {
            Module::handleMsg(command, parameters);
        }
    }

    void subscribe(uint16_t id, Module *module)
    {
        this->subscribers[id] = module;
    }
};
