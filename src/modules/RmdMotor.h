#pragma once

#include <string>
#include <math.h>
#include "driver/gpio.h"
#include "esp_timer.h"

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../modules/Can.h"

class RmdMotor : public Module
{
private:
    bool output = false;
    const double ratio = 6.0;

    Can *can;

public:
    RmdMotor(std::string name, Can *can) : Module(name)
    {
        this->can = can;
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "turn")
        {
            uint16_t speed = atof(cut_first_word(msg, ',').c_str()) * ratio;
            uint32_t angle = atof(cut_first_word(msg, ',').c_str()) * ratio * 100;
            printf("speed %d, angle %d\n", speed, angle);
            this->can->send(0x141, 0xa4, 0,
                            *((uint8_t *)(&speed) + 0),
                            *((uint8_t *)(&speed) + 1),
                            *((uint8_t *)(&angle) + 0),
                            *((uint8_t *)(&angle) + 1),
                            *((uint8_t *)(&angle) + 2),
                            *((uint8_t *)(&angle) + 3));
        }
        else if (command == "zero")
        {
            this->can->send(0x141, 0x19, 0, 0, 0, 0, 0, 0, 0);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
