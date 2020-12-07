#pragma once

#include <string>
#include <math.h>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../modules/Can.h"

class RmdMotor : public Module
{
private:
    double ratio = 1.0;
    double speed = 10.0;
    double minAngle = 0.0;
    double maxAngle = 360.0;

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
            double requested_angle = atof(cut_first_word(msg, ',').c_str());
            double clipped_angle = std::max(std::min(requested_angle, maxAngle), minAngle);
            uint16_t speed = this->speed * this->ratio;
            uint32_t angle = clipped_angle * this->ratio * 100;
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
        else if (command == "stop")
        {
            this->stop();
        }
        else if (command == "off")
        {
            this->can->send(0x141, 0x80, 0, 0, 0, 0, 0, 0, 0);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "ratio")
        {
            ratio = atof(value.c_str());
        }
        else if (key == "speed")
        {
            speed = atof(value.c_str());
        }
        else if (key == "minAngle")
        {
            minAngle = atof(value.c_str());
        }
        else if (key == "maxAngle")
        {
            maxAngle = atof(value.c_str());
        }
        else
        {
            printf("Unknown setting: %s\n", key.c_str());
        }
    }

    void stop()
    {
        this->can->send(0x141, 0x81, 0, 0, 0, 0, 0, 0, 0);
    }
};
