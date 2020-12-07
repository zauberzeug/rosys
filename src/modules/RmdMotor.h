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
    bool output = false;
    double ratio = 1.0;
    double speed = 10.0;
    double minAngle = 0.0;
    double maxAngle = 360.0;

    const uint16_t id = 0x141;

    Can *can;

    double target = 0;

    enum State
    {
        IDLE = 0,
        MOVE = 1,
        HOME = 2,
        STOP = 3,
    };

public:
    RmdMotor(std::string name, Can *can) : Module(name)
    {
        this->can = can;
        this->can->subscribe(this->id, this);
    }

    void loop()
    {
        if (output)
            printf("%s %d\n", this->name.c_str(), this->state);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "move")
        {
            this->target = atof(cut_first_word(msg, ',').c_str());
            this->move();
        }
        else if (command == "home")
        {
            this->target = 0;
            this->move();
        }
        else if (command == "zero")
        {
            uint8_t data[8] = {0x19, 0, 0, 0, 0, 0, 0, 0};
            this->can->send(this->id, data);
        }
        else if (command == "stop")
        {
            this->stop();
        }
        else if (command == "get")
        {
            printf("%s get %d\n", this->name.c_str(), this->state);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }

    void handleCanMsg(uint16_t id, uint8_t data[8])
    {
        // TODO
    }

    void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
        }
        else if (key == "ratio")
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

    void move()
    {
        double clipped_angle = std::max(std::min(this->target, maxAngle), minAngle);
        uint16_t speed = this->speed * this->ratio;
        uint32_t angle = clipped_angle * this->ratio * 100;
        uint8_t data[8] = {
            0xa4,
            0,
            *((uint8_t *)(&speed) + 0),
            *((uint8_t *)(&speed) + 1),
            *((uint8_t *)(&angle) + 0),
            *((uint8_t *)(&angle) + 1),
            *((uint8_t *)(&angle) + 2),
            *((uint8_t *)(&angle) + 3),
        };
        this->can->send(this->id, data);
        this->state = MOVE;
    }

    void stop()
    {
        uint8_t data[8] = {0x81, 0, 0, 0, 0, 0, 0, 0};
        this->can->send(this->id, data);
        this->state = STOP;
    }
};
