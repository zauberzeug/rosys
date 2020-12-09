#pragma once

#include <string>
#include <math.h>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../modules/Can.h"
#include "../utils/checksum.h"

class RmdMotor : public Module
{
private:
    bool output = false;
    double ratio = 1.0;
    double speed = 10.0;
    double minAngle = 0.0;
    double maxAngle = 360.0;
    double tolerance = 0.1;

    Can *can;
    const uint16_t can_id = 0x141;

    double target = 0;
    double angle = 0;

    enum State
    {
        STOP = 0,
        MOVE = 1,
        HOME = 2,
    };

public:
    RmdMotor(std::string name, Can *can) : Module(name)
    {
        this->can = can;
        this->can->subscribe(this->can_id, this);
    }
    
    void loop()
    {
        if (output)
            cprintln("%s %d %.3f", this->name.c_str(), this->state, this->angle);

        uint8_t data[8] = {0x92, 0, 0, 0, 0, 0, 0, 0};
        this->can->send(this->can_id, data);
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
            this->can->send(this->can_id, data);
        }
        else if (command == "stop")
        {
            this->stop();
        }
        else if (command == "get")
        {
            cprintln("%s get %d %.3f", this->name.c_str(), this->state, this->angle);
        }
        else
        {
            cprintln("Unknown command: %s", command.c_str());
        }
    }

    void handleCanMsg(uint16_t can_id, uint8_t data[8])
    {
        if (data[0] == 0x92)
        {
            int64_t value;
            std::memcpy(&value, data + 1, 7);
            this->angle = value / 100.0 / this->ratio;
        }

        if (this->state == MOVE) {
            if (std::abs(this->angle - this->target) < this->tolerance) {
                this->state = this->target == 0 ? HOME : STOP;
            }
        }
        else {
            this->state = std::abs(this->angle) < this->tolerance ? HOME : STOP;
        }
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
        else if (key == "tolerance")
        {
            tolerance = atof(value.c_str());
        }
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
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
        this->can->send(this->can_id, data);
        this->state = MOVE;
    }

    void stop()
    {
        uint8_t data[8] = {0x81, 0, 0, 0, 0, 0, 0, 0};
        this->can->send(this->can_id, data);
        if (this->state != HOME)
            this->state = STOP;
    }
};
