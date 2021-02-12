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
    double ratio = 1.0;
    double speed = 10.0;
    double minAngle = -INFINITY;
    double maxAngle = INFINITY;
    double tolerance = 0.1;

    Can *can;
    const uint16_t can_id = 0x141;

    double target = 0;
    double angle = 0;
    uint8_t error = 0;

    enum State
    {
        STOP = 0,
        MOVE = 1,
        HOME = 2,
        HOMING = 3,
    };

public:
    RmdMotor(std::string name, Can *can) : Module(name)
    {
        this->can = can;
        this->can->subscribe(this->can_id, this);
    }

    void loop()
    {
        this->can->send(this->can_id, 0x92, 0, 0, 0, 0, 0, 0, 0);
        this->can->send(this->can_id, 0x9a, 0, 0, 0, 0, 0, 0, 0);

        if (this->state != MOVE and this->angle != 0 and std::abs(this->angle) < this->tolerance)
        {
            this->state = HOME;
        }

        Module::loop();
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d %d %.3f", this->state, this->error, this->angle);
        return buffer;
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "move")
        {
            this->send_move(atof(parameters.c_str()));
            this->state = MOVE;
        }
        else if (command == "home")
        {
            this->send_move(0);
            this->state = HOMING;
        }
        else if (command == "zero")
        {
            this->can->send(this->can_id, 0x19, 0, 0, 0, 0, 0, 0, 0);
        }
        else if (command == "stop")
        {
            this->stop();
        }
        else if (command == "clearError")
        {
            this->can->send(this->can_id, 0x9b, 0, 0, 0, 0, 0, 0, 0);
        }
        else
        {
            Module::handleMsg(command, parameters);
        }
    }

    void handleCanMsg(uint16_t can_id, uint8_t data[8])
    {
        if (data[0] == 0x92)
        {
            int64_t value = 0;
            std::memcpy(&value, data + 1, 7);
            this->angle = (value << 8) / 256.0 / 100.0 / this->ratio;
        }
        if (data[0] == 0x9a)
        {
            this->error = data[7];
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
        else if (key == "tolerance")
        {
            tolerance = atof(value.c_str());
        }
        else
        {
            Module::set(key, value);
        }
    }

    void send_move(double target)
    {
        this->target = std::max(std::min(target, maxAngle), minAngle);
        uint16_t speed = this->speed * this->ratio;
        uint32_t angle = this->target * this->ratio * 100;
        this->can->send(this->can_id,
                        0xa4,
                        0,
                        *((uint8_t *)(&speed) + 0),
                        *((uint8_t *)(&speed) + 1),
                        *((uint8_t *)(&angle) + 0),
                        *((uint8_t *)(&angle) + 1),
                        *((uint8_t *)(&angle) + 2),
                        *((uint8_t *)(&angle) + 3));
    }

    void stop()
    {
        if (this->state == STOP)
        {
            return;
        }
        else if (this->state == HOME)
        {
            this->send_move(0);
            this->state = HOME;
        }
        else
        {
            this->send_move(this->angle);
            this->state = STOP;
        }
    }
};
