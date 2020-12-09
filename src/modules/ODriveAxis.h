#pragma once

#include <string>
#include <math.h>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../modules/Can.h"
#include "../utils/checksum.h"

class ODriveAxis : public Module
{
private:
    bool output = false;
    double speed = 1.0;
    double torque = 1.0;
    double minPos = 0.0;
    double maxPos = 1.0;
    double tolerance = 0.1;

    Can *can;
    uint16_t can_id;

    double target = 0;
    double position = 0;

    enum State
    {
        STOP = 0,
        MOVE = 1,
        HOME = 2,
    };

public:
    ODriveAxis(std::string name, Can *can, std::string parameters) : Module(name)
    {
        this->can = can;
        this->can_id = std::stoi(parameters, nullptr, 16);
        this->can->subscribe(this->can_id + 0x009, this);
    }

    void loop()
    {
        if (output)
            cprintln("%s %d %.3f", this->name.c_str(), this->state, this->position);

        uint8_t data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
        this->can->send(this->can_id + 0x009, data, true);
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
        else if (command == "stop")
        {
            this->stop();
        }
        else if (command == "get")
        {
            cprintln("%s get %d %.3f", this->name.c_str(), this->state, this->position);
        }
        else
        {
            cprintln("Unknown command: %s", command.c_str());
        }
    }

    void handleCanMsg(uint16_t can_id, uint8_t data[8])
    {
        if (can_id == this->can_id + 0x009)
        {
            float value;
            std::memcpy(&value, data, 4);
            this->position = value;
        }

        if (this->state == MOVE) {
            if (std::abs(this->position - this->target) < this->tolerance) {
                this->state = this->target == 0 ? HOME : STOP;
            }
        }
        else {
            this->state = std::abs(this->position) < this->tolerance ? HOME : STOP;
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
        }
        else if (key == "speed")
        {
            speed = atof(value.c_str());
        }
        else if (key == "torque")
        {
            torque = atof(value.c_str());
        }
        else if (key == "minPos")
        {
            minPos = atof(value.c_str());
        }
        else if (key == "maxPos")
        {
            maxPos = atof(value.c_str());
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
        uint8_t data[8] = { 8, 0, 0, 0, 0, 0, 0, 0 }; // AXIS_STATE_CLOSED_LOOP_CONTROL
        this->can->send(this->can_id + 0x007, data);

        float pos = std::max(std::min(this->target, maxPos), minPos);
        int16_t vel = this->speed * 1000;
        int16_t trq = this->torque * 1000;
        std::memcpy(data+0, &pos, 4);
        std::memcpy(data+4, &vel, 2);
        std::memcpy(data+6, &trq, 2);
        this->can->send(this->can_id + 0x00c, data);
        this->state = MOVE;
    }

    void stop()
    {
        uint8_t data[8] = { 1, 0, 0, 0, 0, 0, 0, 0 }; // AXIS_STATE_IDLE
        std::memcpy(data, &state, 8);
        this->can->send(this->can_id + 0x007, data);

        if (this->state != HOME)
            this->state = STOP;
    }
};
