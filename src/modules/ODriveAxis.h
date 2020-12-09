#pragma once

#include <string>
#include <math.h>

#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../modules/Can.h"
#include "../modules/Button.h"
#include "../utils/checksum.h"

class ODriveAxis : public Module
{
private:
    bool output = false;
    double minPos = -100.0;
    double maxPos = 100.0;
    double tolerance = 0.5;
    double homeTorque = -0.1;

    Can *can;
    uint16_t can_id;
    Button *home_switch;

    double target = 0;
    double position = 0;
    double offset = 0;

    enum State
    {
        STOP = 0,
        MOVE = 1,
        HOME = 2,
        HOMING = 3,
    };

public:
    ODriveAxis(std::string name, Button *home_switch, Can *can, std::string parameters) : Module(name)
    {
        this->can = can;
        this->can_id = std::stoi(parameters, nullptr, 16);
        this->home_switch = home_switch;
        this->can->subscribe(this->can_id + 0x009, this);
    }

    void loop()
    {
        if (output)
            cprintln("%s %d %.3f", this->name.c_str(), this->state, this->position);

        this->can->send(this->can_id + 0x009, 0, 0, 0, 0, 0, 0, 0, 0, true);

        if (this->state == HOMING and this->home_switch->state == 0) {
            this->state = HOME;
            this->stop();
            this->offset = this->position;
            this->position = 0;
        }
        if (this->state == MOVE and std::abs(this->position - this->target) < this->tolerance) {
            this->state = STOP;
            this->stop();
        }
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
            this->home();
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
            float pos, vel;
            std::memcpy(&pos, data+0, 4);
            std::memcpy(&vel, data+4, 4);
            this->position = pos - this->offset;
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "output")
        {
            output = value == "1";
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
        else if (key == "homeTorque")
        {
            homeTorque = atof(value.c_str());
        }
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
        }
    }

    void move()
    {
        this->can->send(this->can_id + 0x007, 8, 0, 0, 0, 0, 0, 0, 0); // AXIS_STATE_CLOSED_LOOP_CONTROL
        this->can->send(this->can_id + 0x00b, 3, 0, 0, 0, 1, 0, 0, 0); // CONTROL_MODE_POSITION_CONTROL

        float pos = std::max(std::min(this->target, maxPos), minPos) + this->offset;
        uint8_t data[8];
        std::memcpy(data, &pos, 4);
        this->can->send(this->can_id + 0x00c, data);
        this->state = MOVE;
    }

    void home()
    {
        this->can->send(this->can_id + 0x007, 8, 0, 0, 0, 0, 0, 0, 0); // AXIS_STATE_CLOSED_LOOP_CONTROL
        this->can->send(this->can_id + 0x00b, 1, 0, 0, 0, 1, 0, 0, 0); // CONTROL_MODE_TORQUE_CONTROL

        uint8_t data[8];
        std::memcpy(data, &this->homeTorque, 4);
        this->can->send(this->can_id + 0x00e, data);
        this->state = HOMING;
    }

    void stop()
    {
        this->can->send(this->can_id + 0x007, 1, 0, 0, 0, 0, 0, 0, 0); // AXIS_STATE_IDLE

        if (this->state != HOME)
            this->state = STOP;
    }
};
