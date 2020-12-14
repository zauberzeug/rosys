#pragma once

#include <string>

#include "Module.h"
#include "../modules/ODriveAxis.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class WheelPair : public Module
{
private:
    double leftTorqueFactor = 1.0;
    double rightTorqueFactor = 1.0;
    double leftSpeedFactor = 1.0;
    double rightSpeedFactor = 1.0;
    double width = 1.0;

    ODriveAxis *leftAxis;
    ODriveAxis *rightAxis;

public:
    WheelPair(std::string name, ODriveAxis *left, ODriveAxis *right) : Module(name)
    {
        this->leftAxis = left;
        this->rightAxis = right;
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "torque")
        {
            double left = atof(cut_first_word(msg, ',').c_str());
            double right = atof(cut_first_word(msg, ',').c_str());
            this->leftAxis->torque(left * this->leftTorqueFactor);
            this->rightAxis->torque(right * this->rightTorqueFactor);
        }
        else if (command == "speed")
        {
            double linear = atof(cut_first_word(msg, ',').c_str());
            double curvature = atof(cut_first_word(msg, ',').c_str());
            double s_l = linear - linear * width * curvature / 2.0;
            double s_r = linear + linear * width * curvature / 2.0;
            double f_l = s_l != 0.0 ? fabs(linear) / fabs(s_l) : 1.0;
            double f_r = s_r != 0.0 ? fabs(linear) / fabs(s_r) : 1.0;
            double f = _min(f_l, f_r);
            this->leftAxis->speed(s_l * f * this->leftSpeedFactor);
            this->rightAxis->speed(s_r * f * this->rightSpeedFactor);
        }
        else if (command == "left")
        {
            double speed = atof(msg.c_str());
            this->leftAxis->speed(-speed * this->leftSpeedFactor);
            this->rightAxis->speed(speed * this->rightSpeedFactor);
        }
        else if (command == "right")
        {
            double speed = atof(msg.c_str());
            this->leftAxis->speed(speed * this->leftSpeedFactor);
            this->rightAxis->speed(-speed * this->rightSpeedFactor);
        }
        else if (command == "stop")
        {
            stop();
        }
        else
        {
            cprintln("Unknown command: %s", command.c_str());
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "width")
        {
            width = atof(value.c_str());
        }
        else if (key == "leftTorqueFactor")
        {
            leftTorqueFactor = atof(value.c_str());
        }
        else if (key == "rightTorqueFactor")
        {
            rightTorqueFactor = atof(value.c_str());
        }
        else if (key == "leftSpeedFactor")
        {
            leftSpeedFactor = atof(value.c_str());
        }
        else if (key == "rightSpeedFactor")
        {
            rightSpeedFactor = atof(value.c_str());
        }
        else
        {
            cprintln("Unknown setting: %s", key.c_str());
        }
    }

    void stop()
    {
        this->leftAxis->stop();
        this->rightAxis->stop();
    }
};
