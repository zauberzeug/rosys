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

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "torque")
        {
            double left = atof(cut_first_word(parameters, ',').c_str());
            double right = atof(cut_first_word(parameters, ',').c_str());
            this->leftAxis->torque(left * this->leftTorqueFactor);
            this->rightAxis->torque(right * this->rightTorqueFactor);
        }
        else if (command == "speed")
        {
            double linear = atof(cut_first_word(parameters, ',').c_str());
            double angular = atof(cut_first_word(parameters, ',').c_str());
            double left = linear - angular * width / 2.0;
            double right = linear + angular * width / 2.0;
            this->leftAxis->speed(left * this->leftSpeedFactor);
            this->rightAxis->speed(right * this->rightSpeedFactor);
        }
        else if (command == "stop")
        {
            stop();
        }
        else
        {
            Module::handleMsg(command, parameters);
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
            Module::set(key, value);
        }
    }

    void stop()
    {
        this->leftAxis->stop();
        this->rightAxis->stop();
    }
};
