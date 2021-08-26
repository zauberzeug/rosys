#pragma once

#include <string>

#include "Module.h"
#include "../modules/ODriveMotor.h"
#include "../utils/strings.h"
#include "../utils/checksum.h"

class ODriveWheels : public Module
{
private:
    double leftTorqueFactor = 1.0;
    double rightTorqueFactor = 1.0;
    double leftSpeedFactor = 1.0;
    double rightSpeedFactor = 1.0;
    double width = 1.0;

    ODriveMotor *leftMotor;
    ODriveMotor *rightMotor;

    double linear_speed = 0.0;
    double angular_speed = 0.0;

public:
    ODriveWheels(std::string name, ODriveMotor *left, ODriveMotor *right) : Module(name)
    {
        this->leftMotor = left;
        this->rightMotor = right;
    }

    void loop()
    {
        static unsigned long int last_millis = 0;
        static double last_left_position;
        static double last_right_position;

        if (last_millis != 0)
        {
            unsigned long int dMillis = millis() - last_millis;
            double left_speed = (this->leftMotor->position - last_left_position) / dMillis * 1000;
            double right_speed = (this->rightMotor->position - last_right_position) / dMillis * 1000;
            this->linear_speed = (left_speed + right_speed) / 2;
            this->angular_speed = (right_speed - left_speed) / this->width;
        }

        last_millis = millis();
        last_left_position = this->leftMotor->position;
        last_right_position = this->rightMotor->position;
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%7.4f %7.4f", this->linear_speed, this->angular_speed);
        return buffer;
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "torque")
        {
            double left = atof(cut_first_word(parameters, ',').c_str());
            double right = atof(cut_first_word(parameters, ',').c_str());
            this->leftMotor->torque(left * this->leftTorqueFactor);
            this->rightMotor->torque(right * this->rightTorqueFactor);
        }
        else if (command == "speed_lr") // DEPRICATED
        {
            double left = atof(cut_first_word(parameters, ',').c_str());
            double right = atof(cut_first_word(parameters, ',').c_str());
            this->leftMotor->speed(left * this->leftSpeedFactor);
            this->rightMotor->speed(right * this->rightSpeedFactor);
        }
        else if (command == "speed")
        {
            double linear = atof(cut_first_word(parameters, ',').c_str());
            double angular = atof(cut_first_word(parameters, ',').c_str());
            double left = linear - angular * width / 2.0;
            double right = linear + angular * width / 2.0;
            this->leftMotor->speed(left * this->leftSpeedFactor);
            this->rightMotor->speed(right * this->rightSpeedFactor);
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
        this->leftMotor->stop();
        this->rightMotor->stop();
    }
};
