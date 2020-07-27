#pragma once

#include <string>
#include "driver/gpio.h"

#include "roboclaw/RoboClaw.h"
#include "Module.h"
#include "../Port.h"
#include "../utils/strings.h"
#include "../utils/defines.h"

class Drive : public Module
{
private:
    bool output = false;

    RoboClaw *claw;

public:
    Drive(std::string name, std::string type) : Module(name)
    {
        if (type != "roboclaw" and not type.empty())
        {
            printf("Invalid type: %s\n", type.c_str());
            return;
        }
        claw = new RoboClaw(UART_NUM_1, GPIO_NUM_26, GPIO_NUM_27, 38400, 0x80);
    }

    void setup()
    {
        claw->begin();
    }

    void loop()
    {
        if (output)
            printf("%s ...\n", this->name.c_str());
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
            {
                output = msg == "1";
            }
            else
            {
                printf("Unknown setting: %s\n", key.c_str());
            }
        }
        else if (command == "pw")
        {
            double left = atof(cut_first_word(msg, ',').c_str());
            double right = atof(cut_first_word(msg, ',').c_str());
            unsigned short int dutyL = (short int)(constrain(left, -1, 1) * 32767);
            unsigned short int dutyR = (short int)(constrain(right, -1, 1) * 32767);
            claw->DutyM1M2(dutyL, dutyR);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
