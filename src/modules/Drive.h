#pragma once

#include <string>
#include "driver/gpio.h"

#include "roboclaw/RoboClaw.h"
#include "Module.h"
#include "../ports/Port.h"
#include "../utils/strings.h"
#include "../utils/defines.h"
#include "../utils/timing.h"

class Drive : public Module
{
private:
    bool output = false;
    bool use300HzSpeedReadings = true;
    double mPerTick = 0.001; // wheelDiameter * 3.1415927 / (countsPerSpoke * spokes * gearRatio)
    double width = 1.0;
    Port *bumper_port = NULL;
    unsigned long int bumper_debounce = 1000;

    double lastTemperature = 0;
    double lastBattery = 0;

    unsigned long timeOfLastPower = 0;
    unsigned long timeOfLastSpeed = 0;
    unsigned long timeOfLastBrake = 0;
    unsigned long timeOfLastReverse = 0;
    unsigned long timeOfLastBump = 0;

    double leftPower;
    double rightPower;
    double leftSpeed;
    double rightSpeed;
    double leftReversePower;
    double rightReversePower;

    enum States
    {
        IDLE,
        POWERING,
        SPEEDING,
        BRAKING,
        REVERSING,
    } state = IDLE;

    RoboClaw *claw;

    bool sendSpeed(double left, double right)
    {
        if (lastTemperature > 70.0)
            return claw->DutyM1M2(0, 0);

        unsigned int leftCountsPerSecond = left / mPerTick;
        unsigned int rightCountsPerSecond = right / mPerTick;
        unsigned long acceleration = 0.7 / mPerTick;
        return claw->SpeedAccelM1M2(acceleration, leftCountsPerSecond, rightCountsPerSecond);
    }

    bool sendPower(double left, double right)
    {
        if (lastTemperature > 70.0)
            return claw->DutyM1M2(0, 0);

        unsigned short int dutyL = (short int)(constrain(left, -1, 1) * 32767);
        unsigned short int dutyR = (short int)(constrain(right, -1, 1) * 32767);
        return claw->DutyM1M2(dutyL, dutyR);
    }

    bool getSpeedFromPid(double &left, double &right)
    {
        unsigned int leftCountsPerSecond;
        unsigned int rightCountsPerSecond;
        bool valid = claw->ReadISpeeds(leftCountsPerSecond, rightCountsPerSecond);
        left = ((int)leftCountsPerSecond) * mPerTick;
        right = ((int)rightCountsPerSecond) * mPerTick;
        return valid;
    }

    bool getSpeedFromCounterDiffs(double &left, double &right)
    {
        uint32_t now = micros();
        static uint32_t lastStamp = 0;
        static uint32_t lastEncL = 0;
        static uint32_t lastEncR = 0;
        uint32_t encL;
        uint32_t encR;
        double dT = (now - lastStamp) * 1.0e-6;
        bool valid = claw->ReadEncoders(encL, encR);
        if (valid)
        {
            left = lastStamp > 0 ? (int)(encL - lastEncL) * mPerTick / dT : 0;
            right = lastStamp > 0 ? (int)(encR - lastEncR) * mPerTick / dT : 0;
            lastEncL = encL;
            lastEncR = encR;
            lastStamp = now;
        }
        return valid;
    }

    bool getSpeed(double &left, double &right)
    {
        return use300HzSpeedReadings ? getSpeedFromPid(left, right) : getSpeedFromCounterDiffs(left, right);
    }

    bool getTemp(uint16_t &temp)
    {
        bool valid = claw->ReadTemp(temp);
        if (valid)
            lastTemperature = 0.1 * temp;
        return valid;
    }

    bool getBattery(uint16_t &battery)
    {
        bool valid;
        battery = claw->ReadMainBatteryVoltage(&valid);
        if (valid)
            lastBattery = 0.1 * battery;
        return valid;
    }

    bool handleError(bool valid)
    {
        static int count = 0;

        if (valid)
            count = 0;
        else
        {
            printf("%s Communication problem with %s RoboClaw\n", ++count < 3 ? "warn" : "error", name.c_str());
            claw->clear();
        }

        return valid;
    }

    void triggerPower(double left, double right)
    {
        timeOfLastPower = millis();
        leftPower = left;
        rightPower = right;
        if (state == IDLE or state == SPEEDING)
            state = POWERING;
    }

    void triggerSpeed(double left, double right)
    {
        timeOfLastSpeed = millis();
        leftSpeed = left;
        rightSpeed = right;
        if (state == IDLE and millisSince(timeOfLastBump) > 2.0e3)
            state = SPEEDING;
    }

    void triggerBump(double current_left_speed, double current_right_speed)
    {
        if (state == SPEEDING)
        {
            state = REVERSING;
            leftReversePower = current_left_speed > 0 ? -1.0 : 1.0;
            rightReversePower = current_right_speed > 0 ? -1.0 : 1.0;
            timeOfLastReverse = millis();
        }
    }

    void triggerStop()
    {
        if (state == SPEEDING)
        {
            state = BRAKING;
            timeOfLastBrake = millis();
        }
    }

    void init_bumper()
    {
        if (bumper_port != NULL)
            bumper_port->setup(true, -1);
    }

public:
    Drive(std::string name, std::string parameters) : Module(name)
    {
        std::string type = cut_first_word(parameters, ',');
        int address = atoi(cut_first_word(parameters, ',').c_str());

        if (type != "roboclaw" and not type.empty())
        { 
            printf("Invalid type: %s\n", type.c_str());
        }
        claw = new RoboClaw(UART_NUM_1, GPIO_NUM_26, GPIO_NUM_27, 38400, address);
    }

    void setup()
    {
        claw->begin();
        init_bumper();
    }

    void loop()
    {
        double left_speed = 0, right_speed = 0;
        if (not handleError(getSpeed(left_speed, right_speed)))
            return;
        double linear = (left_speed + right_speed) / 2.0;
        double angular = (right_speed - left_speed) / width;

        uint16_t temp;
        if (not handleError(getTemp(temp)))
            return;

        uint16_t battery;
        if (not handleError(getBattery(battery)))
            return;

        if (bumper_port != NULL)
        {
            static bool bumper_state = false;
            if (bumper_port->get_level() == 0)
                timeOfLastBump = millis();
            unsigned long millisSinceLastBump = millis() - timeOfLastBump;
            bool new_bumper_state = millisSinceLastBump <= bumper_debounce;
            if (new_bumper_state != bumper_state and output)
                printf("%s bump %s\n", name.c_str(), new_bumper_state ? "start" : "stop");
            bumper_state = new_bumper_state;
            if (bumper_state)
                triggerBump(left_speed, right_speed);
        }

        if (state == IDLE)
        {
            handleError(sendPower(0, 0));
        }
        else if (state == POWERING)
        {
            handleError(sendPower(leftPower, rightPower));
            if (millisSince(timeOfLastPower) > 0.5e3)
                to_idle();
        }
        else if (state == SPEEDING)
        {
            double m = millisSince(timeOfLastSpeed);
            if (m < 0.2e3)
                handleError(sendSpeed(leftSpeed, rightSpeed));
            else if (m < 0.5e3)
            {
                double factor = 1.0 - (m - 0.2e3) / (0.5e3 - 0.2e3);
                handleError(sendSpeed(factor * leftSpeed, factor * rightSpeed));
            }
            else if (m < 0.7e3)
                handleError(sendSpeed(0, 0));
            else
                to_idle();
        }
        else if (state == BRAKING)
        {
            handleError(sendSpeed(0, 0));
            if (millisSince(timeOfLastBrake) > 1.0e3)
                to_idle();
        }
        else if (state == REVERSING)
        {
            double factor = 1.0 - millisSince(timeOfLastReverse) / 0.4e3;
            handleError(sendPower(factor * leftReversePower, factor * rightReversePower));
            if (millisSince(timeOfLastReverse) > 0.4e3)
                to_idle();
        }

        if (output)
        {
            printf("%s status\t", this->name.c_str());
            printf("%.4f\t", linear);
            printf("%.4f\t", angular);
            printf("%.1f\t", temp * 0.1);
            printf("%.1f\t", battery * 0.1);
            if (state == IDLE)
                printf("IDLE\n");
            if (state == POWERING)
                printf("POWERING\n");
            if (state == SPEEDING)
                printf("SPEEDING\n");
            if (state == BRAKING)
                printf("BRAKING\n");
            if (state == REVERSING)
                printf("REVERSING\n");
        }
    }

    void to_idle()
    {
        state = IDLE;
        sendPower(0, 0);
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
                output = msg == "1";
            else if (key == "mPerTick")
                mPerTick = atof(msg.c_str());
            else if (key == "width")
                width = atof(msg.c_str());
            else if (key == "use300HzSpeedReadings")
                use300HzSpeedReadings = msg == "1";
            else if (key == "bumper_port")
            {
                bumper_port = Port::fromString(msg);
                init_bumper();
            }
            else if (key == "bumper_debounce")
                bumper_debounce = atoi(msg.c_str());
            else
                printf("Unknown setting: %s\n", key.c_str());
        }
        else if (command == "pw")
        {
            double left = atof(cut_first_word(msg, ',').c_str());
            double right = atof(cut_first_word(msg, ',').c_str());
            triggerPower(left, right);
        }
        else if (command == "speed")
        {
            double speed = atof(cut_first_word(msg, ',').c_str());
            double curvature = atof(cut_first_word(msg, ',').c_str());
            double s_l = speed - speed * width * curvature / 2.0;
            double s_r = speed + speed * width * curvature / 2.0;
            double f_l = s_l != 0.0 ? fabs(speed) / fabs(s_l) : 1.0;
            double f_r = s_r != 0.0 ? fabs(speed) / fabs(s_r) : 1.0;
            double f = _min(f_l, f_r);
            triggerSpeed(s_l * f, s_r * f);
        }
        else if (command == "left")
        {
            double speed = atof(msg.c_str());
            triggerSpeed(-speed, speed);
        }
        else if (command == "right")
        {
            double speed = atof(msg.c_str());
            triggerSpeed(speed, -speed);
        }
        else if (command == "stop")
        {
            triggerStop();
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
