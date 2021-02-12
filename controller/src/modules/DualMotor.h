#pragma once

#include <string>

#include "Module.h"
#include "roboclaw/RoboClaw.h"
#include "../utils/strings.h"
#include "../utils/defines.h"
#include "../utils/checksum.h"

class DualMotor : public Module
{
private:
    uint32_t accel1 = 1e5;
    uint32_t accel2 = 1e5;

    double homePw1 = 0;
    double homePw2 = 0;

    std::map<std::string, std::pair<int32_t, int32_t>> points;

    enum States
    {
        IDLE,
        HOMING_START,
        HOMING,
        MOVING,
    } state = IDLE;

    struct claw_values_t
    {
        uint32_t statusBits;
        uint8_t depth1;
        uint8_t depth2;
        int32_t position1;
        int32_t position2;
        int16_t current1;
        int16_t current2;
        int32_t countsPerSecond1;
        int32_t countsPerSecond2;
    } values;

    RoboClaw *claw;

    std::string state_to_string(States state)
    {
        return state == IDLE ? "IDLE" : state == HOMING_START ? "HOMING_START" : state == HOMING ? "HOMING" : state == MOVING ? "MOVING" : "unknown";
    }

    bool sendPower(double pw1, double pw2)
    {
        unsigned short int duty1 = (short int)(constrain(pw1, -1, 1) * 32767);
        unsigned short int duty2 = (short int)(constrain(pw2, -1, 1) * 32767);
        return claw->DutyM1M2(duty1, duty2);
    }

    bool read_values(claw_values_t &values)
    {
        values = {};

        uint8_t status;
        bool valid;

        values.statusBits = claw->ReadError(&valid);
        if (not valid)
            return false;

        valid = claw->ReadBuffers(values.depth1, values.depth2);
        if (not valid)
            return false;

        values.position1 = claw->ReadEncM1(&status, &valid);
        if (not valid)
            return false;

        values.position2 = claw->ReadEncM2(&status, &valid);
        if (not valid)
            return false;

        valid = claw->ReadCurrents(values.current1, values.current2);
        if (not valid)
            return false;

        values.countsPerSecond1 = claw->ReadISpeedM1(&status, &valid);
        if (not valid)
            return false;

        values.countsPerSecond2 = claw->ReadISpeedM2(&status, &valid);
        if (not valid)
            return false;

        return true;
    }

    std::string getOutput()
    {
        char buffer[256];
        std::sprintf(buffer, "%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%x\t%s",
            values.position1,
            values.position2,
            values.countsPerSecond1,
            values.countsPerSecond2,
            values.depth1,
            values.depth2,
            values.current1,
            values.current2,
            values.statusBits,
            state_to_string(state).c_str());
        return buffer;
    }

    bool handleError(bool valid)
    {
        static int count = 0;

        if (valid)
            count = 0;
        else
        {
            cprintln("%s Communication problem with %s RoboClaw", ++count < 3 ? "warn" : "error", name.c_str());
            claw->clear();
        }

        return valid;
    }

public:
    DualMotor(std::string name, std::string parameters) : Module(name)
    {
        std::string type = cut_first_word(parameters, ',');
        int address = parameters.empty() ? 128 : atoi(cut_first_word(parameters, ',').c_str());
        int baud = parameters.empty() ? 38400 : atoi(cut_first_word(parameters, ',').c_str());

        if (type != "roboclaw" and not type.empty())
        {
            cprintln("Invalid type: %s", type.c_str());
        }
        claw = new RoboClaw(UART_NUM_1, GPIO_NUM_26, GPIO_NUM_27, baud, address);
    }

    void setup()
    {
        claw->begin();
    }

    void loop()
    {
        if (not handleError(read_values(values)))
            return;

        Module::loop();

        static int16_t lastCurrent1 = 0;
        static int16_t lastCurrent2 = 0;
        if (state == HOMING_START)
        {
            lastCurrent1 = 0;
            lastCurrent2 = 0;

            wiggle();
            sendPower(-homePw1, -homePw2);
            delay(500);

            state = HOMING;
            return;
        }

        if (state == HOMING)
        {
            bool overcurrent1 = _min(lastCurrent1, values.current1) > 1000;
            bool overcurrent2 = _min(lastCurrent2, values.current2) > 1000;
            lastCurrent1 = values.depth1 == 0x80 ? values.current1 : 0;
            lastCurrent2 = values.depth2 == 0x80 ? values.current2 : 0;
            if (overcurrent1 or overcurrent2)
            {
                cprintln("error Overcurrent");
                if (handleError(sendPower(0, 0)))
                    state = IDLE;
            }
        }

        bool limit1 = values.statusBits & 0x400000;
        bool limit2 = values.statusBits & 0x800000;
        if (state == HOMING)
        {
            handleError(sendPower(limit1 ? 0 : homePw1, limit2 ? 0 : homePw2));
            if (limit1 & limit2)
            {
                state = IDLE;
                claw->ResetEncoders();
                cprintln("%s home completed", name.c_str());
            }
        }

        bool isEStopPressed = values.statusBits & 1;
        if (state == MOVING)
        {
            if (values.depth1 == 0x80 and values.depth2 == 0x80)
            {
                state = IDLE;
                if (not isEStopPressed)
                    cprintln("%s move completed", name.c_str());
            }
            else {
                if (limit1 or limit2) {
                    wiggle();
                    stop();
                    cprintln("error Limit switch during move command");
                }
            }
        }
    }

    void handleMsg(std::string command, std::string parameters)
    {
        if (command == "pw")
        {
            double pw1 = atof(cut_first_word(parameters, ',').c_str());
            double pw2 = atof(cut_first_word(parameters, ',').c_str());
            handleError(sendPower(pw1, pw2));
        }
        else if (command == "home")
            state = HOMING_START;
        else if (command == "move")
        {
            if (state == IDLE)
            {
                std::string arg1 = cut_first_word(parameters, ',');
                std::string arg2 = cut_first_word(parameters, ',');
                std::string arg3 = cut_first_word(parameters, ',');
                if (points.count(arg1) > 0)
                    move(points[arg1].first, points[arg1].second, atof(arg2.c_str()));
                else
                    move(atoi(arg1.c_str()), atoi(arg2.c_str()), atof(arg3.c_str()));
            }
            else
            {
                cprintln("Can't start moving in state %s", state_to_string(state).c_str());
            }
        }
        else if (command == "stop")
            stop();
        else
        {
            Module::handleMsg(command, parameters);
        }
    }

    void set(std::string key, std::string value)
    {
        if (key == "accel1")
        {
            accel1 = atoi(value.c_str());
        }
        else if (key == "accel2")
        {
            accel2 = atoi(value.c_str());
        }
        else if (key == "homePw1")
        {
            homePw1 = atof(value.c_str());
        }
        else if (key == "homePw2")
        {
            homePw2 = atof(value.c_str());
        }
        else if (starts_with(key, "point_"))
        {
            cut_first_word(key, '_');
            int32_t pos1 = atoi(cut_first_word(value, ',').c_str());
            int32_t pos2 = atoi(cut_first_word(value, ',').c_str());
            points[key] = std::make_pair(pos1, pos2);
        }
        else
        {
            Module::set(key, value);
        }
    }

    void move(int32_t target1, int32_t target2, double duration)
    {
        uint32_t speed1 = abs(target1 - values.position1) / duration;
        uint32_t speed2 = abs(target2 - values.position2) / duration;
        if (handleError(claw->SpeedAccelDeccelPositionM1M2(
                this->accel1, speed1, this->accel1, target1,
                this->accel2, speed2, this->accel2, target2, 1)))
            state = MOVING;
    }

    void stop()
    {
        bool valid = false;
        while (not valid)
            valid = sendPower(0, 0);
        state = IDLE;
    }

    void wiggle()
    {
        claw->DutyM1M2(1, 1);
        delay(10);
        claw->DutyM1M2(65535, 65535);
        delay(10);
    }
};