#pragma once

#include <string>

#include "Module.h"
#include "roboclaw/RoboClaw.h"
#include "../utils/strings.h"
#include "../utils/defines.h"

class DualMotor : public Module
{
private:
    bool output = false;

    uint32_t accel1 = 1e5;
    uint32_t accel2 = 1e5;

    Port *limit1A = NULL;
    Port *limit1B = NULL;
    Port *limit2A = NULL;
    Port *limit2B = NULL;

    int limit1A_active = 0;
    int limit1B_active = 0;
    int limit2A_active = 0;
    int limit2B_active = 0;

    double homePw1 = 0;
    double homePw2 = 0;

    std::map<std::string, std::pair<int32_t, int32_t>> points;

    enum LimitStates
    {
        UNKNOWN,
        ACTIVE,
        PASSIVE,
    };

    enum States
    {
        IDLE,
        HOMING_START,
        HOMING_BACKWARD,
        HOMING_FORWARD,
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
        return state == IDLE ? "IDLE" :
            state == HOMING_START ? "HOMING_START" :
            state == HOMING_BACKWARD ? "HOMING_BACKWARD" :
            state == HOMING_FORWARD ? "HOMING_FORWARD" :
            state == MOVING ? "MOVING" : "unknown";
    }

    bool sendPower(double pw1, double pw2)
    {
        unsigned short int duty1 = (short int)(constrain(pw1, -1, 1) * 32767);
        unsigned short int duty2 = (short int)(constrain(pw2, -1, 1) * 32767);
        return claw->DutyM1M2(duty1, duty2);
    }

    bool read_values(claw_values_t &values)
    {
        values ={};

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

    void print_values(claw_values_t values)
    {
        printf("tool status %d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%x\t%s\n",
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

public:
    DualMotor(std::string name, std::string parameters) : Module(name)
    {
        std::string type = cut_first_word(parameters, ',');
        int address = parameters.empty() ? 128 : atoi(cut_first_word(parameters, ',').c_str());
        int baud = parameters.empty() ? 38400 : atoi(cut_first_word(parameters, ',').c_str());

        if (type != "roboclaw" and not type.empty())
        {
            printf("Invalid type: %s\n", type.c_str());
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

        if (output)
            print_values(values);

        static int16_t lastCurrent1 = 0;
        static int16_t lastCurrent2 = 0;
        if (state == HOMING_START)
        {
            lastCurrent1 = 0;
            lastCurrent2 = 0;
            state = HOMING_BACKWARD;
        }

        if (state == HOMING_BACKWARD or state == HOMING_FORWARD)
        {
            bool overcurrent1 = _min(lastCurrent1, values.current1) > 1000;
            bool overcurrent2 = _min(lastCurrent2, values.current2) > 1000;
            lastCurrent1 = values.depth1 == 0x80 ? values.current1 : 0;
            lastCurrent2 = values.depth2 == 0x80 ? values.current2 : 0;
            if (overcurrent1 or overcurrent2)
            {
                printf("error Overcurrent\n");
                if (handleError(sendPower(0, 0)))
                    state = IDLE;
            }
        }

        LimitStates limit1 = homePw1 < 0 ?
            (limit1A == NULL ? UNKNOWN : limit1A->get_level() == limit1A_active ? ACTIVE : PASSIVE) :
            (limit1B == NULL ? UNKNOWN : limit1B->get_level() == limit1B_active ? ACTIVE : PASSIVE);
        LimitStates limit2 = homePw2 < 0 ?
            (limit2A == NULL ? UNKNOWN : limit2A->get_level() == limit2A_active ? ACTIVE : PASSIVE) :
            (limit2B == NULL ? UNKNOWN : limit2B->get_level() == limit2B_active ? ACTIVE : PASSIVE);
        if (state == HOMING_BACKWARD)
        {
            handleError(sendPower(limit1 == PASSIVE ? homePw1 : 0, limit2 == PASSIVE ? homePw2 : 0));
            if (limit1 != PASSIVE and limit2 != PASSIVE)
                state = HOMING_FORWARD;
        }
        if (state == HOMING_FORWARD)
        {
            handleError(sendPower(limit1 == ACTIVE ? -homePw1 / 2 : 0, limit2 == ACTIVE ? -homePw2 / 2 : 0));
            if (limit1 != ACTIVE and limit2 != ACTIVE)
            {
                claw->ResetEncoders();
                state = IDLE;
                printf("tool home completed\n"); // TODO
            }
        }

        bool isEStopPressed = values.statusBits & 1;
        if (state == MOVING and values.depth1 == 0x80 and values.depth2 == 0x80)
        {
            state = IDLE;
            if (not isEStopPressed)
                printf("tool move completed\n"); // TODO
        }

        // TODO: stop if moving into limit switch
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
                output = msg == "1";
            else if (key == "accel1")
                accel1 = atoi(msg.c_str());
            else if (key == "accel2")
                accel2 = atoi(msg.c_str());
            else if (key == "limit1A") {
                limit1A = Port::fromString(msg);
                limit1A->setup(true, 1);
            }
            else if (key == "limit1B") {
                limit1B = Port::fromString(msg);
                limit1B->setup(true, 1);
            }
            else if (key == "limit2A") {
                limit2A = Port::fromString(msg);
                limit2A->setup(true, 1);
            }
            else if (key == "limit2B") {
                limit2B = Port::fromString(msg);
                limit2B->setup(true, 1);
            }
            else if (key == "limit1A_active")
                limit1A_active = atoi(msg.c_str());
            else if (key == "limit1B_active")
                limit1B_active = atoi(msg.c_str());
            else if (key == "limit2A_active")
                limit2A_active = atoi(msg.c_str());
            else if (key == "limit2B_active")
                limit2B_active = atoi(msg.c_str());
            else if (key == "homePw1")
                homePw1 = atof(msg.c_str());
            else if (key == "homePw2")
                homePw2 = atof(msg.c_str());
            else if (starts_with(key, "point_")) {
                cut_first_word(key, '_');
                int32_t pos1 = atoi(cut_first_word(msg, ',').c_str());
                int32_t pos2 = atoi(cut_first_word(msg, ',').c_str());
                points[key] = std::make_pair(pos1, pos2);
            }
            else
                printf("Unknown setting: %s\n", key.c_str());
        }
        else if (command == "pw")
        {
            double pw1 = atof(cut_first_word(msg, ',').c_str());
            double pw2 = atof(cut_first_word(msg, ',').c_str());
            handleError(sendPower(pw1, pw2));
        }
        else if (command == "home")
            home();
        else if (command == "move")
        {
            if (state == IDLE)
            {
                std::string arg1 = cut_first_word(msg, ',');
                std::string arg2 = cut_first_word(msg, ',');
                std::string arg3 = cut_first_word(msg, ',');
                if (points.count(arg1) > 0)
                    move(points[arg1].first, points[arg1].second, atof(arg2.c_str()));
                else
                    move(atoi(arg1.c_str()), atoi(arg2.c_str()), atof(arg3.c_str()));
            }
            else
            {
                printf("Can't start moving in state %s\n", state_to_string(state).c_str());
            }
        }
        else if (command == "get") {
            if (handleError(read_values(values)))
                print_values(values);
        }
        else if (command == "stop")
            stop();
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }

    void home()
    {
        if (claw->ReadError())
        {
            printf("error Please move tool out of end position and restart robot.\n");
            return;
        }

        if (state != IDLE)
        {
            printf("error Tool needs to be in idle state for homing.\n");
            return;
        }

        state = HOMING_START;
    }

    void move(int32_t target1, int32_t target2, double duration)
    {
        printf("MOVE %d,%d (%.3f s)\n", target1, target2, duration);

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
};