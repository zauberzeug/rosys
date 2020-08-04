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

    int32_t scaleA = 1.0;
    int32_t scaleB = 1.0;
    uint32_t accelA = 1e5;
    uint32_t accelB = 1e5;

    const double safety = 0.03;

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
        uint8_t depthA;
        uint8_t depthB;
        int32_t positionA;
        int32_t positionB;
        int16_t currentA;
        int16_t currentB;
        int32_t countsPerSecondB;
    } values;

    RoboClaw *claw;

    bool sendPower(double pwA, double pwB)
    {
        unsigned short int duty1 = (short int)(constrain(pwA, -1, 1) * 32767);
        unsigned short int duty2 = (short int)(constrain(pwB, -1, 1) * 32767);
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

        valid = claw->ReadBuffers(values.depthA, values.depthB);
        if (not valid)
            return false;

        values.positionA = claw->ReadEncM1(&status, &valid);
        if (not valid)
            return false;

        values.positionB = claw->ReadEncM2(&status, &valid);
        if (not valid)
            return false;

        valid = claw->ReadCurrents(values.currentA, values.currentB);
        if (not valid)
            return false;

        values.countsPerSecondB = claw->ReadISpeedM2(&status, &valid);
        if (not valid)
            return false;

        return true;
    }

    void print_values(claw_values_t values)
    {
        printf("tool status %.4f\t%.4f\t%.4f\t%d\t%d\t%d\t%d\t%x\n",
               (double)values.positionA / scaleA,
               (double)values.positionB / scaleB,
               (double)values.countsPerSecondB / scaleB,
               values.depthA,
               values.depthB,
               values.currentA,
               values.currentB,
               values.statusBits);
    }

    bool handleError(bool valid)
    {
        static int count = 0;

        if (valid)
            count = 0;
        else
        {
            printf("%s Communication problem with ToolClaw\n", ++count < 3 ? "warn" : "error");
            claw->clear();
        }

        return valid;
    }

public:
    DualMotor(std::string name, std::string type) : Module(name)
    {
        if (type != "roboclaw" and not type.empty())
        {
            printf("Invalid type: %s\n", type.c_str());
        }
        claw = new RoboClaw(UART_NUM_1, GPIO_NUM_26, GPIO_NUM_27, 38400, 0x80);
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

        static int16_t lastCurrentA = 0;
        static int16_t lastCurrentB = 0;
        if (state == HOMING_START)
        {
            lastCurrentA = 0;
            lastCurrentB = 0;
            state = HOMING_BACKWARD;
        }

        if (state == HOMING_BACKWARD or state == HOMING_FORWARD)
        {
            bool overcurrentA = _min(lastCurrentA, values.currentA) > 1000;
            bool overcurrentB = _min(lastCurrentB, values.currentB) > 1000;
            lastCurrentA = values.depthA == 0x80 ? values.currentA : 0;
            lastCurrentB = values.depthB == 0x80 ? values.currentB : 0;
            if (overcurrentA or overcurrentB)
            {
                printf("error Overcurrent\n");
                if (handleError(sendPower(0, 0)))
                    state = IDLE;
            }
        }

        bool limitA = values.statusBits & 0x400000;
        bool limitB = values.statusBits & 0x800000;
        if (state == HOMING_BACKWARD)
        {
            handleError(sendPower(limitA ? 0 : 0.1, limitB ? 0 : -0.15));
            if (limitA and limitB)
                state = HOMING_FORWARD;
        }
        if (state == HOMING_FORWARD)
        {
            handleError(sendPower(limitA ? -0.05 : 0, limitB ? 0.075 : 0));
            if (not limitA and not limitB)
            {
                claw->ResetEncoders();
                int32_t targetPosA = -safety * scaleA;
                int32_t targetPosB = safety * scaleB;
                if (handleError(claw->SpeedAccelDeccelPositionM1M2(this->accelA, 10 * abs(targetPosA), this->accelA, targetPosA,
                                                                   this->accelB, 10 * abs(targetPosB), this->accelB, targetPosB, 1)))
                {
                    state = IDLE;
                    printf("tool home completed\n"); // TODO
                }
            }
        }

        bool isEStopPressed = values.statusBits & 1;
        if (state == MOVING and values.depthA == 0x80 and values.depthB == 0x80)
        {
            state = IDLE;
            if (not isEStopPressed)
                printf("tool move completed\n");
        }
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "set")
        {
            std::string key = cut_first_word(msg, '=');
            if (key == "output")
                output = msg == "1";
            else if (key == "scaleA")
                scaleA = atoi(msg.c_str());
            else if (key == "scaleB")
                scaleB = atoi(msg.c_str());
            else if (key == "accelA")
                accelA = atoi(msg.c_str());
            else if (key == "accelB")
                accelB = atoi(msg.c_str());
            else
                printf("Unknown setting: %s\n", key.c_str());
        }
        else if (command == "pw")
        {
            double pwA = atof(cut_first_word(msg, ',').c_str());
            double pwB = atof(cut_first_word(msg, ',').c_str());
            handleError(sendPower(pwA, pwB));
        }
        else if (command == "home")
            home();
        else if (command == "move")
        {
            if (state == IDLE)
            {
                double targetA = atof(cut_first_word(msg, ',').c_str());
                double targetB = atof(cut_first_word(msg, ',').c_str());
                double duration = atof(cut_first_word(msg, ',').c_str());
                move(targetA, targetB, duration);
            }
            else
            {
                printf("Can't start moving in state %d\n", state);
            }
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

    void move(double targetA, double targetB, double duration)
    {
        int32_t targetPosA = constrain(targetA, -1.00, -safety) * scaleA;
        int32_t targetPosB = constrain(targetB, safety, 1.00) * scaleB;
        uint32_t speedA = abs(targetPosA - values.positionA) / duration;
        uint32_t speedB = abs(targetPosB - values.positionB) / duration;
        if (handleError(claw->SpeedAccelDeccelPositionM1M2(this->accelA, speedA, this->accelA, targetPosA,
                                                           this->accelB, speedB, this->accelB, targetPosB, 1)))
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