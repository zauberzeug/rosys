#pragma once

#include "Module.h"
#include "../utils/strings.h"
#include "bno055/BNO055ESP32.h"

class Bno055 : public Module
{
private:
    BNO055 *bno;

public:
    Bno055(std::string name) : Module(name)
    {
        i2c_config_t conf;
        conf.mode = I2C_MODE_MASTER;
        conf.sda_io_num = GPIO_NUM_21;
        conf.scl_io_num = GPIO_NUM_22;
        conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
        conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
        conf.master.clk_speed = 100000;
        i2c_param_config(I2C_NUM_1, &conf);
        i2c_driver_install(I2C_NUM_1, I2C_MODE_MASTER, 0, 0, 0);
        i2c_set_timeout(I2C_NUM_1, 300000);

        this->bno = new BNO055(I2C_NUM_1, 0x28);
        this->bno->begin();
        this->bno->enableExternalCrystal();
        this->bno->setOprModeNdof();
    }

    void setup()
    {
    }

    void loop()
    {
    }

    void handleMsg(std::string msg)
    {
        std::string command = cut_first_word(msg);

        if (command == "get")
        {
            bno055_vector_t e = this->bno->getVectorEuler();
            printf("%s %7.2f %7.2f %7.2f\n", this->name.c_str(), e.x, e.y, e.z);
        }
        else
        {
            printf("Unknown command: %s\n", command.c_str());
        }
    }
};
