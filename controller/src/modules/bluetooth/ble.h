#pragma once

#include <string>

class Ble {

  public:

    static std::string value;

    static void init(std::string device_name, void (*handleCommand)(std::string));

};

