#pragma once

#include <map>
#include <string>

#include "modules/Module.h"
#include "utils/strings.h"

class Condition
{
public:
    std::string trigger;
    bool equality;
    int state;
    std::string msg;

    Condition(std::string line)
    {
        trigger = cut_first_word(line);
        equality = cut_first_word(line) == "==";
        state = atoi(cut_first_word(line).c_str());
        msg = std::string(line);
    }

    bool is_true(std::map<std::string, Module *> *modules)
    {
        return ((state == (*modules)[trigger]->state) == equality);
    }

    std::string to_string()
    {
        return std::string("if ")
            + trigger + " "
            + (equality ? "== " : "!= ")
            + std::to_string(state) + " "
            + msg;
    }
};