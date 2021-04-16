#pragma once

#include <map>
#include <string>

#include "modules/Module.h"
#include "utils/strings.h"

struct part_t {
    std::string trigger;
    bool equality;
    int state;
};

class Condition
{
public:
    std::vector<part_t> parts;
    std::string msg;

    Condition(std::string line)
    {
        while (true) {
            std::string word = cut_first_word(line);
            if (word == "if" or word == "&&") {
                std::string trigger = cut_first_word(line);
                bool equality = cut_first_word(line) == "==";
                int state = atoi(cut_first_word(line).c_str());
                parts.push_back({ trigger, equality, state });
            }
            else {
                msg = word + " " + line;
                break;
            }
        };
    }

    bool is_true(std::map<std::string, Module *> *modules)
    {
        for (auto const &part : parts)
        {
            if ((part.state == (*modules)[part.trigger]->state) != part.equality)
                return false;
        }
        return true;
    }

    std::string to_string()
    {
        std::string result = "";
        for (auto const &part : parts)
        {
            result += "&& " + part.trigger + (part.equality ? " == " : " != ") + std::to_string(part.state);
        }
        return result.replace(0, 3, "if ");
    }
};