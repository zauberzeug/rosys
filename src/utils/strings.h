#pragma once

#include <string>

std::string cut_first_word(std::string &msg, char delimiter = ' ')
{
    int space = msg.find(delimiter);
    std::string word = space < 0 ? msg : msg.substr(0, space);
    msg = space < 0 ? std::string() : msg.substr(space + 1);
    return word;
}