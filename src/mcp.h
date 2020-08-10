#pragma once

namespace mcp {

    void init();

    void set_pullup(int bank, int number, bool value);

    void write_bit(int bank, int number, int level);

    int read_bit(int bank, int number);

}