#pragma once

namespace mcp
{

    void init();

    void set_mode(int bank, int number, bool input);

    void set_pullup(int bank, int number, bool value);

    void write_bit(int bank, int number, int level);

    int read_bit(int bank, int number);

} // namespace mcp