#pragma once

#include "mcp23017.h"

namespace mcp {

    void init();

    void write_bit(int bank, int number, int level);

}