#include "checksum.h"

#include <string>
#include <cstdarg>

uint8_t checksum = 0;

void _cprint(const char *fmt, va_list args)
{
    char buf[256];
    std::vsnprintf(buf, sizeof buf, fmt, args);
    for (unsigned int i = 0; i < sizeof buf and buf[i] > 0; ++i)
    {
        checksum ^= buf[i];
    }
    printf(buf);
}

void cprint(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _cprint(fmt, args);
    va_end(args);
}

void cprintln(const char *fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    _cprint(fmt, args);
    va_end(args);

    printf("^%d\n", checksum);
    checksum = 0;
}