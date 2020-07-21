#include <string>
#include "nvs_flash.h"

namespace storage
{
    void init()
    {
        nvs_flash_init();
    }

    void put(std::string namespace_, std::string key, std::string value)
    {
        nvs_handle handle;
        nvs_open(namespace_.c_str(), NVS_READWRITE, &handle);
        nvs_set_str(handle, key.c_str(), value.c_str());
        nvs_commit(handle);
        nvs_close(handle);
    }

    void erase()
    {
        nvs_flash_erase();
    }
} // namespace storage