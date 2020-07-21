#include <string>
#include "nvs_flash.h"

#include "modules/Configure.h"

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

    std::string get(std::string namespace_, std::string key)
    {
        nvs_handle handle;
        nvs_open(namespace_.c_str(), NVS_READWRITE, &handle);
        size_t size = 0;
        nvs_get_str(handle, key.c_str(), NULL, &size);
        char *buffer = (char *)malloc(size);
        nvs_get_str(handle, key.c_str(), buffer, &size);
        std::string value = std::string(buffer);
        free(buffer);
        nvs_close(handle);
        return value;
    }

    void erase()
    {
        nvs_flash_erase();
    }
} // namespace storage