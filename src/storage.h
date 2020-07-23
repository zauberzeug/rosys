#include <string>
#include "nvs_flash.h"

#include "modules/Configure.h"

namespace storage
{
    std::map<std::string, std::string> dict;

    void init()
    {
        nvs_flash_init();
    }

    void put(std::string namespace_, std::string key, std::string value)
    {
        nvs_handle handle;
        if (nvs_open(namespace_.c_str(), NVS_READWRITE, &handle) != ESP_OK)
            return;

        if (nvs_set_str(handle, key.c_str(), value.c_str()) != ESP_OK)
        {
            nvs_close(handle);
            return;
        }

        if (nvs_commit(handle) != ESP_OK)
        {
            nvs_close(handle);
            return;
        }

        dict[key] = std::string(value);
        nvs_close(handle);
    }

    std::string get(std::string namespace_, std::string key)
    {
        if (dict.count(key) == 1)
            return dict[key];

        nvs_handle handle;
        if (nvs_open(namespace_.c_str(), NVS_READWRITE, &handle) != ESP_OK)
        {
            return "";
        }

        size_t size = 0;
        if (nvs_get_str(handle, key.c_str(), NULL, &size) != ESP_OK)
        {
            nvs_close(handle);
            return "";
        }

        char *value = (char *)malloc(size);
        if (size > 0)
        {
            if (nvs_get_str(handle, key.c_str(), value, &size) != ESP_OK)
            {
                free(value);
                nvs_close(handle);
                return "";
            }
        }

        std::string result = std::string(value);
        free(value);

        dict[key] = result;
        nvs_close(handle);

        return result;
    }

    void erase()
    {
        nvs_flash_erase();
    }
} // namespace storage