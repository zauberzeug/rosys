#pragma once

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
        if (nvs_open(namespace_.c_str(), NVS_READWRITE, &handle) != ESP_OK)
        {
            printf("Could not open storage namespace: %s\n", namespace_.c_str());
            return;
        }

        if (nvs_set_str(handle, key.c_str(), value.c_str()) != ESP_OK)
        {
            printf("Could write to storage: %s.%s=%s\n", namespace_.c_str(), key.c_str(), value.c_str());
            nvs_close(handle);
            return;
        }

        if (nvs_commit(handle) != ESP_OK)
        {
            printf("Could commit storage: %s.%s=%s\n", namespace_.c_str(), key.c_str(), value.c_str());
            nvs_close(handle);
            return;
        }

        nvs_close(handle);
    }

    std::string get(std::string namespace_, std::string key)
    {
        nvs_handle handle;
        if (nvs_open(namespace_.c_str(), NVS_READWRITE, &handle) != ESP_OK)
        {
            printf("Could not open storage namespace: %s\n", namespace_.c_str());
            return "";
        }

        size_t size = 0;
        if (nvs_get_str(handle, key.c_str(), NULL, &size) != ESP_OK)
        {
            printf("Could not peek storage: %s.%s\n", namespace_.c_str(), key.c_str());
            nvs_close(handle);
            return "";
        }

        char *value = (char *)malloc(size);
        if (size > 0)
        {
            if (nvs_get_str(handle, key.c_str(), value, &size) != ESP_OK)
            {
                printf("Could not read storage: %s.%s\n", namespace_.c_str(), key.c_str());
                free(value);
                nvs_close(handle);
                return "";
            }
        }

        std::string result = std::string(value);
        free(value);

        nvs_close(handle);

        return result;
    }

    void print(std::string namespace_, std::string key = "")
    {
        if (key.empty())
        {
            nvs_iterator_t it = nvs_entry_find("nvs", namespace_.c_str(), NVS_TYPE_ANY);
            nvs_entry_info_t info;
            while (it != NULL)
            {
                nvs_entry_info(it, &info);
                it = nvs_entry_next(it);
                print(info.namespace_name, info.key);
            };
        }
        else
        {
            std::string value = get(namespace_, key);
            printf("%s: %s\n", key.c_str(), value.c_str());
        }
    }

    void erase()
    {
        if (nvs_flash_erase() != ESP_OK)
        {
            printf("Could not erase storage\n");
        }
    }

    void erase(std::string namespace_, std::string key = "")
    {
        nvs_handle handle;
        if (nvs_open(namespace_.c_str(), NVS_READWRITE, &handle) != ESP_OK)
        {
            printf("Could not open storage namespace: %s\n", namespace_.c_str());
            return;
        }

        if (key.empty())
        {
            if (nvs_erase_all(handle) != ESP_OK)
            {
                printf("Could not erase namespace: %s\n", namespace_.c_str());
            }
        }
        else
        {
            if (nvs_erase_key(handle, key.c_str()) != ESP_OK)
            {
                printf("Could not erase key: %s.%s\n", namespace_.c_str(), key.c_str());
            }
        }

        nvs_close(handle);
    }
} // namespace storage