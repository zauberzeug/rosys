#pragma once

#include <string>
#include "nvs_flash.h"

#include "utils/strings.h"

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

    void append(std::string namespace_, std::string key, std::string line)
    {
        std::string content = get(namespace_, key);
        content += line + '\n';
        put(namespace_, key, content);
    }

    void print(std::string namespace_, std::string key, std::string substring = "")
    {
        std::string content = get(namespace_, key);
        while (!content.empty())
        {
            std::string line = cut_first_word(content, '\n');
            if (starts_with(line, substring))
                printf("%s\n", line.c_str());
        }
    }

    void remove(std::string namespace_, std::string key, std::string substring = "")
    {
        std::string content = get(namespace_, key);
        std::string newContent = "";
        while (!content.empty())
        {
            std::string line = cut_first_word(content, '\n');
            if (!starts_with(line, substring))
                newContent += line + '\n';
        }
        put(namespace_, key, newContent);
    }

    void erase()
    {
        if (nvs_flash_erase() != ESP_OK)
        {
            printf("Could not erase storage\n");
        }
    }
} // namespace storage