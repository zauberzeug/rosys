#pragma once

#include <string>
#include "nvs_flash.h"

#include "utils/strings.h"

#define NAMESPACE "storage"
#define KEY "main"

namespace storage
{
    void init()
    {
        nvs_flash_init();
    }

    void put(std::string value)
    {
        nvs_handle handle;
        if (nvs_open(NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        {
            printf("Could not open storage namespace: %s\n", NAMESPACE);
            return;
        }

        if (nvs_set_str(handle, KEY, value.c_str()) != ESP_OK)
        {
            printf("Could write to storage: %s.%s=%s\n", NAMESPACE, KEY, value.c_str());
            nvs_close(handle);
            return;
        }

        if (nvs_commit(handle) != ESP_OK)
        {
            printf("Could commit storage: %s.%s=%s\n", NAMESPACE, KEY, value.c_str());
            nvs_close(handle);
            return;
        }

        nvs_close(handle);
    }

    std::string get()
    {
        nvs_handle handle;
        if (nvs_open(NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        {
            printf("Could not open storage namespace: %s\n", NAMESPACE);
            return "";
        }

        size_t size = 0;
        if (nvs_get_str(handle, KEY, NULL, &size) != ESP_OK)
        {
            printf("Could not peek storage: %s.%s\n", NAMESPACE, KEY);
            nvs_close(handle);
            return "";
        }

        char *value = (char *)malloc(size);
        if (size > 0)
        {
            if (nvs_get_str(handle, KEY, value, &size) != ESP_OK)
            {
                printf("Could not read storage: %s.%s\n", NAMESPACE, KEY);
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

    void append(std::string line)
    {
        std::string content = get();
        content += line + '\n';
        put(content);
    }

    void print(std::string substring = "")
    {
        std::string content = get();
        while (!content.empty())
        {
            std::string line = cut_first_word(content, '\n');
            if (starts_with(line, substring))
                printf("%s\n", line.c_str());
        }
    }

    void remove(std::string substring = "")
    {
        std::string content = get();
        std::string newContent = "";
        while (!content.empty())
        {
            std::string line = cut_first_word(content, '\n');
            if (!starts_with(line, substring))
                newContent += line + '\n';
        }
        put(newContent);
    }
} // namespace storage