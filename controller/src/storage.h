#pragma once

#include <string>
#include "nvs_flash.h"

#include "utils/strings.h"
#include "utils/checksum.h"

#define NAMESPACE "storage"
#define KEY "main"

namespace storage
{
    void init()
    {
        nvs_flash_init();
    }

    void write(std::string ns, std::string key, std::string value)
    {
        nvs_handle handle;
        if (nvs_open(ns.c_str(), NVS_READWRITE, &handle) != ESP_OK)
        {
            cprintln("Could not open storage namespace: %s", ns.c_str());
            return;
        }

        if (nvs_set_str(handle, key.c_str(), value.c_str()) != ESP_OK)
        {
            cprintln("Could not write to storage: %s.%s=%s", ns.c_str(), key.c_str(), value.c_str());
            nvs_close(handle);
            return;
        }

        if (nvs_commit(handle) != ESP_OK)
        {
            cprintln("Could not commit storage: %s.%s=%s", ns.c_str(), key.c_str(), value.c_str());
            nvs_close(handle);
            return;
        }

        nvs_close(handle);
    }

    std::string read(std::string ns, std::string key)
    {
        nvs_handle handle;
        if (nvs_open(ns.c_str(), NVS_READWRITE, &handle) != ESP_OK)
        {
            cprintln("Could not open storage namespace: %s", ns.c_str());
            return "";
        }

        size_t size = 0;
        if (nvs_get_str(handle, key.c_str(), NULL, &size) != ESP_OK)
        {
            cprintln("Could not peek storage: %s.%s", ns.c_str(), key.c_str());
            nvs_close(handle);
            return "";
        }

        char *value = (char *)malloc(size);
        if (size > 0)
        {
            if (nvs_get_str(handle, key.c_str(), value, &size) != ESP_OK)
            {
                cprintln("Could not read storage: %s.%s", ns.c_str(), key.c_str());
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

    void put(std::string value)
    {
        write(NAMESPACE, KEY, value);
    }

    std::string get()
    {
        return read(NAMESPACE, KEY);
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
                cprintln(line.c_str());
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