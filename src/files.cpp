/*
 * files.cpp - SPIFFS file system helper functions
 * Copyright (C) 2025 TEEP Project, based on VO2Max-500pa by Lauri Peltonen
 * GPL V3
 */

#include "FS.h"
#include "SPIFFS.h"
#include "esp_log.h"

#include "files.h"
#include "config.h"

static const char *TAG = "FS";
static bool fs_ok = false;
static const char *storeFileName = "/data_log.dat";

bool init_filesystem(void) {
    ESP_LOGI(TAG, "Initializing filesystem...");
    if (!SPIFFS.begin(true)) {
        ESP_LOGE(TAG, "Could not initialize filesystem");
        fs_ok = false;
        return false;
    }
    ESP_LOGI(TAG, "Total: %d, Used: %d", SPIFFS.totalBytes(), SPIFFS.usedBytes());
    fs_ok = true;
    return true;
}

bool write_log_buffer_to_file(const char *buffer, size_t length, size_t block_size, unsigned int position) {
    if (!fs_ok) return false;

    File fp = SPIFFS.open(storeFileName, "wb", true);
    if (!fp) {
        ESP_LOGE(TAG, "Could not open file");
        return false;
    }

    bool ok = true;
    ok &= fp.write(DATA_FILE_MAGIC) == 1;
    ok &= fp.write(DATA_FILE_VERSION) == 1;
    ok &= fp.write((const uint8_t *)&length, sizeof(size_t)) == sizeof(size_t);
    ok &= fp.write((const uint8_t *)&global_settings.storeDataRate, sizeof(int)) == sizeof(int);
    ok &= fp.write((const uint8_t *)&global_settings.integrationTime, sizeof(int)) == sizeof(int);

    // Write oldest data first (from position to end)
    size_t to_write = length - position;
    if (to_write > 0)
        ok &= fp.write((const uint8_t *)&buffer[position * block_size], to_write * block_size) == to_write * block_size;

    // Then newest data (from beginning to position)
    to_write = length - to_write;
    if (to_write > 0)
        ok &= fp.write((const uint8_t *)&buffer[0], to_write * block_size) == to_write * block_size;

    fp.close();

    if (!ok) ESP_LOGE(TAG, "File write incomplete");
    else ESP_LOGI(TAG, "File written successfully");

    return ok;
}

static File read_file;

bool open_data_file(void) {
    read_file = SPIFFS.open(storeFileName, "rb", false);
    return (bool)read_file;
}

void close_data_file(void) {
    read_file.close();
}

bool read_data_file(char *buffer, size_t bytes) {
    return read_file.readBytes(buffer, bytes) == bytes;
}
