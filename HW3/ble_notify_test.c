/*
 * GattLib CCCD Writer Example
 *
 * This program is a modification based on the gattlib_read_write example.
 * Copyright (C) 2016-2024  Olivier Martin <olivier@labapart.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h> // Required for the sleep() function
#include "gattlib.h"

//-- Configuration: Set your device's details here
#define TARGET_DEVICE_NAME      "realme GT Neo2 5G"
#define TARGET_CHAR_UUID        "fff0"      // The characteristic to enable indications on
#define BLE_SCAN_TIMEOUT        20          // Scan for 20 seconds
#define PAIRING_WAIT_SECONDS    15          // ** NEW: Time to wait for PIN entry **
//--

// Global variables for thread synchronization and state management
static pthread_cond_t  m_connection_terminated      = PTHREAD_COND_INITIALIZER;
static pthread_mutex_t m_connection_terminated_lock = PTHREAD_MUTEX_INITIALIZER;
static gattlib_adapter_t* m_adapter;
static int m_terminate = 0;

/*
 * 【修改】強化這個回呼函式，讓它能印出收到的具體資料
 */
static void on_indication_received(const uuid_t* uuid, const uint8_t* data, size_t data_length, void* user_data) {
    char uuid_str[MAX_LEN_UUID_STR + 1];
    char buffer[data_length + 1];
    int i;

    // 將收到的 UUID 轉成字串
    gattlib_uuid_to_string(uuid, uuid_str, sizeof(uuid_str));

    printf("\n--- Notification/Indication Received ---\n");
    printf("From UUID: %s\n", uuid_str);
    printf("Data Length: %zu bytes\n", data_length);

    // 1. 以十六進位 (Hex) 格式印出原始資料
    printf("Raw Data (Hex): ");
    for (i = 0; i < data_length; i++) {
        printf("%02x ", data[i]);
    }
    printf("\n");

    // 2. 嘗試以字串格式印出資料 (如果手機端傳送的是文字)
    // 複製資料到新 buffer 並加上結尾符，確保安全
    memcpy(buffer, data, data_length);
    buffer[data_length] = '\0';
    printf("Data (String): %s\n", buffer);
    printf("--------------------------------------\n\n");
}


/*
 * Callback executed when the connection to the device is established.
 */
static void on_device_connect(gattlib_adapter_t* adapter, const char *dst, gattlib_connection_t* connection, int error, void* user_data) {
    if (error != GATTLIB_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to connect to the device. (error %d)\n", error);
        goto SIGNAL_TERMINATION;
    }

    printf("Successfully connected to %s\n", dst);

    // 【新增】註冊 Notification/Indication 的回呼函式
    // 這一步是告訴 gattlib 當有通知進來時，要去呼叫 on_indication_received
    printf("Registering notification handler...\n");
    gattlib_register_notification(connection, on_indication_received, NULL);
    // 【新增結束】

    uuid_t characteristic_uuid;
    int ret;

    // Convert the characteristic UUID string to a gattlib uuid_t object
    ret = gattlib_string_to_uuid(TARGET_CHAR_UUID, strlen(TARGET_CHAR_UUID), &characteristic_uuid);
    if (ret != GATTLIB_SUCCESS) {
        fprintf(stderr, "ERROR: Could not convert UUID string '%s'.\n", TARGET_CHAR_UUID);
        goto DISCONNECT;
    }

    printf("Enabling indications for characteristic %s...\n", TARGET_CHAR_UUID);

    // ** CLARIFICATION: gattlib_indication_start() correctly writes 0x0002 to the CCCD. **
    // The alternative, gattlib_notification_start(), would write 0x0001.
    ret = gattlib_notification_start(connection, &characteristic_uuid);

    if (ret == GATTLIB_SUCCESS) {
        printf("SUCCESS: Indications enabled. CCCD value is now 0x0002.\n");
        printf("The device is now listening for value changes from the server.\n");

        // ** NEW: Wait for a few seconds to allow time for pairing/PIN entry on the phone. **
        printf("Waiting for %d seconds to allow for pairing or testing...\n", PAIRING_WAIT_SECONDS);
        printf(">>> Please change the characteristic value on your phone now! <<<\n");
        sleep(PAIRING_WAIT_SECONDS);
        printf("Wait time finished.\n");

    } else if (ret == GATTLIB_NOT_FOUND) {
        fprintf(stderr, "ERROR: Could not find characteristic with UUID %s.\n", TARGET_CHAR_UUID);
    } else if (ret == GATTLIB_NOT_SUPPORTED) {
        fprintf(stderr, "ERROR: Characteristic %s does not support indications.\n", TARGET_CHAR_UUID);
    } else {
        fprintf(stderr, "ERROR: Failed to enable indications for UUID %s (error %d).\n", TARGET_CHAR_UUID, ret);
    }

DISCONNECT:
    // Disconnect from the device
    printf("Disconnecting...\n");
    gattlib_disconnect(connection, false /* wait_disconnection */);

SIGNAL_TERMINATION:
    // Signal the main thread that the BLE operation is complete
    pthread_mutex_lock(&m_connection_terminated_lock);
    m_terminate = 1;
    pthread_cond_signal(&m_connection_terminated);
    pthread_mutex_unlock(&m_connection_terminated_lock);
}

/*
 * Callback executed for each BLE device discovered during scanning.
 */
static void ble_discovered_device(gattlib_adapter_t* adapter, const char* addr, const char* name, void *user_data) {
    if (name == NULL) {
        return;
    }

    if (strcmp(name, TARGET_DEVICE_NAME) == 0) {
        printf("Found target device: '%s' with address %s\n", name, addr);
        gattlib_adapter_scan_disable(adapter);

        printf("Connecting to %s...\n", name);
        int ret = gattlib_connect(adapter, addr, GATTLIB_CONNECTION_OPTIONS_LEGACY_BT_SEC_MEDIUM, on_device_connect, NULL);
        if (ret != GATTLIB_SUCCESS) {
            fprintf(stderr, "ERROR: Failed to initiate connection to '%s'.\n", addr);

            pthread_mutex_lock(&m_connection_terminated_lock);
            m_terminate = 1;
            pthread_cond_signal(&m_connection_terminated);
            pthread_mutex_unlock(&m_connection_terminated_lock);
        }
    }
}

/*
 * Main task for the BLE worker thread.
 */
static void* ble_scan_task(void* arg) {
    printf("Scanning for device named '%s'...\n", TARGET_DEVICE_NAME);

    int ret = gattlib_adapter_scan_enable(m_adapter, ble_discovered_device, BLE_SCAN_TIMEOUT, NULL);
    if (ret != GATTLIB_SUCCESS) {
        fprintf(stderr, "ERROR: Failed to start scan.\n");
    }

    pthread_mutex_lock(&m_connection_terminated_lock);
    while (m_terminate == 0) {
        pthread_cond_wait(&m_connection_terminated, &m_connection_terminated_lock);
    }
    pthread_mutex_unlock(&m_connection_terminated_lock);

    printf("BLE task finished.\n");
    return NULL;
}

int main(int argc, char *argv[]) {
    int ret = gattlib_adapter_open(NULL, &m_adapter);
    if (ret) {
        fprintf(stderr, "ERROR: Failed to open bluetooth adapter.\n");
        return 1;
    }

    gattlib_mainloop(ble_scan_task, NULL);
    gattlib_adapter_close(m_adapter);

    printf("Program finished.\n");
    return 0;
}