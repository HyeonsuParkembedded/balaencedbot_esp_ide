/**
 * @file ble_driver.h
 * @brief BLE Driver using NimBLE Stack
 * 
 * ESP32-C6 NimBLE-based BLE driver
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 2.0 (NimBLE)
 */

#ifndef BLE_DRIVER_H
#define BLE_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// Type definitions
typedef uint16_t ble_conn_handle_t;
typedef uint16_t ble_char_handle_t;
typedef uint16_t ble_service_handle_t;

/**
 * @brief BSW BLE UUID (avoid NimBLE type conflict)
 */
typedef struct {
    uint8_t type;
    union {
        uint16_t uuid16;
        uint8_t uuid128[16];
    };
} bsw_ble_uuid_t;

/**
 * @brief Characteristic properties
 */
typedef struct {
    bool read;
    bool write;
    bool notify;
    bool indicate;
} ble_char_properties_t;

/**
 * @brief BLE event types
 */
typedef enum {
    BLE_EVENT_CONNECTED,
    BLE_EVENT_DISCONNECTED,
    BLE_EVENT_DATA_RECEIVED,
    BLE_EVENT_MTU_UPDATE,
    BLE_EVENT_ADVERTISING_COMPLETE
} ble_event_type_t;

/**
 * @brief BLE event data
 */
typedef struct {
    ble_event_type_t type;
    ble_conn_handle_t conn_handle;
    union {
        struct {
            ble_char_handle_t char_handle;
            const uint8_t* data;
            size_t len;
        } data_received;
        struct {
            uint16_t mtu;
        } mtu_update;
    };
} ble_event_t;

/**
 * @brief BLE event callback
 */
typedef void (*ble_event_callback_t)(const ble_event_t* event, void* user_data);

/**
 * @brief Initialize BLE driver
 */
bool ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data);

/**
 * @brief Create BLE service
 */
bool ble_create_service(const bsw_ble_uuid_t* service_uuid, ble_service_handle_t* service_handle);

/**
 * @brief Add characteristic
 */
bool ble_add_characteristic(
    ble_service_handle_t service_handle,
    const bsw_ble_uuid_t* char_uuid,
    const ble_char_properties_t* properties,
    ble_char_handle_t* char_handle
);

/**
 * @brief Start service
 */
bool ble_start_service(ble_service_handle_t service_handle);

/**
 * @brief Start advertising
 */
bool ble_start_advertising(void);

/**
 * @brief Stop advertising
 */
bool ble_stop_advertising(void);

/**
 * @brief Send data (Notify)
 */
bool ble_send_data(ble_conn_handle_t conn_handle, ble_char_handle_t char_handle, const uint8_t* data, size_t length);

/**
 * @brief Create 16-bit UUID
 */
bsw_ble_uuid_t ble_uuid_from_16(uint16_t uuid16);

/**
 * @brief Create 128-bit UUID
 */
bsw_ble_uuid_t ble_uuid_from_128(const uint8_t uuid128[16]);

#ifdef __cplusplus
}
#endif

#endif // BLE_DRIVER_H