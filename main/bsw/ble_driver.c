/**
 * @file ble_driver.c
 * @brief BLE Driver Implementation using NimBLE Stack
 * 
 * NimBLE 스택을 사용한 ESP32-C6 BLE 드라이버 구현입니다.
 * Apache Mynewt의 검증된 BLE 스택을 활용하여 안정적인 통신을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 2.0 (NimBLE)
 */

#include "ble_driver.h"
#include "system_services.h"
#include <string.h>

// NimBLE Stack Includes (ESP-IDF v5.5)
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

static const char* TAG = "BLE_DRIVER";

// BLE Driver Context
typedef struct {
    bool initialized;
    bool advertising;
    char device_name[32];
    ble_event_callback_t event_callback;
    void* user_data;
    uint16_t conn_handle;
} ble_context_t;

static ble_context_t g_ble_ctx = {0};

// GATT Service and Characteristic Storage
#define MAX_SERVICES 8
#define MAX_CHARACTERISTICS 32

typedef struct {
    ble_service_handle_t handle;
    bsw_ble_uuid_t uuid;
    bool in_use;
    struct ble_gatt_svc_def* gatt_svc;
} service_info_t;

typedef struct {
    ble_char_handle_t handle;
    ble_service_handle_t service_handle;
    bsw_ble_uuid_t uuid;
    ble_char_properties_t properties;
    bool in_use;
    uint16_t val_handle;
    struct ble_gatt_chr_def* gatt_chr;
} char_info_t;

static service_info_t g_services[MAX_SERVICES] = {0};
static char_info_t g_characteristics[MAX_CHARACTERISTICS] = {0};
static struct ble_gatt_svc_def g_gatt_services[MAX_SERVICES + 1];  // +1 for NULL terminator
static struct ble_gatt_chr_def g_gatt_chars[MAX_SERVICES][MAX_CHARACTERISTICS / MAX_SERVICES + 1];

static uint16_t g_next_service_handle = 1;
static uint16_t g_next_char_handle = 1;

// Forward declarations
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg);
static int ble_gatt_access_callback(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg);
static void ble_on_sync(void);
static void ble_on_reset(int reason);
static void ble_host_task(void *param);
static service_info_t* find_service_by_handle(ble_service_handle_t handle);
static char_info_t* find_char_by_handle(ble_char_handle_t handle);

/**
 * @brief NimBLE sync callback
 */
static void ble_on_sync(void)
{
    BSW_LOGI(TAG, "NimBLE stack synchronized");
    
    // Set device address - must provide valid address type pointer
    uint8_t addr_type = 0;
    int rc = ble_hs_id_infer_auto(0, &addr_type);
    if (rc != 0) {
        BSW_LOGE(TAG, "Failed to determine address type: %d", rc);
        return;
    }
    
    BSW_LOGI(TAG, "BLE address type determined: %d", addr_type);
}

/**
 * @brief NimBLE reset callback
 */
static void ble_on_reset(int reason)
{
    BSW_LOGW(TAG, "NimBLE stack reset, reason: %d", reason);
}

/**
 * @brief GAP event handler
 */
static int ble_gap_event_handler(struct ble_gap_event *event, void *arg)
{
    switch (event->type) {
        case BLE_GAP_EVENT_CONNECT:
            BSW_LOGI(TAG, "Connection %s; status=%d",
                    event->connect.status == 0 ? "established" : "failed",
                    event->connect.status);
            
            if (event->connect.status == 0) {
                g_ble_ctx.conn_handle = event->connect.conn_handle;
                
                if (g_ble_ctx.event_callback) {
                    ble_event_t evt = {0};
                    evt.type = BLE_EVENT_CONNECTED;
                    evt.conn_handle = event->connect.conn_handle;
                    g_ble_ctx.event_callback(&evt, g_ble_ctx.user_data);
                }
            }
            break;
            
        case BLE_GAP_EVENT_DISCONNECT:
            BSW_LOGI(TAG, "Disconnect; reason=%d", event->disconnect.reason);
            g_ble_ctx.conn_handle = BLE_HS_CONN_HANDLE_NONE;
            
            if (g_ble_ctx.event_callback) {
                ble_event_t evt = {0};
                evt.type = BLE_EVENT_DISCONNECTED;
                evt.conn_handle = event->disconnect.conn.conn_handle;
                g_ble_ctx.event_callback(&evt, g_ble_ctx.user_data);
            }
            
            // Restart advertising
            if (g_ble_ctx.advertising) {
                ble_start_advertising();
            }
            break;
            
        case BLE_GAP_EVENT_ADV_COMPLETE:
            BSW_LOGI(TAG, "Advertising complete");
            g_ble_ctx.advertising = false;
            break;
            
        case BLE_GAP_EVENT_SUBSCRIBE:
            BSW_LOGI(TAG, "Subscribe event; attr_handle=%d, subscribed=%d",
                    event->subscribe.attr_handle,
                    event->subscribe.cur_notify || event->subscribe.cur_indicate);
            break;
            
        default:
            break;
    }
    
    return 0;
}

/**
 * @brief GATT characteristic access callback
 */
static int ble_gatt_access_callback(uint16_t conn_handle, uint16_t attr_handle,
                                    struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    char_info_t* characteristic = (char_info_t*)arg;
    
    if (!characteristic) {
        return BLE_ATT_ERR_UNLIKELY;
    }
    
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            BSW_LOGI(TAG, "GATT read request for char handle %d", characteristic->handle);
            // TODO: Implement read logic if needed
            return 0;
            
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            BSW_LOGI(TAG, "GATT write request for char handle %d, len=%d", 
                    characteristic->handle, ctxt->om->om_len);
            
            if (g_ble_ctx.event_callback) {
                ble_event_t evt = {0};
                evt.type = BLE_EVENT_DATA_RECEIVED;
                evt.conn_handle = conn_handle;
                evt.data_received.char_handle = characteristic->handle;
                evt.data_received.data = ctxt->om->om_data;
                evt.data_received.len = ctxt->om->om_len;
                g_ble_ctx.event_callback(&evt, g_ble_ctx.user_data);
            }
            return 0;
            
        default:
            return BLE_ATT_ERR_UNLIKELY;
    }
}

/**
 * @brief NimBLE host task
 */
static void ble_host_task(void *param)
{
    nimble_port_run();
    nimble_port_freertos_deinit();
}

/**
 * @brief BLE driver initialization
 */
bool ble_driver_init(const char* device_name, ble_event_callback_t callback, void* user_data)
{
    if (g_ble_ctx.initialized) {
        BSW_LOGW(TAG, "BLE already initialized");
        return true;
    }
    
    if (!device_name || !callback) {
        BSW_LOGE(TAG, "Invalid parameters");
        return false;
    }
    
    // Initialize context
    memset(&g_ble_ctx, 0, sizeof(g_ble_ctx));
    strncpy(g_ble_ctx.device_name, device_name, sizeof(g_ble_ctx.device_name) - 1);
    g_ble_ctx.event_callback = callback;
    g_ble_ctx.user_data = user_data;
    g_ble_ctx.conn_handle = BLE_HS_CONN_HANDLE_NONE;
    
    // Initialize NimBLE (HCI init not needed in v5.5)
    
    // Initialize NimBLE host
    nimble_port_init();
    
    // Initialize services
    ble_svc_gap_init();
    ble_svc_gatt_init();
    
    // Set device name
    int ret = ble_svc_gap_device_name_set(device_name);
    if (ret != 0) {
        BSW_LOGE(TAG, "Failed to set device name: %d", ret);
        return false;
    }
    
    // Configure host callbacks
    ble_hs_cfg.sync_cb = ble_on_sync;
    ble_hs_cfg.reset_cb = ble_on_reset;
    
    // Start NimBLE host task
    nimble_port_freertos_init(ble_host_task);
    
    g_ble_ctx.initialized = true;
    BSW_LOGI(TAG, "BLE driver initialized with device name: %s", device_name);
    
    return true;
}

/**
 * @brief Create BLE service
 */
bool ble_create_service(const bsw_ble_uuid_t* service_uuid, ble_service_handle_t* service_handle)
{
    if (!g_ble_ctx.initialized || !service_uuid || !service_handle) {
        BSW_LOGE(TAG, "Invalid parameters for service creation");
        return false;
    }
    
    // Find empty service slot
    service_info_t* service = NULL;
    int service_idx = -1;
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (!g_services[i].in_use) {
            service = &g_services[i];
            service_idx = i;
            break;
        }
    }
    
    if (!service) {
        BSW_LOGE(TAG, "No available service slots");
        return false;
    }
    
    // Initialize service
    service->handle = g_next_service_handle++;
    service->uuid = *service_uuid;
    service->in_use = true;
    service->gatt_svc = &g_gatt_services[service_idx];
    
    // Initialize GATT service definition
    memset(service->gatt_svc, 0, sizeof(struct ble_gatt_svc_def));
    
    if (service_uuid->type == 0) {
        // 16-bit UUID
        service->gatt_svc->uuid = BLE_UUID16_DECLARE(service_uuid->uuid16);
    } else {
        // 128-bit UUID - needs static storage
        static ble_uuid128_t uuid128;
        memcpy(uuid128.value, service_uuid->uuid128, 16);
        uuid128.u.type = BLE_UUID_TYPE_128;
        service->gatt_svc->uuid = &uuid128.u;
    }
    
    service->gatt_svc->type = BLE_GATT_SVC_TYPE_PRIMARY;
    service->gatt_svc->characteristics = g_gatt_chars[service_idx];
    
    // Initialize characteristics array with NULL terminator
    memset(g_gatt_chars[service_idx], 0, sizeof(g_gatt_chars[service_idx]));
    
    *service_handle = service->handle;
    BSW_LOGI(TAG, "Service created: handle=%d", service->handle);
    
    return true;
}

/**
 * @brief Add characteristic to service
 */
bool ble_add_characteristic(ble_service_handle_t service_handle,
                            const bsw_ble_uuid_t* char_uuid,
                            const ble_char_properties_t* properties,
                            ble_char_handle_t* char_handle)
{
    if (!g_ble_ctx.initialized || !char_uuid || !char_handle) {
        BSW_LOGE(TAG, "Invalid parameters for characteristic");
        return false;
    }
    
    // Find service
    service_info_t* service = find_service_by_handle(service_handle);
    if (!service) {
        BSW_LOGE(TAG, "Service not found: %d", service_handle);
        return false;
    }
    
    // Find empty characteristic slot
    char_info_t* characteristic = NULL;
    for (int i = 0; i < MAX_CHARACTERISTICS; i++) {
        if (!g_characteristics[i].in_use) {
            characteristic = &g_characteristics[i];
            break;
        }
    }
    
    if (!characteristic) {
        BSW_LOGE(TAG, "No available characteristic slots");
        return false;
    }
    
    // Find position in service's characteristics array
    int char_idx = 0;
    while (service->gatt_svc->characteristics[char_idx].uuid != NULL) {
        char_idx++;
    }
    
    // Initialize characteristic
    characteristic->handle = g_next_char_handle++;
    characteristic->service_handle = service_handle;
    characteristic->uuid = *char_uuid;
    characteristic->properties = *properties;
    characteristic->in_use = true;
    characteristic->gatt_chr = (struct ble_gatt_chr_def*)&service->gatt_svc->characteristics[char_idx];
    
    // Convert properties to NimBLE flags
    uint8_t flags = 0;
    if (properties->read) flags |= BLE_GATT_CHR_F_READ;
    if (properties->write) flags |= BLE_GATT_CHR_F_WRITE;
    if (properties->notify) flags |= BLE_GATT_CHR_F_NOTIFY;
    if (properties->indicate) flags |= BLE_GATT_CHR_F_INDICATE;
    
    // Initialize GATT characteristic definition
    memset(characteristic->gatt_chr, 0, sizeof(struct ble_gatt_chr_def));
    
    if (char_uuid->type == 0) {
        // 16-bit UUID
        characteristic->gatt_chr->uuid = BLE_UUID16_DECLARE(char_uuid->uuid16);
    } else {
        // 128-bit UUID
        static ble_uuid128_t uuid128[MAX_CHARACTERISTICS];
        static int uuid_idx = 0;
        memcpy(uuid128[uuid_idx].value, char_uuid->uuid128, 16);
        uuid128[uuid_idx].u.type = BLE_UUID_TYPE_128;
        characteristic->gatt_chr->uuid = &uuid128[uuid_idx++].u;
    }
    
    characteristic->gatt_chr->access_cb = ble_gatt_access_callback;
    characteristic->gatt_chr->arg = characteristic;
    characteristic->gatt_chr->flags = flags;
    
    *char_handle = characteristic->handle;
    BSW_LOGI(TAG, "Characteristic added: handle=%d, service=%d", 
            characteristic->handle, service_handle);
    
    return true;
}

/**
 * @brief Start BLE service
 */
bool ble_start_service(ble_service_handle_t service_handle)
{
    if (!g_ble_ctx.initialized) {
        BSW_LOGE(TAG, "BLE not initialized");
        return false;
    }
    
    service_info_t* service = find_service_by_handle(service_handle);
    if (!service) {
        BSW_LOGE(TAG, "Service not found: %d", service_handle);
        return false;
    }
    
    // Prepare GATT services array with NULL terminator
    int svc_count = 0;
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (g_services[i].in_use) {
            g_gatt_services[svc_count++] = *g_services[i].gatt_svc;
        }
    }
    memset(&g_gatt_services[svc_count], 0, sizeof(struct ble_gatt_svc_def));
    
    // Register GATT services
    int rc = ble_gatts_count_cfg(g_gatt_services);
    if (rc != 0) {
        BSW_LOGE(TAG, "Failed to count GATT config: %d", rc);
        return false;
    }
    
    rc = ble_gatts_add_svcs(g_gatt_services);
    if (rc != 0) {
        BSW_LOGE(TAG, "Failed to add GATT services: %d", rc);
        return false;
    }
    
    BSW_LOGI(TAG, "Service started: handle=%d", service_handle);
    return true;
}

/**
 * @brief Start BLE advertising
 */
bool ble_start_advertising(void)
{
    if (!g_ble_ctx.initialized) {
        BSW_LOGE(TAG, "BLE not initialized");
        return false;
    }
    
    struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;
    
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &adv_params, ble_gap_event_handler, NULL);
    if (rc != 0) {
        BSW_LOGE(TAG, "Failed to start advertising: %d", rc);
        return false;
    }
    
    g_ble_ctx.advertising = true;
    BSW_LOGI(TAG, "Advertising started");
    
    return true;
}

/**
 * @brief Stop BLE advertising
 */
bool ble_stop_advertising(void)
{
    if (!g_ble_ctx.initialized) {
        return false;
    }
    
    int rc = ble_gap_adv_stop();
    if (rc != 0) {
        BSW_LOGE(TAG, "Failed to stop advertising: %d", rc);
        return false;
    }
    
    g_ble_ctx.advertising = false;
    BSW_LOGI(TAG, "Advertising stopped");
    
    return true;
}

/**
 * @brief Send data to client
 */
bool ble_send_data(ble_conn_handle_t conn_handle,
                   ble_char_handle_t char_handle,
                   const uint8_t* data,
                   size_t length)
{
    if (!g_ble_ctx.initialized || !data || length == 0) {
        BSW_LOGE(TAG, "Invalid parameters for send");
        return false;
    }
    
    char_info_t* characteristic = find_char_by_handle(char_handle);
    if (!characteristic) {
        BSW_LOGE(TAG, "Characteristic not found: %d", char_handle);
        return false;
    }
    
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, length);
    if (!om) {
        BSW_LOGE(TAG, "Failed to allocate mbuf");
        return false;
    }
    
    // Use GATT server notify (not client)
    int rc = ble_gatts_notify_custom(conn_handle, characteristic->val_handle, om);
    
    if (rc != 0) {
        BSW_LOGE(TAG, "Failed to send data: %d", rc);
        return false;
    }
    
    BSW_LOGI(TAG, "Data sent: conn=%d, char=%d, len=%zu", conn_handle, char_handle, length);
    return true;
}

/**
 * @brief Check if connected
 */
bool ble_is_connected(ble_conn_handle_t conn_handle)
{
    return (g_ble_ctx.conn_handle == conn_handle && 
            g_ble_ctx.conn_handle != BLE_HS_CONN_HANDLE_NONE);
}

/**
 * @brief Create 16-bit UUID
 */
bsw_ble_uuid_t ble_uuid_from_16(uint16_t uuid16)
{
    bsw_ble_uuid_t uuid = {0};
    uuid.type = 0;  // 16-bit
    uuid.uuid16 = uuid16;
    return uuid;
}

/**
 * @brief Create 128-bit UUID
 */
bsw_ble_uuid_t ble_uuid_from_128(const uint8_t uuid128[16])
{
    bsw_ble_uuid_t uuid = {0};
    uuid.type = 1;  // 128-bit
    memcpy(uuid.uuid128, uuid128, 16);
    return uuid;
}

// Helper functions
static service_info_t* find_service_by_handle(ble_service_handle_t handle)
{
    for (int i = 0; i < MAX_SERVICES; i++) {
        if (g_services[i].in_use && g_services[i].handle == handle) {
            return &g_services[i];
        }
    }
    return NULL;
}

static char_info_t* find_char_by_handle(ble_char_handle_t handle)
{
    for (int i = 0; i < MAX_CHARACTERISTICS; i++) {
        if (g_characteristics[i].in_use && g_characteristics[i].handle == handle) {
            return &g_characteristics[i];
        }
    }
    return NULL;
}
