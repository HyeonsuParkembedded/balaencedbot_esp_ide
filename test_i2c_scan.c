/**
 * @file test_i2c_scan.c
 * @brief I2C 버스 스캔 테스트 (센서 탐지)
 * 
 * I2C 버스에 연결된 모든 디바이스를 스캔하여 주소를 출력합니다.
 * MPU6050은 보통 0x68 또는 0x69에 응답합니다.
 */

#include "i2c_driver.h"
#include "system_services.h"

void test_i2c_scan(void) {
    BSW_LOGI("I2C_SCAN", "Starting I2C bus scan...");
    BSW_LOGI("I2C_SCAN", "Scanning addresses 0x03 to 0x77");
    
    int found_count = 0;
    
    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        // Try to write 0 bytes to test if device responds
        esp_err_t ret = i2c_write_raw(BSW_I2C_PORT_0, addr, NULL, 0);
        
        if (ret == ESP_OK) {
            BSW_LOGI("I2C_SCAN", "Found device at address 0x%02X", addr);
            found_count++;
            
            // Check if it's MPU6050
            if (addr == 0x68 || addr == 0x69) {
                BSW_LOGI("I2C_SCAN", "  -> This might be MPU6050!");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(1));  // Small delay between scans
    }
    
    if (found_count == 0) {
        BSW_LOGW("I2C_SCAN", "No I2C devices found!");
        BSW_LOGW("I2C_SCAN", "Check:");
        BSW_LOGW("I2C_SCAN", "  1. Sensor power (VCC = 3.3V)");
        BSW_LOGW("I2C_SCAN", "  2. Pull-up resistors (4.7kΩ on SDA/SCL)");
        BSW_LOGW("I2C_SCAN", "  3. Wiring connections");
    } else {
        BSW_LOGI("I2C_SCAN", "Scan complete: %d device(s) found", found_count);
    }
}
