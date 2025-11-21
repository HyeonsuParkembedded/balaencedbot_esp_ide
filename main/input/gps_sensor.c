/**
 * @file gps_sensor.c
 * @brief GPS Sensor Driver Implementation
 * 
 * UART를 통해 GPS 모듈로부터 NMEA 0183 형식의 데이터를 수신하고
 * 파싱하여 위도, 경도, 고도 등의 위치 정보를 추출합니다.
 */

#include "gps_sensor.h"
#include "esp_log.h"
#include "../bsw/uart_driver.h"

#include <string.h>
#include <stdlib.h>


#define GPS_TAG "GPS_SENSOR"
#define BSW_LOGI(tag, fmt, ...) ESP_LOGI(tag, fmt, ##__VA_ARGS__)
#define BSW_LOGE(tag, fmt, ...) ESP_LOGE(tag, fmt, ##__VA_ARGS__)

// Static function prototypes
static float convert_deg_min_to_dec_deg(float deg_min);
static bool parse_nmea(gps_sensor_t* gps, const char* sentence);
static bool parse_gpgga(gps_sensor_t* gps, const char* sentence);
static bool parse_gprmc(gps_sensor_t* gps, const char* sentence);

/**
 * @brief Initialize GPS sensor
 */
esp_err_t gps_sensor_init(gps_sensor_t* gps, bsw_uart_num_t uart_num, bsw_gpio_num_t tx_pin, bsw_gpio_num_t rx_pin, int baud_rate) {
    if (gps == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    gps->uart_port = uart_num;
    gps->data.initialized = false;
    gps->data.fix_valid = false;
    gps->data.latitude = 0.0;
    gps->data.longitude = 0.0;
    gps->data.altitude = 0.0f;
    gps->data.course = 0.0f;
    gps->data.speed_kmh = 0.0f;
    gps->data.satellites = 0;

    // Initialize BSW UART driver
    esp_err_t ret = uart_driver_init(uart_num, baud_rate, tx_pin, rx_pin);
    if (ret != ESP_OK) {
        BSW_LOGE(GPS_TAG, "Failed to initialize UART driver");
        return ret;
    }

    gps->data.initialized = true;
    BSW_LOGI(GPS_TAG, "GPS sensor initialized");
    return ESP_OK;
}

/**
 * @brief GPS 센서 데이터를 업데이트하여 최신 위치 정보 수신
 */
esp_err_t gps_sensor_update(gps_sensor_t* gps) {
    if (!gps->data.initialized) {
        return ESP_FAIL;
    }

    // Static buffer to accumulate partial packets across calls
    static uint8_t line_buffer[128];
    static int line_idx = 0;
    
    uint8_t temp_buf[64];
    // Non-blocking read using BSW driver
    int len = uart_read_data(gps->uart_port, temp_buf, sizeof(temp_buf), 0);

    for (int i = 0; i < len; i++) {
        uint8_t c = temp_buf[i];
        
        // If buffer overflow, reset (safety)
        if (line_idx >= sizeof(line_buffer) - 1) {
            line_idx = 0; 
        }

        // Standard NMEA end character is '\n' (often \r\n)
        if (c == '\n') {
            line_buffer[line_idx] = '\0'; // Null terminate
            
            // Parse the complete line
            if (parse_nmea(gps, (char*)line_buffer)) {
                // If parsing successful, we could return ESP_OK here, 
                // but we might want to process all available data.
                // For now, let's continue processing.
            }
            
            line_idx = 0; // Reset for next line
        } else if (c != '\r') {
            // Accumulate char (ignore \r)
            line_buffer[line_idx++] = c;
        }
    }

    return ESP_OK;
}

/**
 * @brief 현재 GPS 위도 좌표 반환
 */
double gps_sensor_get_latitude(gps_sensor_t* gps) {
    return gps->data.latitude;
}

/**
 * @brief 현재 GPS 경도 좌표 반환
 */
double gps_sensor_get_longitude(gps_sensor_t* gps) {
    return gps->data.longitude;
}

/**
 * @brief 현재 GPS 고도 반환
 */
float gps_sensor_get_altitude(gps_sensor_t* gps) {
    return gps->data.altitude;
}

/**
 * @brief 이동 방향 반환
 */
float gps_sensor_get_course(gps_sensor_t* gps) {
    return gps->data.course;
}

/**
 * @brief 이동 속도 반환
 */
float gps_sensor_get_speed(gps_sensor_t* gps) {
    return gps->data.speed_kmh;
}

/**
 * @brief 현재 수신 중인 위성 개수 반환
 */
int gps_sensor_get_satellites(gps_sensor_t* gps) {
    return gps->data.satellites;
}

/**
 * @brief GPS 수신 상태 확인
 */
bool gps_sensor_has_fix(gps_sensor_t* gps) {
    return gps->data.fix_valid;
}

/**
 * @brief GPS 센서 초기화 상태 확인
 */
bool gps_sensor_is_initialized(gps_sensor_t* gps) {
    return gps->data.initialized;
}

/**
 * @brief 도분(degree-minute) 형식을 십진도(decimal degree) 형식으로 변환
 */
static float convert_deg_min_to_dec_deg(float deg_min) {
    int degrees = (int)(deg_min / 100);
    float minutes = deg_min - (degrees * 100);
    return degrees + (minutes / 60.0f);
}

/**
 * @brief NMEA 0183 문장을 파싱하여 GPS 데이터 추출
 */
static bool parse_nmea(gps_sensor_t* gps, const char* sentence) {
    if (strncmp(sentence, "$GPGGA", 6) == 0) {
        return parse_gpgga(gps, sentence);
    } else if (strncmp(sentence, "$GPRMC", 6) == 0) {
        return parse_gprmc(gps, sentence);
    }
    return false;
}

/**
 * @brief GPGGA 문장을 파싱하여 위치/고도 정보 추출
 */
static bool parse_gpgga(gps_sensor_t* gps, const char* sentence) {
    char* copy = strdup(sentence);
    if (copy == NULL) return false;
    
    char* token = strtok(copy, ",");
    int field = 0;

    float lat_raw = 0, lon_raw = 0;
    char lat_dir = 'N', lon_dir = 'E';
    int quality = 0;

    while (token != NULL && field < 15) {
        switch (field) {
            case 2: lat_raw = atof(token); break;
            case 3: lat_dir = token[0]; break;
            case 4: lon_raw = atof(token); break;
            case 5: lon_dir = token[0]; break;
            case 6: quality = atoi(token); break;
            case 7: gps->data.satellites = atoi(token); break;
            case 9: gps->data.altitude = atof(token); break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    if (quality > 0 && lat_raw != 0 && lon_raw != 0) {
        gps->data.latitude = convert_deg_min_to_dec_deg(lat_raw);
        gps->data.longitude = convert_deg_min_to_dec_deg(lon_raw);

        if (lat_dir == 'S') gps->data.latitude = -gps->data.latitude;
        if (lon_dir == 'W') gps->data.longitude = -gps->data.longitude;

        gps->data.fix_valid = true;
    } else {
        gps->data.fix_valid = false;
    }

    free(copy);
    return gps->data.fix_valid;
}

/**
 * @brief GPRMC 문장을 파싱하여 수신 상태 확인
 */
static bool parse_gprmc(gps_sensor_t* gps, const char* sentence) {
    char* copy = strdup(sentence);
    if (copy == NULL) return false;

    char* token = strtok(copy, ",");
    int field = 0;
    
    bool status_valid = false;
    float speed_knots = 0.0f;

    while (token != NULL && field < 12) {
        switch (field) {
            case 2:
                if (token[0] == 'A') status_valid = true;
                break;
            case 7: // Speed in knots
                speed_knots = atof(token);
                gps->data.speed_kmh = speed_knots * 1.852f; // Knots to km/h
                break;
            case 8: // True course
                gps->data.course = atof(token);
                break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    gps->data.fix_valid = status_valid; // Update fix_valid based on RMC status
    free(copy);
    return status_valid;
}