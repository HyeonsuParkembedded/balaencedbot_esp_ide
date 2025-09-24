/**
 * @file gps_sensor.c
 * @brief GPS 위성 위치 센서 드라이버 구현 파일
 * 
 * UART를 통해 GPS 모듈로부터 NMEA 0183 형식의 데이터를 수신하고
 * 파싱하여 위도, 경도, 고도 등의 위치 정보를 추출합니다.
 * 
 * @author BalanceBot Team
 * @date 2025-09-20
 * @version 1.0
 */

#include "gps_sensor.h"
#include "../bsw/uart_driver.h"
#ifndef NATIVE_BUILD
#include "esp_log.h"
#endif
#include <string.h>
#include <stdlib.h>

// Mock functions for native build
#ifndef NATIVE_BUILD
#else
char* strdup(const char* s) {
    size_t len = strlen(s) + 1;
    char* dup = malloc(len);
    if (dup) {
        memcpy(dup, s, len);
    }
    return dup;
}
#endif

#ifndef NATIVE_BUILD
static const char* GPS_TAG = "GPS_SENSOR";
#else
#define GPS_TAG "GPS_SENSOR"
#endif

/**
 * @brief 도분(degree-minute) 형식을 십진도(decimal degree) 형식으로 변환
 * @param deg_min 도분 형식의 좌표값 (DDMM.MMMM)
 * @return 십진도 형식의 좌표값
 */
static float convert_deg_min_to_dec_deg(float deg_min);

/**
 * @brief NMEA 0183 문장을 파싱하여 GPS 데이터 추출
 * @param gps GPS 센서 구조체 포인터
 * @param sentence NMEA 문장 문자열
 * @return 파싱 성공 시 true, 실패 시 false
 */
static bool parse_nmea(gps_sensor_t* gps, const char* sentence);

/**
 * @brief GPGGA 문장을 파싱하여 위치/고도 정보 추출
 * @param gps GPS 센서 구조체 포인터
 * @param sentence GPGGA NMEA 문장
 * @return 파싱 성공 시 true, 실패 시 false
 */
static bool parse_gpgga(gps_sensor_t* gps, const char* sentence);

/**
 * @brief GPRMC 문장을 파싱하여 수신 상태 확인
 * @param gps GPS 센서 구조체 포인터
 * @param sentence GPRMC NMEA 문장
 * @return 항상 false (위치 정보는 GPGGA에서만 업데이트)
 */
static bool parse_gprmc(gps_sensor_t* gps, const char* sentence);

/**
 * @brief GPS 센서를 초기화하고 UART 통신 설정
 * 
 * 지정된 UART 포트와 핀을 사용하여 GPS 모듈과의 통신을 설정합니다.
 * GPS 데이터 구조체를 초기화하고 UART 드라이버를 구성합니다.
 * 
 * @param gps GPS 센서 구조체 포인터
 * @param port 사용할 UART 포트 번호
 * @param tx_pin UART 송신 핀 번호
 * @param rx_pin UART 수신 핀 번호
 * @param baudrate UART 통신 속도 (일반적으로 9600)
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t gps_sensor_init(gps_sensor_t* gps, uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baudrate) {
    gps->uart_port = port;
    gps->data.latitude = 0.0;
    gps->data.longitude = 0.0;
    gps->data.altitude = 0.0f;
    gps->data.satellites = 0;
    gps->data.fix_valid = false;
    gps->data.initialized = false;

    esp_err_t ret = uart_driver_init(port, tx_pin, rx_pin, baudrate);
    if (ret != ESP_OK) {
        return ret;
    }

    gps->data.initialized = true;

#ifndef NATIVE_BUILD
    ESP_LOGI(GPS_TAG, "GPS sensor initialized");
#endif
    return ESP_OK;
}

/**
 * @brief GPS 센서 데이터를 업데이트하여 최신 위치 정보 수신
 * 
 * UART를 통해 GPS 모듈로부터 NMEA 문장을 읽어와 파싱합니다.
 * 수신된 데이터에서 GPGGA, GPRMC 문장을 추출하여 위치, 고도, 위성 수 등의
 * 정보를 업데이트합니다.
 * 
 * @param gps GPS 센서 구조체 포인터
 * @return ESP_OK 성공, ESP_FAIL 센서가 초기화되지 않음
 */
esp_err_t gps_sensor_update(gps_sensor_t* gps) {
    if (!gps->data.initialized) {
        return ESP_FAIL;
    }

    uint8_t buffer[256];
    int len = uart_read_data(gps->uart_port, buffer, sizeof(buffer) - 1, 100);

    if (len > 0) {
        buffer[len] = '\0';
        char* sentence = strtok((char*)buffer, "\r\n");

        while (sentence != NULL) {
            if (parse_nmea(gps, sentence)) {
                return ESP_OK;
            }
            sentence = strtok(NULL, "\r\n");
        }
    }

    return ESP_OK;
}

/**
 * @brief 현재 GPS 위도 좌표 반환
 * @param gps GPS 센서 구조체 포인터
 * @return 십진도 형식의 위도 (북위: 양수, 남위: 음수)
 */
double gps_sensor_get_latitude(gps_sensor_t* gps) {
    return gps->data.latitude;
}

/**
 * @brief 현재 GPS 경도 좌표 반환
 * @param gps GPS 센서 구조체 포인터
 * @return 십진도 형식의 경도 (동경: 양수, 서경: 음수)
 */
double gps_sensor_get_longitude(gps_sensor_t* gps) {
    return gps->data.longitude;
}

/**
 * @brief 현재 GPS 고도 반환
 * @param gps GPS 센서 구조체 포인터
 * @return 해수면 기준 고도 (미터)
 */
float gps_sensor_get_altitude(gps_sensor_t* gps) {
    return gps->data.altitude;
}

/**
 * @brief 현재 수신 중인 위성 개수 반환
 * @param gps GPS 센서 구조체 포인터
 * @return 수신 중인 GPS 위성 개수
 */
int gps_sensor_get_satellites(gps_sensor_t* gps) {
    return gps->data.satellites;
}

/**
 * @brief GPS 수신 상태 확인
 * @param gps GPS 센서 구조체 포인터
 * @return GPS 수신 성공 시 true, 실패 시 false
 */
bool gps_sensor_has_fix(gps_sensor_t* gps) {
    return gps->data.fix_valid;
}

/**
 * @brief GPS 센서 초기화 상태 확인
 * @param gps GPS 센서 구조체 포인터
 * @return 초기화 완료 시 true, 미완료 시 false
 */
bool gps_sensor_is_initialized(gps_sensor_t* gps) {
    return gps->data.initialized;
}

/**
 * @brief 도분(degree-minute) 형식을 십진도(decimal degree) 형식으로 변환
 * 
 * GPS NMEA 데이터에서 사용하는 DDMM.MMMM 형식을 표준 십진도 형식으로 변환합니다.
 * 예: 3723.2475 → 37.387458도
 * 
 * @param deg_min 도분 형식의 좌표값 (DDMM.MMMM)
 * @return 십진도 형식의 좌표값
 */
static float convert_deg_min_to_dec_deg(float deg_min) {
    int degrees = (int)(deg_min / 100);
    float minutes = deg_min - (degrees * 100);
    return degrees + (minutes / 60.0f);
}

/**
 * @brief NMEA 0183 문장을 파싱하여 GPS 데이터 추출
 * 
 * 수신된 NMEA 문장의 타입을 확인하고 해당하는 파서 함수를 호출합니다.
 * 현재 GPGGA(위치/고도)와 GPRMC(수신상태) 문장을 지원합니다.
 * 
 * @param gps GPS 센서 구조체 포인터
 * @param sentence NMEA 문장 문자열
 * @return 파싱 성공 및 데이터 업데이트 시 true, 그 외 false
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
 * 
 * GPGGA (Global Positioning System Fix Data) 문장에서 위도, 경도, 고도,
 * 위성 개수, GPS 품질 등의 정보를 추출하여 GPS 데이터 구조체에 저장합니다.
 * 
 * GPGGA 문장 형식:
 * $GPGGA,time,lat,lat_dir,lon,lon_dir,quality,satellites,hdop,altitude,M,height,M,time,checksum
 * 
 * @param gps GPS 센서 구조체 포인터
 * @param sentence GPGGA NMEA 문장
 * @return GPS 수신이 유효한 경우 true, 무효한 경우 false
 */
static bool parse_gpgga(gps_sensor_t* gps, const char* sentence) {
    char* copy = strdup(sentence);
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
 * 
 * GPRMC (Recommended Minimum Course) 문장에서 GPS 수신 유효성을 확인합니다.
 * 'A'는 유효한 수신, 'V'는 무효한 수신을 나타냅니다.
 * 위치 정보는 GPGGA에서만 업데이트하므로 이 함수는 상태 확인용입니다.
 * 
 * @param gps GPS 센서 구조체 포인터
 * @param sentence GPRMC NMEA 문장
 * @return 항상 false (위치 정보는 GPGGA에서만 업데이트)
 */
static bool parse_gprmc(gps_sensor_t* gps, const char* sentence) {
    // Simplified GPRMC parsing - mainly for fix validity
    char* copy = strdup(sentence);
    char* token = strtok(copy, ",");
    int field = 0;

    while (token != NULL && field < 3) {
        if (field == 2) {
            gps->data.fix_valid = (token[0] == 'A');
            break;
        }
        token = strtok(NULL, ",");
        field++;
    }

    free(copy);
    return false; // Don't update position from RMC
}