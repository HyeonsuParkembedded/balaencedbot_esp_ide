/**
 * @file gps_sensor.h
 * @brief GPS Sensor Driver Header
 */

#ifndef GPS_SENSOR_H
#define GPS_SENSOR_H

#include <stdbool.h>
#include "driver/uart.h"

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief GPS Sensor Handle Structure
 */
typedef struct {
    struct {
        bool initialized;
        double latitude;
        double longitude;
        float altitude;
        float course;
        float speed_kmh;
        int satellites;
        bool fix_valid;
    } data;
    uart_port_t uart_port;
} gps_sensor_t;

/**
 * @brief Initialize GPS sensor
 * @param gps Pointer to GPS sensor handle
 * @param uart_num UART port number
 * @param tx_pin TX GPIO pin
 * @param rx_pin RX GPIO pin
 * @param baud_rate Baud rate
 * @return ESP_OK on success
 */
esp_err_t gps_sensor_init(gps_sensor_t* gps, uart_port_t uart_num, int tx_pin, int rx_pin, int baud_rate);

/**
 * @brief Update GPS sensor data
 * @param gps GPS sensor handle
 * @return ESP_OK on success
 */
esp_err_t gps_sensor_update(gps_sensor_t* gps);

/**
 * @brief Get Latitude
 * @param gps GPS sensor handle
 * @return double Latitude in degrees
 */
double gps_sensor_get_latitude(gps_sensor_t* gps);

/**
 * @brief Get Longitude
 * @param gps GPS sensor handle
 * @return double Longitude in degrees
 */
double gps_sensor_get_longitude(gps_sensor_t* gps);

/**
 * @brief Get Altitude
 * @param gps GPS sensor handle
 * @return float Altitude in meters
 */
float gps_sensor_get_altitude(gps_sensor_t* gps);

/**
 * @brief Get Course
 * @param gps GPS sensor handle
 * @return float Course in degrees
 */
float gps_sensor_get_course(gps_sensor_t* gps);

/**
 * @brief Get Speed
 * @param gps GPS sensor handle
 * @return float Speed in km/h
 */
float gps_sensor_get_speed(gps_sensor_t* gps);

/**
 * @brief Get Satellite Count
 * @param gps GPS sensor handle
 * @return int Number of satellites
 */
int gps_sensor_get_satellites(gps_sensor_t* gps);

/**
 * @brief Check if GPS has fix
 * @param gps GPS sensor handle
 * @return true if fix is valid
 */
bool gps_sensor_has_fix(gps_sensor_t* gps);

/**
 * @brief Check if GPS is initialized
 * @param gps GPS sensor handle
 * @return true if initialized
 */
bool gps_sensor_is_initialized(gps_sensor_t* gps);

#ifdef __cplusplus
}
#endif

#endif // GPS_SENSOR_H