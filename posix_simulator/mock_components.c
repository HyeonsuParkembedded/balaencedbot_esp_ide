// Mock files for BalanceBot POSIX Simulator
// These are minimal implementations to satisfy the build system

#include "../mock_bsw.h"

// GPS Sensor Mock
bsw_err_t gps_sensor_init(void* gps) { 
    BSW_LOGI("MOCK_GPS", "GPS sensor initialized");
    return BSW_OK; 
}
bsw_err_t gps_sensor_update(void* gps) { return BSW_OK; }

// Encoder Sensor Mock  
bsw_err_t encoder_sensor_init(void* enc) {
    BSW_LOGI("MOCK_ENC", "Encoder sensor initialized");
    return BSW_OK;
}
float encoder_sensor_get_speed(void* enc) { return 0.0f; }

// Battery Sensor Mock
bsw_err_t battery_sensor_init(void* bat) {
    BSW_LOGI("MOCK_BAT", "Battery sensor initialized");
    return BSW_OK;
}
float battery_sensor_get_voltage(void* bat) { return 3.7f; }

// Motor Control Mock
bsw_err_t motor_control_init(void* motor) {
    BSW_LOGI("MOCK_MOT", "Motor control initialized");
    return BSW_OK;
}
bsw_err_t motor_control_set_speed(void* motor, float speed) {
    BSW_LOGD("MOCK_MOT", "Motor speed set to %.2f", speed);
    return BSW_OK;
}
bsw_err_t motor_control_stop(void* motor) {
    BSW_LOGI("MOCK_MOT", "Motor stopped");
    return BSW_OK;
}

// BLE Controller Mock
bsw_err_t ble_controller_init(void* ble) {
    BSW_LOGI("MOCK_BLE", "BLE controller initialized");
    return BSW_OK;
}
bsw_err_t ble_controller_update(void* ble) { return BSW_OK; }

// Servo Standup Mock
bsw_err_t servo_standup_init(void* servo) {
    BSW_LOGI("MOCK_SERVO", "Servo standup initialized");
    return BSW_OK;
}
bsw_err_t servo_standup_update(void* servo) { return BSW_OK; }
bool servo_standup_is_standing_up(void* servo) { return false; }

// Error Recovery Mock
bsw_err_t error_recovery_init(void) {
    BSW_LOGI("MOCK_ERR", "Error recovery initialized");
    return BSW_OK;
}

// Config Manager Mock  
bsw_err_t config_manager_init(void) {
    BSW_LOGI("MOCK_CFG", "Config manager initialized");
    return BSW_OK;
}