/**
 * @file gpio_driver.c
 * @brief BSW GPIO 드라이버 구현 파일
 * 
 * GPIO 핀드라이버 직접 레지스터 제어한 고성능 GPIO 드라이버 구현입니다
 * ESP32-C6의 GPIO 및 IO_MUX 레지스터를 직접 조작하여 최적화된 성능을 제공합니다
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-04
 * @version 3.0 (FreeRTOS Multitasking Safe)
 */

#include "gpio_driver.h"
#include "system_services.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"
#include "esp_intr_alloc.h"
#include "soc/interrupts.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

static const char* TAG = "BSW_GPIO";

// FreeRTOS Mutex for GPIO thread safety
static SemaphoreHandle_t gpio_mutex = NULL;
static bool gpio_initialized = false;

/**
 * @brief GPIO 핀 번호 유효성 검사(Flash 메모리 보호 위해)
 * 
 * @param gpio_num 검증할 GPIO 핀 번호
 * @return ESP_OK: 사용 가능, ESP_ERR_INVALID_ARG: 범위 초과, ESP_ERR_NOT_SUPPORTED: Flash 사용 불가
 * 
 * @note ESP32-C6에서 GPIO 26-30은 Flash 연결용이며 사용시 접근 불가
 */
static inline esp_err_t validate_gpio_num(bsw_gpio_num_t gpio_num) {
    if (gpio_num >= BSW_GPIO_PIN_COUNT) {
        BSW_LOGE(TAG, "GPIO %d exceeds maximum pin count %d", gpio_num, BSW_GPIO_PIN_COUNT);
        return ESP_ERR_INVALID_ARG;
    }
    
    if (gpio_num >= GPIO_FLASH_PIN_START) {
        BSW_LOGE(TAG, "GPIO %d is reserved for Flash (26-30 not accessible on ESP32-C6)", gpio_num);
        return ESP_ERR_NOT_SUPPORTED;
    }
    
    return ESP_OK;
}

/**
 * @brief GPIO 드라이버 초기화 (FreeRTOS Mutex 생성)
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_init(void) {
    // Mutex 생성 (처음 한번만)
    if (gpio_mutex == NULL) {
        gpio_mutex = xSemaphoreCreateRecursiveMutex();
        if (gpio_mutex == NULL) {
            BSW_LOGE(TAG, "Failed to create GPIO mutex");
            return ESP_ERR_NO_MEM;
        }
    }
    
    // GPIO 매트릭스 및 IO_MUX 기본 설정
    // ESP32-C6의 GPIO 클럭 설정
    // 추가적인 영역 초기화가 필요하면 여기서 추가
    
    gpio_initialized = true;
    BSW_LOGI(TAG, "GPIO driver initialized with FreeRTOS mutex protection");
    return ESP_OK;
}

/**
 * @brief 개별 GPIO 핀 설정 (TRM 기반 직접 레지스터 제어)
 * 
 * @param gpio_num GPIO 핀번호
 * @param mode GPIO 모드
 * @param pull_up 풀업 활성화 여부
 * @param pull_down 풀다운 활성화 여부
 * @return ESP_OK 성공, ESP_FAIL 실패
 * 
 * @note ESP32-C6 TRM에 명시된 확장 GPIO 레지스터 주소와 비트 마스크를 이용합니다
 */
esp_err_t bsw_gpio_config_pin(bsw_gpio_num_t gpio_num, bsw_gpio_mode_t mode, 
                              bsw_gpio_pull_mode_t pull_up, bsw_gpio_pull_mode_t pull_down) {
    // Flash 메모리 보호를 위해 GPIO 번호 검사
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Mutex protection for configuration
    if (gpio_mutex == NULL) {
        BSW_LOGE(TAG, "GPIO not initialized, call bsw_gpio_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTakeRecursive(gpio_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        BSW_LOGE(TAG, "Failed to acquire GPIO mutex");
        return ESP_ERR_TIMEOUT;
    }

    // 방향 설정 (W1TS/W1TC + 오픈 드레인 모드 지정)
    ret = bsw_gpio_set_direction(gpio_num, mode);
    if (ret != ESP_OK) {
        xSemaphoreGiveRecursive(gpio_mutex);
        return ret;
    }
    
    // 풀업/풀다운 설정 (GPIO_PIN_N_REG + 비트 마스크)
    ret = bsw_gpio_set_pull_mode(gpio_num, pull_up, pull_down);
    if (ret != ESP_OK) {
        xSemaphoreGiveRecursive(gpio_mutex);
        return ret;
    }
    
    xSemaphoreGiveRecursive(gpio_mutex);
    
    BSW_LOGI(TAG, "GPIO %d configured: mode=%d, pull_up=%d, pull_down=%d", 
             gpio_num, mode, pull_up, pull_down);
    
    return ESP_OK;
}

/**
 * @brief GPIO 설정 (비트마스크 사용방식)
 * 
 * @param pGPIOConfig GPIO 설정 구조체 포인터
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_config(const bsw_gpio_config_t* pGPIOConfig) {
    if (!pGPIOConfig) {
        BSW_LOGE(TAG, "GPIO config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 비트마스크의 각 비트에 대한 설정 적용
    for (int gpio_num = 0; gpio_num < BSW_GPIO_PIN_COUNT; gpio_num++) {
        if (pGPIOConfig->pin_bit_mask & (1ULL << gpio_num)) {
            esp_err_t ret = bsw_gpio_config_pin(gpio_num, pGPIOConfig->mode, 
                                                pGPIOConfig->pull_up_en, pGPIOConfig->pull_down_en);
            if (ret != ESP_OK) {
                BSW_LOGE(TAG, "Failed to configure GPIO %d", gpio_num);
                return ret;
            }
        }
    }

    return ESP_OK;
}

/**
 * @brief GPIO 레벨 읽기 (직접 레지스터 읽기)
 * 
 * @param gpio_num GPIO 핀번호
 * @return GPIO 레벨 (0 또는 1)
 * 
 * @note GPIO_IN_REG 레지스터에서 직접 읽어 GPIO 입력 레벨을 확인합니다
 */
int bsw_gpio_get_level(bsw_gpio_num_t gpio_num) {
    // Flash 메모리 보호를 위해 GPIO 번호 검사
    if (validate_gpio_num(gpio_num) != ESP_OK) {
        return 0;
    }
    
    // GPIO 입력 레지스터에서 직접 읽기 (확장 GPIO_IN_REG_OFFSET 이용)
    return (REG_READ(BSW_GPIO_IN_REG) >> gpio_num) & 0x1;
}

/**
 * @brief GPIO 레벨 설정 (W1TS/W1TC 레지스터 - Atomic & Thread-safe)
 * 
 * @param gpio_num GPIO 핀번호
 * @param level 설정할 레벨 (0 또는 1)
 * @return ESP_OK 성공, ESP_FAIL 실패
 * 
 * @note W1TS (Write 1 to Set) 레지스터와 W1TC (Write 1 to Clear) 레지스터를 이용해
 *       한번의 쓰기 작업으로 설정 상태를 안전하게 변경합니다 (Atomic operation)
 * @note Mutex 불필요: W1TS/W1TC 레지스터만을 이용하므로 atomic하며 mutex 없이 thread-safe합니다
 */
esp_err_t bsw_gpio_set_level(bsw_gpio_num_t gpio_num, uint32_t level) {
    // Flash 메모리 보호를 위해 GPIO 번호 검사
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // W1TS/W1TC 레지스터 직접 이용해 GPIO 레벨 설정
    if (level) {
        // W1TS (Write 1 to Set) 레지스터 이용 -> Atomic & Thread-safe
        REG_WRITE(BSW_GPIO_OUT_W1TS_REG, (1ULL << gpio_num));
    } else {
        // W1TC (Write 1 to Clear) 레지스터 이용 -> Atomic & Thread-safe
        REG_WRITE(BSW_GPIO_OUT_W1TC_REG, (1ULL << gpio_num));
    }
    
    return ESP_OK;
}

/**
 * @brief GPIO 방향 설정 (W1TS/W1TC + 오픈 드레인 모드 지정)
 * 
 * @param gpio_num GPIO 핀번호
 * @param mode GPIO 모드
 * @return ESP_OK 성공, ESP_FAIL 실패
 * 
 * @note W1TS/W1TC 레지스터를 이용해 출력 방향을 설정하고,
 *       PAD_DRIVER 비트를 조여 오픈 드레인 모드를 지정합니다.
 */
esp_err_t bsw_gpio_set_direction(bsw_gpio_num_t gpio_num, bsw_gpio_mode_t mode) {
    // Flash 메모리 보호를 위해 GPIO 번호 검사
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Mutex protection for Read-Modify-Write operations
    if (gpio_mutex == NULL) {
        BSW_LOGE(TAG, "GPIO not initialized, call bsw_gpio_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTakeRecursive(gpio_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        BSW_LOGE(TAG, "Failed to acquire GPIO mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    switch (mode) {
        case BSW_GPIO_MODE_INPUT:
            // 입력 모드: 출력 비활성화
            REG_WRITE(BSW_GPIO_ENABLE_W1TC_REG, (1ULL << gpio_num));
            break;
            
        case BSW_GPIO_MODE_OUTPUT:
            // 출력 모드 (Push-Pull): 출력 활성화, 오픈 드레인 비활성화
            REG_WRITE(BSW_GPIO_ENABLE_W1TS_REG, (1ULL << gpio_num));
            reg_val &= ~GPIO_PIN_PAD_DRIVER_BIT; // 오픈 드레인 비활성화
            REG_WRITE(pin_reg_addr, reg_val);
            break;
            
        case BSW_GPIO_MODE_OUTPUT_OD:
            // 오픈 드레인 출력: 출력 활성화 PAD_DRIVER 비트 설정
            REG_WRITE(BSW_GPIO_ENABLE_W1TS_REG, (1ULL << gpio_num));
            reg_val |= GPIO_PIN_PAD_DRIVER_BIT; // 오픈 드레인 활성화
            REG_WRITE(pin_reg_addr, reg_val);
            break;
            
        case BSW_GPIO_MODE_INPUT_OUTPUT:
        case BSW_GPIO_MODE_INPUT_OUTPUT_OD:
            // 입출력 모드: 출력 활성화
            REG_WRITE(BSW_GPIO_ENABLE_W1TS_REG, (1ULL << gpio_num));
            if (mode == BSW_GPIO_MODE_INPUT_OUTPUT_OD) {
                reg_val |= GPIO_PIN_PAD_DRIVER_BIT; // 오픈 드레인 활성화
            } else {
                reg_val &= ~GPIO_PIN_PAD_DRIVER_BIT; // 오픈 드레인 비활성화
            }
            REG_WRITE(pin_reg_addr, reg_val);
            break;
            
        case BSW_GPIO_MODE_DISABLE:
        default:
            // GPIO 비활성화: 출력 비활성화
            REG_WRITE(BSW_GPIO_ENABLE_W1TC_REG, (1ULL << gpio_num));
            break;
    }
    
    xSemaphoreGiveRecursive(gpio_mutex);
    return ESP_OK;
}

/**
 * @brief GPIO 풀업/풀다운 설정 (GPIO_PIN_N_REG + 비트 마스크 이용)
 * 
 * @param gpio_num GPIO 핀번호
 * @param pull_up 풀업 활성화 여부
 * @param pull_down 풀다운 활성화 여부
 * @return ESP_OK 성공, ESP_FAIL 실패
 * 
 * @note 이 함수는 GPIO_PIN_PULLUP_BIT, GPIO_PIN_PULLDOWN_BIT 매크로를 이용해
 *       명확하고 안전하게 풀업/풀다운 설정을 합니다.
 */
esp_err_t bsw_gpio_set_pull_mode(bsw_gpio_num_t gpio_num, bsw_gpio_pull_mode_t pull_up, bsw_gpio_pull_mode_t pull_down) {
    // Flash 메모리 보호를 위해 GPIO 번호 검사
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Mutex protection for Read-Modify-Write operations
    if (gpio_mutex == NULL) {
        BSW_LOGE(TAG, "GPIO not initialized, call bsw_gpio_init() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (xSemaphoreTakeRecursive(gpio_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
        BSW_LOGE(TAG, "Failed to acquire GPIO mutex");
        return ESP_ERR_TIMEOUT;
    }
    
    // GPIO_PIN_N_REG 레지스터를 이용한 풀업/풀다운 설정
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // 기존 풀업/풀다운 설정 비트 클리어 (비트 마스크 이용)
    reg_val &= ~(GPIO_PIN_PULLUP_BIT | GPIO_PIN_PULLDOWN_BIT);
    
    // 새로운 설정 적합 비트를 사용
    if (pull_up == BSW_GPIO_PULLUP_ENABLE) {
        reg_val |= GPIO_PIN_PULLUP_BIT;
    }
    if (pull_down == BSW_GPIO_PULLDOWN_ENABLE) {
        reg_val |= GPIO_PIN_PULLDOWN_BIT;
    }
    
    REG_WRITE(pin_reg_addr, reg_val);
    
    xSemaphoreGiveRecursive(gpio_mutex);
    return ESP_OK;
}

/**
 * @brief 레지스터에 직접 쓰기 (비트 연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param value 값
 */
void bsw_gpio_raw_write_reg(uint32_t reg_addr, uint32_t value) {
    *(volatile uint32_t*)reg_addr = value;
}

/**
 * @brief 레지스터에서 직접 읽기 (비트 연산)
 * 
 * @param reg_addr 레지스터 주소
 * @return 값
 */
uint32_t bsw_gpio_raw_read_reg(uint32_t reg_addr) {
    return *(volatile uint32_t*)reg_addr;
}

/**
 * @brief 레지스터 특정 비트 설정 (비트 연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param bit_mask 설정할 비트 마스크
 */
void bsw_gpio_raw_set_bits(uint32_t reg_addr, uint32_t bit_mask) {
    BSW_SET_REG_BITS(reg_addr, bit_mask);
}

/**
 * @brief 레지스터 특정 비트 클리어 (비트 연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param bit_mask 클리어할 비트 마스크
 */
void bsw_gpio_raw_clear_bits(uint32_t reg_addr, uint32_t bit_mask) {
    BSW_CLEAR_REG_BITS(reg_addr, bit_mask);
}

/**
 * @brief 레지스터 특정 비트 토글 (비트 연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param bit_mask 토글할 비트 마스크
 */
void bsw_gpio_raw_toggle_bits(uint32_t reg_addr, uint32_t bit_mask) {
    uint32_t current_val = *(volatile uint32_t*)reg_addr;
    *(volatile uint32_t*)reg_addr = current_val ^ bit_mask;
}

/**
 * @brief GPIO 기능 설정 (ESP32-C6 TRM 기반 - GPIO_PINn_REG 이용)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param func_sel 기능 선택 (0-7) - 용도에 따라 다름 (IO_MUX는 별도 레지스터)
 * @param pullup 풀업 활성화
 * @param pulldown 풀다운 활성화
 * 
 * @note ESP32-C6 TRM Chapter 6.4.6: GPIO_PINn_REG 이용
 *       IO_MUX 기능 선택 0x60092000 영역은 별도 레지스터임
 *       따라서 GPIO_PINn_REG를 이용함
 */
void bsw_gpio_configure_iomux(bsw_gpio_num_t gpio_num, uint32_t func_sel, bool pullup, bool pulldown) {
    if (validate_gpio_num(gpio_num) != ESP_OK) return;
    
    // Mutex protection for read-modify-write operation
    if (gpio_mutex != NULL) {
        xSemaphoreTakeRecursive(gpio_mutex, portMAX_DELAY);
    }
    
    // ESP32-C6: GPIO_PINn_REG 이용 (통합 레지스터)
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    
    // 현재 읽기
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // 풀업 설정 (비트 7, FUN_WPU)
    if (pullup) {
        reg_val |= GPIO_PIN_PULLUP_BIT;
    } else {
        reg_val &= ~GPIO_PIN_PULLUP_BIT;
    }
    
    // 풀다운 설정 (비트 8, FUN_WPD)
    if (pulldown) {
        reg_val |= GPIO_PIN_PULLDOWN_BIT;
    } else {
        reg_val &= ~GPIO_PIN_PULLDOWN_BIT;
    }
    
    // 레지스터 쓰기
    REG_WRITE(pin_reg_addr, reg_val);
    
    if (gpio_mutex != NULL) {
        xSemaphoreGiveRecursive(gpio_mutex);
    }
}

/**
 * @brief GPIO 드라이브 강도 설정 (ESP32-C6 TRM 기반)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param strength 드라이브 강도 (0-3: 5mA, 10mA, 20mA, 40mA)
 * 
 * @note ESP32-C6 TRM Chapter 6.4.6: GPIO_PINn_REG??비트 1:0 (FUN_DRV)
 *       - 0: ~5mA
 *       - 1: ~10mA
 *       - 2: ~20mA
 *       - 3: ~40mA
 */
void bsw_gpio_set_drive_strength(bsw_gpio_num_t gpio_num, uint8_t strength) {
    if (validate_gpio_num(gpio_num) != ESP_OK || strength > 3) return;
    
    // Mutex protection for read-modify-write operation
    if (gpio_mutex != NULL) {
        xSemaphoreTakeRecursive(gpio_mutex, portMAX_DELAY);
    }
    
    // ESP32-C6: GPIO_PINn_REG 이용 (통합 레지스터)
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    
    // 현재 읽기
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // 드라이브 강도 설정 (비트 1:0, FUN_DRV)
    reg_val = (reg_val & ~GPIO_PIN_DRIVE_STRENGTH_MASK) | 
              ((strength & 0x3) << GPIO_PIN_DRIVE_STRENGTH_SHIFT);
    
    // 레지스터 쓰기
    REG_WRITE(pin_reg_addr, reg_val);
    
    if (gpio_mutex != NULL) {
        xSemaphoreGiveRecursive(gpio_mutex);
    }
}

/**
 * @brief GPIO 슬루 레이트 설정 (ESP32-C6 TRM 기반)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param fast_slew true=빠른 슬루 레이트, false=느린 슬루 레이트
 * 
 * @note ESP32-C6 TRM Chapter 6.4.6: GPIO_PINn_REG 비트 9 (FUN_SLP_SEL)
 */
void bsw_gpio_set_slew_rate(bsw_gpio_num_t gpio_num, bool fast_slew) {
    if (validate_gpio_num(gpio_num) != ESP_OK) return;
    
    // Mutex protection for read-modify-write operation
    if (gpio_mutex != NULL) {
        xSemaphoreTakeRecursive(gpio_mutex, portMAX_DELAY);
    }
    
    // ESP32-C6: GPIO_PINn_REG 이용 (통합 레지스터)
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    
    // 현재 읽기
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // 슬루 레이트 설정 (비트 9, FUN_SLP_SEL)
    if (fast_slew) {
        reg_val |= GPIO_PIN_SLEW_RATE_BIT;
    } else {
        reg_val &= ~GPIO_PIN_SLEW_RATE_BIT;
    }
    
    // 레지스터 쓰기
    REG_WRITE(pin_reg_addr, reg_val);
    
    if (gpio_mutex != NULL) {
        xSemaphoreGiveRecursive(gpio_mutex);
    }
}

// BSW GPIO 인터럽트 처리 - 하드웨어 인터럽트 구현

/**
 * @brief GPIO ISR 핸들러 배열
 */
static bsw_gpio_isr_t gpio_isr_handlers[BSW_GPIO_PIN_COUNT];
static void* gpio_isr_args[BSW_GPIO_PIN_COUNT];
static bool isr_service_installed = false;
static intr_handle_t gpio_isr_handle = NULL;

/**
 * @brief 하드웨어 GPIO ISR (IRAM_ATTR로 IRAM에 배치)
 * 
 * GPIO 인터럽트 발생 시 ESP32-C6 하드웨어에서 직접 호출되는 함수입니다.
 * 
 * @param arg 사용자 데이터 (현재 미사용)
 * 
 * @note IRAM_ATTR: 함수가 IRAM에 배치되어 캐시 미스 없이 빠르게 실행됩니다.
 *       인터럽트 핸들러는 가능한 짧고 빠르게 실행되어야 합니다.
 */
static void IRAM_ATTR gpio_isr_handler(void* arg) {
    // GPIO 인터럽트 상태 읽기 (어떤 GPIO가 인터럽트 발생시켰는지 확인)
    uint32_t gpio_intr_status = REG_READ(BSW_GPIO_STATUS_REG);
    
    // 인터럽트가 발생한 핀들에 대해 처리
    for (int gpio_num = 0; gpio_num < GPIO_USABLE_PIN_COUNT; gpio_num++) {
        if (gpio_intr_status & (1UL << gpio_num)) {
            // 인터럽트 상태 클리어(W1TC: Write 1 to Clear)
            REG_WRITE(BSW_GPIO_STATUS_W1TC_REG, (1UL << gpio_num));
            
            // 등록된 핸들러 호출
            if (gpio_isr_handlers[gpio_num] != NULL) {
                gpio_isr_handlers[gpio_num](gpio_isr_args[gpio_num]);
            }
        }
    }
}

/**
 * @brief BSW GPIO 인터럽트 서비스 설치
 * 
 * @param intr_alloc_flags 인터럽트 할당 플래그(ESP_INTR_FLAG_LEVEL1, ESP_INTR_FLAG_IRAM 등)
 * @return esp_err_t ESP_OK 성공, ESP_ERR_INVALID_STATE 이미 설치된 경우 오류
 * 
 * @note ESP32-C6 하드웨어 GPIO 인터럽트 사용입니다.
 *       - CPU 인터럽트 컨트롤러에 ISR 등록
 *       - 폴링이 아닌 하드웨어 기반 인터럽트로 99.9% 이상 호감도
 *       - 코드가 같아도 빠른 호출 최적화
 */
esp_err_t bsw_gpio_install_isr_service(int intr_alloc_flags) {
    if (isr_service_installed) {
        BSW_LOGW(TAG, "GPIO ISR service already installed");
        return ESP_ERR_INVALID_STATE;
    }
    
    // ISR 핸들러 배열 초기화
    for (int i = 0; i < BSW_GPIO_PIN_COUNT; i++) {
        gpio_isr_handlers[i] = NULL;
        gpio_isr_args[i] = NULL;
    }
    
    // CPU 인터럽트 컨트롤러에 GPIO ISR 등록
    // ETS_GPIO_INTR_SOURCE는 ESP32-C6의 GPIO 인터럽트 소스 번호
    esp_err_t ret = esp_intr_alloc(ETS_GPIO_INTR_SOURCE, 
                                   intr_alloc_flags | ESP_INTR_FLAG_IRAM,
                                   gpio_isr_handler, 
                                   NULL, 
                                   &gpio_isr_handle);
    
    if (ret != ESP_OK) {
        BSW_LOGE(TAG, "Failed to allocate GPIO interrupt: %s", esp_err_to_name(ret));
        return ret;
    }
    
    isr_service_installed = true;
    BSW_LOGI(TAG, "GPIO hardware ISR service installed (flags: 0x%x)", intr_alloc_flags);
    return ESP_OK;
}

/**
 * @brief GPIO 인터럽트 핸들러 등록
 * 
 * @param gpio_num GPIO 핀 번호
 * @param isr_handler 인터럽트 핸들러 함수
 * @param args 핸들러에 전달할 사용자 데이터
 * @return esp_err_t ESP_OK 성공, ESP_ERR_INVALID_STATE ISR 서비스 미설치 경우 오류
 * 
 * @note 인터럽트 핸들러 등록입니다.
 *       등록 후 bsw_gpio_set_intr_type()와 bsw_gpio_intr_enable()
 *       호출하여 인터럽트 활성화해야 합니다.
 */
esp_err_t bsw_gpio_isr_handler_add(bsw_gpio_num_t gpio_num, bsw_gpio_isr_t isr_handler, void* args) {
    if (!isr_service_installed) {
        BSW_LOGE(TAG, "GPIO ISR service not installed, call bsw_gpio_install_isr_service() first");
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    if (gpio_isr_handlers[gpio_num] != NULL) {
        BSW_LOGW(TAG, "GPIO %d handler already exists, replacing", gpio_num);
    }
    
    // 인터럽트 핸들러 등록
    gpio_isr_handlers[gpio_num] = isr_handler;
    gpio_isr_args[gpio_num] = args;
    
    // GPIO 입력 모드 설정
    bsw_gpio_set_direction(gpio_num, BSW_GPIO_MODE_INPUT);
    
    BSW_LOGI(TAG, "GPIO %d hardware interrupt handler registered", gpio_num);
    
    return ESP_OK;
}

/**
 * @brief GPIO ISR 핸들러 제거
 * 
 * @param gpio_num GPIO 핀 번호
 * @return esp_err_t ESP_OK 성공, ESP_ERR_INVALID_STATE ISR 서비스 미설치 경우 오류
 * 
 * @note 핸들러 제거 시에 bsw_gpio_intr_disable()를 호출하여 인터럽트 비활성화하는 것을 권장합니다.
 */
esp_err_t bsw_gpio_isr_handler_remove(bsw_gpio_num_t gpio_num) {
    if (!isr_service_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 인터럽트 비활성화
    bsw_gpio_intr_disable(gpio_num);
    
    // 핸들러 제거
    gpio_isr_handlers[gpio_num] = NULL;
    gpio_isr_args[gpio_num] = NULL;
    
    BSW_LOGI(TAG, "GPIO %d ISR handler removed", gpio_num);
    return ESP_OK;
}

/**
 * @brief BSW GPIO ISR 서비스 제거
 * 
 * @note 모든 GPIO 인터럽트 비활성화하고 CPU 인터럽트도 해제합니다.
 */
void bsw_gpio_uninstall_isr_service(void) {
    if (!isr_service_installed) {
        return;
    }
    
    // 모든 GPIO 인터럽트 비활성화
    for (int i = 0; i < GPIO_USABLE_PIN_COUNT; i++) {
        if (gpio_isr_handlers[i] != NULL) {
            bsw_gpio_intr_disable(i);
        }
        gpio_isr_handlers[i] = NULL;
        gpio_isr_args[i] = NULL;
    }
    
    // CPU 인터럽트 핸들러 해제
    if (gpio_isr_handle != NULL) {
        esp_intr_free(gpio_isr_handle);
        gpio_isr_handle = NULL;
    }
    
    isr_service_installed = false;
    BSW_LOGI(TAG, "GPIO ISR service uninstalled");
}

/**
 * @brief GPIO 인터럽트 타입 설정 (레지스터 직접 접근)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param intr_type 인터럽트 타입(상승 에지, 하강 에지, 양쪽 에지, 로우 레벨, 하이 레벨)
 * @return esp_err_t ESP_OK 성공, 기타 오류
 * 
 * @note ESP32-C6 TRM의 GPIO_PINn_REG INT_TYPE 필드 (비트 9:7)를 설정합니다.
 *       - 0: 비활성화
 *       - 1: 상승 에지
 *       - 2: 하강 에지
 *       - 3: 양쪽 에지
 *       - 4: 로우 레벨
 *       - 5: 하이 레벨
 */
esp_err_t bsw_gpio_set_intr_type(bsw_gpio_num_t gpio_num, bsw_gpio_int_type_t intr_type) {
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Mutex protection for read-modify-write operation
    if (gpio_mutex != NULL) {
        if (xSemaphoreTakeRecursive(gpio_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            BSW_LOGE(TAG, "Failed to acquire GPIO mutex");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // INT_TYPE 필드 클리어(비트 9:7)
    reg_val &= ~GPIO_PIN_INT_TYPE_MASK;
    
    // 새로운 인터럽트 타입 설정
    reg_val |= ((intr_type & 0x7) << GPIO_PIN_INT_TYPE_SHIFT);
    
    REG_WRITE(pin_reg_addr, reg_val);
    
    if (gpio_mutex != NULL) {
        xSemaphoreGiveRecursive(gpio_mutex);
    }
    
    BSW_LOGI(TAG, "GPIO %d interrupt type set to %d", gpio_num, intr_type);
    return ESP_OK;
}

/**
 * @brief GPIO 인터럽트 활성화 (레지스터 직접 접근)
 * 
 * @param gpio_num GPIO 핀 번호
 * @return esp_err_t ESP_OK 성공, 기타 오류
 * 
 * @note ESP32-C6 TRM의 GPIO_PINn_REG INT_ENA 필드 (비트 17:13)를 설정합니다.
 *       CPU 인터럽트도 활성화하기 위해 비트 13 (CPU interrupt enable)을 설정합니다.
 */
esp_err_t bsw_gpio_intr_enable(bsw_gpio_num_t gpio_num) {
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Mutex protection for read-modify-write operation
    if (gpio_mutex != NULL) {
        if (xSemaphoreTakeRecursive(gpio_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            BSW_LOGE(TAG, "Failed to acquire GPIO mutex");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // INT_ENA 필드 설정 (비트 13: CPU interrupt enable)
    reg_val |= (1U << GPIO_PIN_INT_ENA_SHIFT);
    
    REG_WRITE(pin_reg_addr, reg_val);
    
    if (gpio_mutex != NULL) {
        xSemaphoreGiveRecursive(gpio_mutex);
    }
    
    BSW_LOGI(TAG, "GPIO %d interrupt enabled", gpio_num);
    return ESP_OK;
}

/**
 * @brief GPIO 인터럽트 비활성화 (레지스터 직접 접근)
 * 
 * @param gpio_num GPIO 핀 번호
 * @return esp_err_t ESP_OK 성공, 기타 오류
 * 
 * @note ESP32-C6 TRM의 GPIO_PINn_REG INT_ENA 필드 (비트 17:13)를 설정합니다.
 */
esp_err_t bsw_gpio_intr_disable(bsw_gpio_num_t gpio_num) {
    esp_err_t ret = validate_gpio_num(gpio_num);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // Mutex protection for read-modify-write operation
    if (gpio_mutex != NULL) {
        if (xSemaphoreTakeRecursive(gpio_mutex, pdMS_TO_TICKS(100)) != pdTRUE) {
            BSW_LOGE(TAG, "Failed to acquire GPIO mutex");
            return ESP_ERR_TIMEOUT;
        }
    }
    
    uint32_t pin_reg_addr = GPIO_PIN_N_REG(gpio_num);
    uint32_t reg_val = REG_READ(pin_reg_addr);
    
    // INT_ENA 필드 클리어(비트 17:13)
    reg_val &= ~GPIO_PIN_INT_ENA_MASK;
    
    REG_WRITE(pin_reg_addr, reg_val);
    
    // 인터럽트 상태 클리어(특히 아마도 놓치는 인터럽트 클리어)
    REG_WRITE(BSW_GPIO_STATUS_W1TC_REG, (1UL << gpio_num));
    
    if (gpio_mutex != NULL) {
        xSemaphoreGiveRecursive(gpio_mutex);
    }
    
    BSW_LOGI(TAG, "GPIO %d interrupt disabled", gpio_num);
    return ESP_OK;
}

/**
 * @brief GPIO 폴링 기반 인터럽트 핸들러 함수 (임시 대체용)
 * 
 * @param gpio_num 감시할 GPIO 핀
 * 
 * @deprecated 폴링 기반 인터럽트 사용은 권장하지 않습니다.
 *             폴링 방식은 CPU 사용률이 높고 응답 지연 가능성이 있습니다.
 * @note 가능한 환경을 위해 정식 ISR 사용을 권장합니다.
 */
void bsw_gpio_poll_isr(bsw_gpio_num_t gpio_num) {
    static uint32_t prev_state[BSW_GPIO_PIN_COUNT] = {0};
    
    if (!isr_service_installed || validate_gpio_num(gpio_num) != ESP_OK) {
        return;
    }
    
    if (gpio_isr_handlers[gpio_num] == NULL) {
        return;
    }
    
    // 현재 GPIO 상태 읽기
    uint32_t current_state = GPIO_BIT_READ(gpio_num);
    
    // 상태 변화 감지 (변경 처리 필요)
    if (current_state != prev_state[gpio_num]) {
        prev_state[gpio_num] = current_state;
        
        // ISR 호출
        if (gpio_isr_handlers[gpio_num]) {
            gpio_isr_handlers[gpio_num](gpio_isr_args[gpio_num]);
        }
    }
}

/**
 * @brief GPIO 드라이버 종료 (리소스 해제)
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 * 
 * @note ISR 서비스 제거 후 mutex도 삭제합니다.
 */
esp_err_t bsw_gpio_deinit(void) {
    // ISR 서비스 제거
    if (isr_service_installed) {
        bsw_gpio_uninstall_isr_service();
    }
    
    // Mutex 삭제
    if (gpio_mutex != NULL) {
        vSemaphoreDelete(gpio_mutex);
        gpio_mutex = NULL;
    }
    
    gpio_initialized = false;
    BSW_LOGI(TAG, "GPIO driver deinitialized");
    return ESP_OK;
}
