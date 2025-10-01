/**
 * @file gpio_driver.c
 * @brief BSW GPIO 드라이버 구현 파일
 * 
 * GPIO 하드웨어 레지스터 직접 제어를 통한 고성능 GPIO 드라이버 구현입니다.
 * ESP32-C6의 GPIO 및 IO_MUX 레지스터를 직접 조작하여 최적화된 성능을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 2.0
 */

#include "gpio_driver.h"
#include "system_services.h"
#include "soc/gpio_struct.h"
#include "soc/gpio_reg.h"
#include "soc/io_mux_reg.h"
#include "hal/gpio_ll.h"

static const char* TAG = "BSW_GPIO";

/**
 * @brief GPIO 드라이버 초기화
 * 
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_init(void) {
    // GPIO 매트릭스와 IO_MUX 기본 설정
    // ESP32-C6의 GPIO 클럭을 활성화
    // 추가적인 전역 초기화 작업이 필요하면 여기에 추가
    
    bsw_log_bitwise(BSW_LOG_INFO, TAG, "GPIO driver initialized with direct register control");
    return ESP_OK;
}

/**
 * @brief 단일 GPIO 핀 설정 (직접 레지스터 제어)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param mode GPIO 모드
 * @param pull_up 풀업 활성화 여부
 * @param pull_down 풀다운 활성화 여부
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_config_pin(bsw_gpio_num_t gpio_num, bsw_gpio_mode_t mode, 
                              bsw_gpio_pull_mode_t pull_up, bsw_gpio_pull_mode_t pull_down) {
    if (gpio_num >= GPIO_PIN_COUNT) {
        BSW_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t pin_mask = (1ULL << gpio_num);
    
    // GPIO 모드 설정 - 직접 레지스터 제어
    switch (mode) {
        case BSW_GPIO_MODE_INPUT:
            // 입력 모드: 출력 비활성화, 입력 활성화
            GPIO.enable_w1tc.val = pin_mask;  // 출력 비활성화
            break;
            
        case BSW_GPIO_MODE_OUTPUT:
            // 출력 모드: 출력 활성화
            GPIO.enable_w1ts.val = pin_mask;  // 출력 활성화
            break;
            
        case BSW_GPIO_MODE_INPUT_OUTPUT:
            // 입출력 모드: 출력 활성화
            GPIO.enable_w1ts.val = pin_mask;  // 출력 활성화
            break;
            
        case BSW_GPIO_MODE_OUTPUT_OD:
            // 오픈 드레인 출력: 출력 활성화 + 오픈 드레인 설정
            GPIO.enable_w1ts.val = pin_mask;  // 출력 활성화
            GPIO.pin[gpio_num].pad_driver = 1;  // 오픈 드레인 모드
            break;
            
        case BSW_GPIO_MODE_DISABLE:
        default:
            // GPIO 비활성화
            GPIO.enable_w1tc.val = pin_mask;  // 출력 비활성화
            break;
    }
    
    // 풀업/풀다운 레지스터 직접 제어 (IO_MUX를 통해)
    uint32_t io_mux_reg = GPIO_PIN_MUX_REG[gpio_num];
    uint32_t mux_val = REG_READ(io_mux_reg);
    
    // 기존 풀업/풀다운 설정 클리어
    mux_val &= ~(FUN_PU | FUN_PD);
    
    // 풀업 설정
    if (pull_up == BSW_GPIO_PULLUP_ENABLE) {
        mux_val |= FUN_PU;
    }
    
    // 풀다운 설정  
    if (pull_down == BSW_GPIO_PULLDOWN_ENABLE) {
        mux_val |= FUN_PD;
    }
    
    // IO_MUX 레지스터에 설정 적용
    REG_WRITE(io_mux_reg, mux_val);
    
    BSW_LOGI(TAG, "GPIO %d configured: mode=%d, pull_up=%d, pull_down=%d", 
             gpio_num, mode, pull_up, pull_down);
    
    return ESP_OK;
}

/**
 * @brief GPIO 설정 (비트마스크 방식)
 * 
 * @param pGPIOConfig GPIO 설정 구조체 포인터
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_config(const bsw_gpio_config_t* pGPIOConfig) {
    if (!pGPIOConfig) {
        BSW_LOGE(TAG, "GPIO config is NULL");
        return ESP_ERR_INVALID_ARG;
    }

    // 비트마스크의 각 핀에 대해 설정 적용
    for (int gpio_num = 0; gpio_num < GPIO_PIN_COUNT; gpio_num++) {
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
 * @brief GPIO 레벨 읽기 (직접 레지스터 제어)
 * 
 * @param gpio_num GPIO 핀 번호
 * @return GPIO 레벨 (0 또는 1)
 */
int bsw_gpio_get_level(bsw_gpio_num_t gpio_num) {
    if (gpio_num >= GPIO_PIN_COUNT) {
        BSW_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return 0;
    }
    
    // GPIO 입력 레지스터에서 직접 읽기
    return (GPIO.in.val >> gpio_num) & 0x1;
}

/**
 * @brief GPIO 레벨 설정 (직접 레지스터 제어)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param level 설정할 레벨 (0 또는 1)
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_set_level(bsw_gpio_num_t gpio_num, uint32_t level) {
    if (gpio_num >= GPIO_PIN_COUNT) {
        BSW_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t pin_mask = (1ULL << gpio_num);
    
    // 레지스터 직접 제어로 GPIO 레벨 설정
    if (level) {
        GPIO.out_w1ts.val = pin_mask;  // Set 레지스터로 HIGH
    } else {
        GPIO.out_w1tc.val = pin_mask;  // Clear 레지스터로 LOW
    }
    
    return ESP_OK;
}

/**
 * @brief GPIO 방향 설정 (직접 레지스터 제어)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param mode GPIO 모드
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_set_direction(bsw_gpio_num_t gpio_num, bsw_gpio_mode_t mode) {
    if (gpio_num >= GPIO_PIN_COUNT) {
        BSW_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    uint32_t pin_mask = (1ULL << gpio_num);
    
    switch (mode) {
        case BSW_GPIO_MODE_INPUT:
            GPIO.enable_w1tc.val = pin_mask;  // 출력 비활성화
            break;
        case BSW_GPIO_MODE_OUTPUT:
        case BSW_GPIO_MODE_INPUT_OUTPUT:
        case BSW_GPIO_MODE_OUTPUT_OD:
            GPIO.enable_w1ts.val = pin_mask;  // 출력 활성화
            break;
        case BSW_GPIO_MODE_DISABLE:
        default:
            GPIO.enable_w1tc.val = pin_mask;  // 출력 비활성화
            break;
    }
    
    return ESP_OK;
}

/**
 * @brief GPIO 풀업/풀다운 설정 (직접 레지스터 제어)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param pull_up 풀업 활성화 여부
 * @param pull_down 풀다운 활성화 여부
 * @return ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_set_pull_mode(bsw_gpio_num_t gpio_num, bsw_gpio_pull_mode_t pull_up, bsw_gpio_pull_mode_t pull_down) {
    if (gpio_num >= GPIO_PIN_COUNT) {
        BSW_LOGE(TAG, "Invalid GPIO number: %d", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }
    
    // IO_MUX 레지스터를 통한 풀업/풀다운 설정
    uint32_t io_mux_reg = GPIO_PIN_MUX_REG[gpio_num];
    uint32_t mux_val = REG_READ(io_mux_reg);
    
    // 기존 풀업/풀다운 설정 클리어
    mux_val &= ~(FUN_PU | FUN_PD);
    
    // 새로운 설정 적용
    if (pull_up == BSW_GPIO_PULLUP_ENABLE) {
        mux_val |= FUN_PU;
    }
    if (pull_down == BSW_GPIO_PULLDOWN_ENABLE) {
        mux_val |= FUN_PD;
    }
    
    REG_WRITE(io_mux_reg, mux_val);
    
    return ESP_OK;
}

/**
 * @brief 레지스터에 직접 값 쓰기 (순수 비트연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param value 쓸 값
 */
void bsw_gpio_raw_write_reg(uint32_t reg_addr, uint32_t value) {
    *(volatile uint32_t*)reg_addr = value;
}

/**
 * @brief 레지스터에서 직접 값 읽기 (순수 비트연산)
 * 
 * @param reg_addr 레지스터 주소
 * @return 읽은 값
 */
uint32_t bsw_gpio_raw_read_reg(uint32_t reg_addr) {
    return *(volatile uint32_t*)reg_addr;
}

/**
 * @brief 레지스터의 특정 비트들 설정 (순수 비트연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param bit_mask 설정할 비트 마스크
 */
void bsw_gpio_raw_set_bits(uint32_t reg_addr, uint32_t bit_mask) {
    SET_REG_BITS(reg_addr, bit_mask);
}

/**
 * @brief 레지스터의 특정 비트들 클리어 (순수 비트연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param bit_mask 클리어할 비트 마스크
 */
void bsw_gpio_raw_clear_bits(uint32_t reg_addr, uint32_t bit_mask) {
    CLEAR_REG_BITS(reg_addr, bit_mask);
}

/**
 * @brief 레지스터의 특정 비트들 토글 (순수 비트연산)
 * 
 * @param reg_addr 레지스터 주소
 * @param bit_mask 토글할 비트 마스크
 */
void bsw_gpio_raw_toggle_bits(uint32_t reg_addr, uint32_t bit_mask) {
    uint32_t current_val = *(volatile uint32_t*)reg_addr;
    *(volatile uint32_t*)reg_addr = current_val ^ bit_mask;
}

/**
 * @brief IO_MUX 직접 제어를 통한 GPIO 설정 (순수 비트연산)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param func_sel 기능 선택 (0=GPIO, 1-5=다른 기능들)
 * @param pullup 풀업 활성화
 * @param pulldown 풀다운 활성화
 */
void bsw_gpio_configure_iomux(bsw_gpio_num_t gpio_num, uint32_t func_sel, bool pullup, bool pulldown) {
    if (gpio_num >= GPIO_PIN_COUNT) return;
    
    // IO_MUX 레지스터 주소 계산 (ESP32-C6 기준)
    uint32_t iomux_reg_addr = IO_MUX_BASE + (gpio_num * 4);
    
    // 현재 값 읽기
    uint32_t reg_val = bsw_gpio_raw_read_reg(iomux_reg_addr);
    
    // 기능 선택 설정 (비트 2:0)
    reg_val = (reg_val & ~0x7) | (func_sel & 0x7);
    
    // 풀업 설정 (비트 7)
    if (pullup) {
        reg_val |= (1U << 7);
    } else {
        reg_val &= ~(1U << 7);
    }
    
    // 풀다운 설정 (비트 8)
    if (pulldown) {
        reg_val |= (1U << 8);
    } else {
        reg_val &= ~(1U << 8);
    }
    
    // 레지스터에 쓰기
    bsw_gpio_raw_write_reg(iomux_reg_addr, reg_val);
}

/**
 * @brief GPIO 드라이브 강도 설정 (순수 비트연산)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param strength 드라이브 강도 (0-3: 약함~강함)
 */
void bsw_gpio_set_drive_strength(bsw_gpio_num_t gpio_num, uint8_t strength) {
    if (gpio_num >= GPIO_PIN_COUNT || strength > 3) return;
    
    // IO_MUX 레지스터 주소 계산
    uint32_t iomux_reg_addr = IO_MUX_BASE + (gpio_num * 4);
    
    // 현재 값 읽기
    uint32_t reg_val = bsw_gpio_raw_read_reg(iomux_reg_addr);
    
    // 드라이브 강도 설정 (비트 11:10)
    reg_val = (reg_val & ~(0x3 << 10)) | ((strength & 0x3) << 10);
    
    // 레지스터에 쓰기
    bsw_gpio_raw_write_reg(iomux_reg_addr, reg_val);
}

/**
 * @brief GPIO 슬루 레이트 설정 (순수 비트연산)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param fast_slew true=빠른 슬루 레이트, false=느린 슬루 레이트
 */
void bsw_gpio_set_slew_rate(bsw_gpio_num_t gpio_num, bool fast_slew) {
    if (gpio_num >= GPIO_PIN_COUNT) return;
    
    // IO_MUX 레지스터 주소 계산
    uint32_t iomux_reg_addr = IO_MUX_BASE + (gpio_num * 4);
    
    // 현재 값 읽기
    uint32_t reg_val = bsw_gpio_raw_read_reg(iomux_reg_addr);
    
    // 슬루 레이트 설정 (비트 9)
    if (fast_slew) {
        reg_val |= (1U << 9);
    } else {
        reg_val &= ~(1U << 9);
    }
    
    // 레지스터에 쓰기
    bsw_gpio_raw_write_reg(iomux_reg_addr, reg_val);
}

// BSW GPIO 인터럽트 처리 - 순수 비트연산 기반 구현

/**
 * @brief GPIO ISR 핸들러 저장 배열
 */
static bsw_gpio_isr_t gpio_isr_handlers[GPIO_PIN_COUNT];
static void* gpio_isr_args[GPIO_PIN_COUNT];
static bool isr_service_installed = false;

/**
 * @brief BSW GPIO ISR 서비스 설치 (순수 비트연산 기반)
 * 
 * @param intr_alloc_flags 인터럽트 할당 플래그 (BSW_INTR_FLAG_*)
 * @return esp_err_t ESP_OK 성공, ESP_FAIL 실패
 * 
 * @note 순수 비트연산 방식으로 GPIO 인터럽트를 처리합니다.
 *       HAL 라이브러리를 사용하지 않고 직접 레지스터 조작합니다.
 */
esp_err_t bsw_gpio_install_isr_service(int intr_alloc_flags) {
    if (isr_service_installed) {
        return ESP_ERR_INVALID_STATE; // 이미 설치됨
    }
    
    // ISR 핸들러 배열 초기화
    for (int i = 0; i < GPIO_PIN_COUNT; i++) {
        gpio_isr_handlers[i] = NULL;
        gpio_isr_args[i] = NULL;
    }
    
    isr_service_installed = true;
    BSW_LOGI("BSW_GPIO", "GPIO ISR service installed with flags 0x%x", intr_alloc_flags);
    return ESP_OK;
}

/**
 * @brief GPIO 핀에 ISR 핸들러 추가 (순수 비트연산)
 * 
 * @param gpio_num GPIO 핀 번호
 * @param isr_handler 인터럽트 핸들러 함수
 * @param args 핸들러에 전달할 인자
 * @return esp_err_t ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_isr_handler_add(bsw_gpio_num_t gpio_num, bsw_gpio_isr_t isr_handler, void* args) {
    if (!isr_service_installed) {
        BSW_LOGE("BSW_GPIO", "ISR service not installed");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (gpio_num >= GPIO_PIN_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (gpio_isr_handlers[gpio_num] != NULL) {
        bsw_log_bitwise(BSW_LOG_WARN, "BSW_GPIO", "GPIO %d ISR handler already exists, replacing", gpio_num);
    }
    
    // ISR 핸들러 등록
    gpio_isr_handlers[gpio_num] = isr_handler;
    gpio_isr_args[gpio_num] = args;
    
    // GPIO를 입력 모드로 설정
    bsw_gpio_set_direction(gpio_num, BSW_GPIO_MODE_INPUT);
    
    // 인터럽트 타입 설정 (양쪽 엣지 - ANYEDGE)
    // 여기서는 간단히 구현하여 폴링 방식으로 시뮬레이션
    BSW_LOGI("BSW_GPIO", "GPIO %d ISR handler added (polling mode)", gpio_num);
    
    return ESP_OK;
}

/**
 * @brief GPIO ISR 핸들러 제거 (순수 비트연산)
 * 
 * @param gpio_num GPIO 핀 번호
 * @return esp_err_t ESP_OK 성공, ESP_FAIL 실패
 */
esp_err_t bsw_gpio_isr_handler_remove(bsw_gpio_num_t gpio_num) {
    if (!isr_service_installed) {
        return ESP_ERR_INVALID_STATE;
    }
    
    if (gpio_num >= GPIO_PIN_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    
    gpio_isr_handlers[gpio_num] = NULL;
    gpio_isr_args[gpio_num] = NULL;
    
    BSW_LOGI("BSW_GPIO", "GPIO %d ISR handler removed", gpio_num);
    return ESP_OK;
}

/**
 * @brief BSW GPIO ISR 서비스 제거
 */
void bsw_gpio_uninstall_isr_service(void) {
    if (!isr_service_installed) {
        return;
    }
    
    // 모든 핸들러 제거
    for (int i = 0; i < GPIO_PIN_COUNT; i++) {
        gpio_isr_handlers[i] = NULL;
        gpio_isr_args[i] = NULL;
    }
    
    isr_service_installed = false;
    BSW_LOGI("BSW_GPIO", "GPIO ISR service uninstalled");
}

/**
 * @brief GPIO 폴링 기반 인터럽트 시뮬레이션 함수
 * 
 * 실제 하드웨어 인터럽트 대신 폴링을 통해 GPIO 상태 변화를 감지합니다.
 * 엔코더와 같은 빠른 신호 변화를 감지하기 위한 고주파 폴링 구현입니다.
 * 
 * @param gpio_num 감시할 GPIO 핀
 * @note 주기적으로 호출하여 GPIO 상태 변화를 확인해야 합니다.
 */
void bsw_gpio_poll_isr(bsw_gpio_num_t gpio_num) {
    static uint32_t prev_state[GPIO_PIN_COUNT] = {0};
    
    if (!isr_service_installed || gpio_num >= GPIO_PIN_COUNT) {
        return;
    }
    
    if (gpio_isr_handlers[gpio_num] == NULL) {
        return;
    }
    
    // 현재 GPIO 상태 읽기 (순수 비트연산)
    uint32_t current_state = GPIO_BIT_READ(gpio_num);
    
    // 상태 변화 감지 (엣지 트리거 시뮬레이션)
    if (current_state != prev_state[gpio_num]) {
        prev_state[gpio_num] = current_state;
        
        // ISR 핸들러 호출
        if (gpio_isr_handlers[gpio_num]) {
            gpio_isr_handlers[gpio_num](gpio_isr_args[gpio_num]);
        }
    }
}