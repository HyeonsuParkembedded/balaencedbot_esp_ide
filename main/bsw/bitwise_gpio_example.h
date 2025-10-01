/**
 * @file bitwise_gpio_example.h
 * @brief BSW 순수 비트연산 GPIO 제어 데모 헤더 파일
 * 
 * ESP32-C6 GPIO 레지스터를 직접 조작하여 최고 성능을 달성하는
 * 비트연산 기반 GPIO 제어 데모 함수 선언을 제공합니다.
 * 
 * @author Hyeonsu Park, Suyong Kim
 * @date 2025-10-01
 * @version 1.0 (비트연산 순수 구현)
 */

#ifndef BSW_BITWISE_GPIO_EXAMPLE_H
#define BSW_BITWISE_GPIO_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief BSW 순수 비트연산 GPIO 제어 데모 실행
 * 
 * ESP32-C6 GPIO 레지스터를 직접 제어하여 다양한 비트연산 기법을
 * 시연합니다. ESP-IDF 함수를 전혀 사용하지 않고 레지스터 직접 접근으로
 * 최고 성능을 달성하는 방법을 보여줍니다.
 * 
 * 데모 내용:
 * 1. 레지스터 직접 주소 접근
 * 2. 매크로 기반 고속 제어  
 * 3. 멀티 GPIO 동시 제어
 * 4. 입력 GPIO 비트연산 읽기
 * 5. IO_MUX 직접 비트연산 제어
 * 6. 원시 레지스터 비트 조작
 * 7. 인라인 함수 최적화 제어
 */
void demo_bitwise_gpio_control(void);

/**
 * @brief 레지스터 제어 방식 비교 분석
 * 
 * 다양한 GPIO 레지스터 제어 방식의 성능과 차이점을 분석하고
 * 결과를 출력합니다.
 * 
 * 비교 대상:
 * - ESP-IDF 구조체 방식
 * - 순수 레지스터 주소 방식
 * - 비트 마스크 조작 방식
 */
void analyze_register_differences(void);

#ifdef __cplusplus
}
#endif

#endif // BSW_BITWISE_GPIO_EXAMPLE_H