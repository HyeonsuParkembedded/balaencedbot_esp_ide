# BalanceBot Testing Guide

## Native Testing Environment

이 프로젝트는 ESP32 코드의 핵심 모듈들을 네이티브 환경에서 테스트할 수 있도록 설정되었습니다.

### 설정된 구성요소

1. **Native Test Environment**
   - Unity 테스트 프레임워크 사용
   - ESP32/FreeRTOS 의존성 Mock 처리
   - Kalman Filter 및 PID Controller 모듈 테스트

2. **테스트 커버리지**
   - **Kalman Filter**: 초기화, 각도 업데이트, 노이즈 제거
   - **PID Controller**: 초기화, P/I/D 동작, 출력 제한, 리셋

### 테스트 실행 방법

#### 네이티브 테스트 실행
```bash
# PlatformIO CLI 사용
C:\Users\hyuns\.platformio\penv\Scripts\platformio.exe test --environment native

# 또는 배치 파일 사용
./test_runner.bat
```

#### ESP32 빌드
```bash
# ESP32 펌웨어 빌드
C:\Users\hyuns\.platformio\penv\Scripts\platformio.exe run --environment esp32-s3-devkitc-1

# 또는 배치 파일 사용
./build_esp32.bat
```

### 테스트 결과 예시
```
Running BalanceBot Native Tests
================================

Running Kalman Filter Tests...
test\test_modules.c:144: test_kalman_filter_init      [PASSED]
test\test_modules.c:145: test_kalman_filter_update    [PASSED]

Running PID Controller Tests...
test\test_modules.c:148: test_pid_controller_init                [PASSED]
test\test_modules.c:149: test_pid_controller_proportional        [PASSED]
test\test_modules.c:150: test_pid_controller_output_limits       [PASSED]
test\test_modules.c:151: test_pid_controller_reset               [PASSED]

✓ All tests passed!

================== 6 test cases: 6 succeeded in 00:00:01.608 ==================
```

### 파일 구조
```
BalanceBot/
├── src/                    # ESP32 소스 코드
│   ├── kalman_filter.c/h   # 칼만 필터 구현
│   ├── pid_controller.c/h  # PID 컨트롤러 구현
│   └── main.c              # 메인 애플리케이션
├── test/                   # 네이티브 테스트
│   ├── test_modules.c      # 통합 테스트 파일
│   └── freertos/           # FreeRTOS Mock 헤더
├── platformio.ini          # 빌드 설정 (native/ESP32)
├── test_runner.bat         # 네이티브 테스트 실행 스크립트
└── build_esp32.bat         # ESP32 빌드 스크립트
```

### Mock 처리된 의존성
- **FreeRTOS**: `xTaskGetTickCount()`, 태스크 관련 함수들
- **ESP32 로깅**: `ESP_LOGI`, `ESP_LOGW`, `ESP_LOGE`
- **시간 함수**: 네이티브 환경용 시간 Mock

### 주의사항
1. 네이티브 테스트는 핵심 알고리즘 로직만 검증
2. 실제 하드웨어 인터페이스는 ESP32에서만 테스트 가능
3. 센서 데이터, 모터 제어 등은 통합 테스트 시 확인 필요