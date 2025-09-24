# BalanceBot: 전문적인 아키텍처 기반의 ESP32-C6 균형 로봇

이 프로젝트는 **ESP32-C6**를 기반으로 한 2륜 균형 로봇(Self-Balancing Robot)입니다. 단순한 기능 구현을 넘어, **AUTOSAR(오토사)와 유사한 계층형 소프트웨어 아키텍처**를 적용하여 코드의 **이식성, 재사용성, 테스트 용이성**을 극대화한 것이 특징입니다.

FreeRTOS를 활용한 실시간 멀티태스킹, 칼만 필터를 이용한 정밀한 센서 융합, 그리고 중앙 상태 머신을 통한 안정적인 동작 제어 등 상용 제품 수준의 임베디드 소프트웨어 설계를 경험할 수 있습니다.

## 🏗️ BSW(Basic Software) 아키텍처

이 프로젝트의 가장 큰 특징은 **완전한 BSW 아키텍처 구현**입니다:

### BSW 계층 구조
- **GPIO Driver**: 디지털 I/O 및 인터럽트 관리
- **PWM Driver**: 모터 제어용 5kHz PWM 및 서보용 50Hz PWM 지원
- **I2C Driver**: IMU 센서 통신용 I2C 인터페이스
- **UART Driver**: GPS 및 디버깅 통신
- **BLE Driver**: 블루투스 통신 추상화
- **System Services**: 시스템 시간, 로깅, 딜레이 등 공통 서비스

### BSW 설계 원칙
- **하드웨어 독립성**: 상위 레이어는 ESP-IDF API를 직접 호출하지 않음
- **일관된 인터페이스**: 모든 BSW 드라이버는 표준화된 함수명과 반환값 사용
- **오류 처리**: 체계적인 오류 코드 및 로깅 시스템
- **메모리 안전성**: 동적 메모리 할당 최소화 및 안전한 버퍼 관리

## 📱 Flutter 모바일 앱

이 프로젝트는 ESP32 펌웨어와 함께 **Flutter 기반 모바일 컨트롤러 앱**을 포함합니다:

### 주요 기능
- **BLE 무선 통신**: `flutter_blue_plus`를 사용한 안정적인 블루투스 연결
- **실시간 제어**: 조이스틱 기반 로봇 조종 
- **PID 실시간 튜닝**: 모바일에서 PID 파라미터 조정
- **상태 모니터링**: 로봇 각도, 속도, 배터리 상태 실시간 확인
- **서보 기립**: 넘어진 로봇 자동 기립 기능

### 빠른 시작
```bash
cd balance_bot_controller

# 의존성 설치
flutter pub get

# 디버그 실행
flutter run

# APK 빌드
flutter build apk --release
```

자세한 Flutter 앱 사용법은 [`balance_bot_controller/README.md`](balance_bot_controller/README.md)를 참조하세요.

## 📚 프로젝트 문서화

이 프로젝트는 **Doxygen**을 사용하여 완전히 문서화되어 있습니다:

### 문서 생성 방법
```powershell
# 자동 문서 생성 (PowerShell)
.\scripts\generate_docs.ps1

# 또는 직접 Doxygen 실행
doxygen Doxyfile
```

### 문서 구성
- **HTML 문서**: `docs/html/index.html`에서 확인 가능
- **검색 기능**: 코드, 클래스, 함수 검색 지원
- **트리뷰**: 파일 및 클래스 계층 구조 탐색
- **한국어 지원**: 한국어 문서화 및 인터페이스
- **통합 문서**: ESP32 C 코드와 Flutter Dart 코드 통합 문서

### 문서화 범위
- **ESP32 펌웨어**: 모든 C 파일, 함수, 구조체 상세 문서화
- **Flutter 앱**: Dart 클래스, 메서드, 위젯 완전 문서화
- **아키텍처 설계**: BSW, Input, Logic, Output 계층 설명
- **API 레퍼런스**: BLE 통신 프로토콜 및 PID 파라미터 문서
- **사용법 가이드**: 설치, 빌드, 실행 단계별 안내

## ✨ 주요 특징 (Key Features)

### ESP32-C6 펌웨어 (ESP-IDF v5.5)
* **완전한 BSW 아키텍처:** ESP-IDF 하드웨어 제어를 담당하는 **BSW(기반 소프트웨어) 계층**과 실제 로직을 담당하는 **애플리케이션 계층(Input, Logic, Output, System)**을 완벽하게 분리하여 이식성과 유지보수성을 극대화했습니다.
* **실시간 제어 시스템:** **FreeRTOS**를 사용하여 센서 데이터 처리, 균형 제어, 상태 모니터링 등 여러 작업을 독립적인 Task로 분리하여 안정적인 실시간 성능을 보장합니다.
* **안정적인 상태 관리:** `IDLE`, `BALANCING`, `FALLEN` 등 로봇의 모든 동작을 관리하는 **중앙 상태 머신(State Machine)**을 구현하여 예외 상황에서도 예측 가능하고 안정적으로 동작합니다.
* **정밀한 센서 융합:** IMU(MPU6050) 센서의 가속도계와 자이로스코프 데이터를 **칼만 필터(Kalman Filter)** 알고리즘으로 융합하여 노이즈를 최소화하고 정확한 기울기 각도를 측정합니다.
* **서보 기립 시스템:** 넘어진 로봇을 자동으로 일으켜 세우는 전용 서보 모터 제어 시스템 (50Hz PWM)
* **이중 PWM 시스템:** 모터 제어용 5kHz PWM과 서보 제어용 50Hz PWM을 동시 지원

### Flutter 모바일 앱
* **현대적인 BLE 통신:** **flutter_blue_plus** 라이브러리를 사용한 안정적이고 현대적인 블루투스 연결
* **실시간 PID 튜닝:** 모바일 앱에서 실시간으로 PID 파라미터를 조정하고 즉시 적용
* **직관적인 UI/UX:** Material Design 3 기반의 현대적이고 직관적인 사용자 인터페이스
* **상태 모니터링:** 로봇의 각도, 속도, 배터리 상태를 실시간으로 시각화

### 개발 및 테스트
* **ESP-IDF v5.5:** 최신 ESP-IDF 프레임워크 기반 개발
* **CMake 빌드 시스템:** 현대적이고 유연한 빌드 환경
* **중앙 집중식 설정:** 모든 하드웨어 핀, PID 상수, 시스템 파라미터를 **`config.h` 파일에서 중앙 관리**하여 튜닝 및 수정이 매우 용이합니다.
* **완전한 문서화:** **Doxygen**을 통한 ESP32-C6 C 코드와 Flutter Dart 코드의 통합 문서화
* **NATIVE_BUILD 제거:** 조건부 컴파일 완전 제거로 코드 단순성 향상

## 🏗️ 소프트웨어 아키텍처

이 프로젝트는 다음과 같은 5개의 계층으로 명확하게 분리되어 있습니다.

1.  **BSW (Basic Software):** 하드웨어 제어 계층

      * ESP-IDF의 주변장치(GPIO, I2C, UART, PWM, BLE 등)를 직접 제어하는 드라이버 코드입니다.
      * 하드웨어에 대한 모든 종속성을 이 계층에 격리시켜 이식성을 극대화합니다.
      * 표준화된 함수 인터페이스와 오류 처리 시스템을 제공합니다.

2.  **Input (입력 계층):** 센서 데이터 가공

      * BSW를 통해 센서의 원본(Raw) 데이터를 읽어와 `Logic` 계층이 사용할 수 있는 물리량(각도, 속도 등)으로 변환합니다.
      * IMU, GPS, 엔코더 센서 데이터를 처리합니다.

3.  **Logic (로직 계층):** 로봇의 두뇌

      * `Input` 계층의 데이터를 바탕으로 PID 제어, 칼만 필터 등 핵심 알고리즘을 수행하여 로봇이 "무엇을 할지" 결정합니다.
      * 이 계층은 하드웨어와 완전히 독립적입니다.

4.  **Output (출력 계층):** 행동 제어

      * `Logic` 계층의 결정을 받아 BSW를 통해 실제 모터, 서보, BLE 등 액추에이터를 구동합니다.
      * 모터 제어, 서보 기립, BLE 통신을 담당합니다.

5.  **System (시스템 계층):** 시스템 서비스

      * 오류 복구, 시스템 상태 관리 등 전체 시스템 운영을 담당합니다.
      * BSW 시스템 서비스를 활용한 안정적인 시스템 관리를 제공합니다.

## 🛠️ 개발 환경 및 빌드

이 프로젝트는 **ESP-IDF v5.5**를 사용하여 개발되었습니다.

### 1\. ESP32-C6 펌웨어 빌드 및 업로드

```bash
# ESP-IDF 환경 설정
export.bat  # Windows
source export.sh  # Linux/Mac

# 프로젝트 빌드
idf.py build

# 펌웨어 업로드
idf.py flash

# 시리얼 모니터 실행
idf.py monitor
```

### 2\. VS Code 개발 환경

ESP-IDF VS Code Extension을 사용하여 통합 개발 환경에서 작업할 수 있습니다:

```bash
# VS Code에서 빌드
Ctrl+Shift+P -> ESP-IDF: Build Project

# VS Code에서 플래시
Ctrl+Shift+P -> ESP-IDF: Flash Project

# VS Code에서 모니터
Ctrl+Shift+P -> ESP-IDF: Monitor Device
```

## ⚙️ 설정 및 튜닝

로봇의 모든 주요 설정은 `main/config.h` 파일에서 수정할 수 있습니다.

  * **하드웨어 핀 설정:** 모터, 엔코더, 센서의 GPIO 핀 번호
  * **제어 파라미터:** PID 제어 상수 (`Kp`, `Ki`, `Kd`)
  * **로봇 물리 제원:** 바퀴 지름, 엔코더 PPR 등
  * **BSW 설정:** PWM 주파수, I2C 클럭 등

<!-- end list -->

```c
// main/config.h 예시

// PID Controller tuning
#define CONFIG_BALANCE_PID_KP           50.0f
#define CONFIG_BALANCE_PID_KI           0.5f
#define CONFIG_BALANCE_PID_KD           2.0f

// GPIO Pin Configuration
#define CONFIG_LEFT_MOTOR_PIN_A         4
#define CONFIG_LEFT_MOTOR_PIN_B         5
#define CONFIG_SERVO_STANDUP_PIN        18

// PWM Configuration
#define CONFIG_MOTOR_PWM_FREQ           5000    // 5kHz for motors
#define CONFIG_SERVO_PWM_FREQ           50      // 50Hz for servo
```

## 📜 라이선스

이 프로젝트는 [MIT 라이선스](LICENSE)를 따릅니다.

## 🆕 최신 업데이트 (2025.09.24)

### BSW 아키텍처 완전 구현
- ✅ **완전한 BSW 계층**: GPIO, PWM, I2C, UART, BLE, System Services 드라이버 구현
- ✅ **ESP32-C6 지원**: 최신 ESP32-C6 MCU 및 ESP-IDF v5.5 기반
- ✅ **이중 PWM 시스템**: 모터용 5kHz와 서보용 50Hz PWM 동시 지원
- ✅ **서보 기립 시스템**: 넘어진 로봇 자동 기립 기능 완전 구현

### 코드 품질 개선
- ✅ **NATIVE_BUILD 제거**: 모든 조건부 컴파일 코드 제거로 단순성 향상
- ✅ **한국어 주석 수정**: UTF-8 인코딩 문제 완전 해결
- ✅ **BSW 표준화**: 일관된 함수명, 오류 처리, 로깅 시스템
- ✅ **메모리 최적화**: 984,959 bytes 이미지 크기, DIRAM 23.14% 사용

### 개발 환경 현대화
- ✅ **ESP-IDF v5.5**: 최신 ESP-IDF 프레임워크 적용
- ✅ **CMake 빌드**: 현대적인 빌드 시스템 완전 이전
- ✅ **VS Code 통합**: ESP-IDF Extension 기반 통합 개발환경
- ✅ **Git 관리**: main 브랜치 기반 체계적인 버전 관리
