# BalanceBot: 전문적인 아키텍처 기반의 ESP32 균형 로봇

이 프로젝트는 **ESP32-S3**를 기반으로 한 2륜 균형 로봇(Self-Balancing Robot)입니다. 단순한 기능 구현을 넘어, **AUTOSAR(오토사)와 유사한 계층형 소프트웨어 아키텍처**를 적용하여 코드의 **이식성, 재사용성, 테스트 용이성**을 극대화한 것이 특징입니다.

FreeRTOS를 활용한 실시간 멀티태스킹, 칼만 필터를 이용한 정밀한 센서 융합, 그리고 중앙 상태 머신을 통한 안정적인 동작 제어 등 상용 제품 수준의 임베디드 소프트웨어 설계를 경험할 수 있습니다.

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

### ESP32 펌웨어
* **전문적인 소프트웨어 아키텍처:** 하드웨어 제어를 담당하는 **BSW(기반 소프트웨어) 계층**과 실제 로직을 담당하는 **애플리케이션 계층(Input, Logic, Output)**을 완벽하게 분리하여 이식성과 유지보수성을 극대화했습니다.
* **실시간 제어 시스템:** **FreeRTOS**를 사용하여 센서 데이터 처리, 균형 제어, 상태 모니터링 등 여러 작업을 독립적인 Task로 분리하여 안정적인 실시간 성능을 보장합니다.
* **안정적인 상태 관리:** `IDLE`, `BALANCING`, `FALLEN` 등 로봇의 모든 동작을 관리하는 **중앙 상태 머신(State Machine)**을 구현하여 예외 상황에서도 예측 가능하고 안정적으로 동작합니다.
* **정밀한 센서 융합:** IMU(MPU6050) 센서의 가속도계와 자이로스코프 데이터를 **칼만 필터(Kalman Filter)** 알고리즘으로 융합하여 노이즈를 최소화하고 정확한 기울기 각도를 측정합니다.

### Flutter 모바일 앱
* **현대적인 BLE 통신:** **flutter_blue_plus** 라이브러리를 사용한 안정적이고 현대적인 블루투스 연결
* **실시간 PID 튜닝:** 모바일 앱에서 실시간으로 PID 파라미터를 조정하고 즉시 적용
* **직관적인 UI/UX:** Material Design 3 기반의 현대적이고 직관적인 사용자 인터페이스
* **상태 모니터링:** 로봇의 각도, 속도, 배터리 상태를 실시간으로 시각화

### 개발 및 테스트
* **단위 테스트 환경:** PlatformIO의 **네이티브(Native) 테스트 환경**을 구축하여, 실제 하드웨어 없이 PC에서도 PID 제어기, 칼만 필터 등 핵심 로직을 검증할 수 있습니다.
* **중앙 집중식 설정:** 모든 하드웨어 핀, PID 상수, 시스템 파라미터를 **`config.h` 파일에서 중앙 관리**하여 튜닝 및 수정이 매우 용이합니다.
* **완전한 문서화:** **Doxygen**을 통한 ESP32 C 코드와 Flutter Dart 코드의 통합 문서화

## 🏗️ 소프트웨어 아키텍처

이 프로젝트는 다음과 같은 4개의 계층으로 명확하게 분리되어 있습니다.

1.  **BSW (Basic Software):** 하드웨어 제어 계층

      * MCU의 주변장치(I2C, UART, PWM 등)를 직접 제어하는 드라이버 코드입니다.
      * 하드웨어에 대한 모든 종속성을 이 계층에 격리시켜 이식성을 극대화합니다.

2.  **Input (입력 계층):** 센서 데이터 가공

      * BSW를 통해 센서의 원본(Raw) 데이터를 읽어와 `Logic` 계층이 사용할 수 있는 물리량(각도, 속도 등)으로 변환합니다.

3.  **Logic (로직 계층):** 로봇의 두뇌

      * `Input` 계층의 데이터를 바탕으로 PID 제어, 칼만 필터 등 핵심 알고리즘을 수행하여 로봇이 "무엇을 할지" 결정합니다.
      * 이 계층은 하드웨어와 완전히 독립적입니다.

4.  **Output (출력 계층):** 행동 제어

      * `Logic` 계층의 결정을 받아 BSW를 통해 실제 모터, 서보 등 액추에이터를 구동합니다.

## 🛠️ 개발 환경 및 빌드

이 프로젝트는 **PlatformIO**를 사용하여 개발되었습니다.

### 1\. ESP32 펌웨어 빌드 및 업로드

```bash
# 프로젝트 빌드
pio run -e esp32-s3-devkitc-1

# 펌웨어 업로드
pio run -e esp32-s3-devkitc-1 -t upload

# 시리얼 모니터 실행
pio device monitor -e esp32-s3-devkitc-1
```

### 2\. PC 네이티브 단위 테스트 실행

핵심 로직(PID, 칼만 필터 등)은 하드웨어 없이 PC에서 직접 테스트할 수 있습니다.

```bash
# 네이티브 환경에서 단위 테스트 실행
pio test -e native -v
```

## ⚙️ 설정 및 튜닝

로봇의 모든 주요 설정은 `src/config.h` 파일에서 수정할 수 있습니다.

  * **하드웨어 핀 설정:** 모터, 엔코더, 센서의 GPIO 핀 번호
  * **제어 파라미터:** PID 제어 상수 (`Kp`, `Ki`, `Kd`)
  * **로봇 물리 제원:** 바퀴 지름, 엔코더 PPR 등

<!-- end list -->

```c
// src/config.h 예시

// PID Controller tuning
#define CONFIG_BALANCE_PID_KP           50.0f
#define CONFIG_BALANCE_PID_KI           0.5f
#define CONFIG_BALANCE_PID_KD           2.0f

// Left motor pins
#define CONFIG_LEFT_MOTOR_A_PIN         GPIO_NUM_4
#define CONFIG_LEFT_MOTOR_B_PIN         GPIO_NUM_5
```

## 📜 라이선스

이 프로젝트는 [MIT 라이선스](LICENSE)를 따릅니다.

## 🆕 최신 업데이트 (2025.09.20)

### Flutter 앱 현대화
- ✅ **블루투스 라이브러리 업그레이드**: `flutter_bluetooth_serial` → `flutter_blue_plus`
- ✅ **Java 호환성 개선**: Java 8 deprecation 경고 완전 해결
- ✅ **의존성 정리**: 불필요한 third_party 라이브러리 제거
- ✅ **빌드 최적화**: 더 빠르고 안정적인 APK 빌드

### 문서화 시스템
- ✅ **통합 문서화**: Doxygen을 통한 ESP32 + Flutter 통합 문서
- ✅ **한국어 지원**: 완전한 한국어 문서화 및 인터페이스
- ✅ **검색 기능**: 코드, 클래스, 함수 통합 검색
- ✅ **최적화**: 문서 표시 및 네비게이션 개선

### 프로젝트 구조 개선
- ✅ **코드 정리**: 중복 파일 및 불필요한 백업 파일 제거
- ✅ **.gitignore 업데이트**: 더 나은 버전 관리
- ✅ **라이브러리 표준화**: 모든 문서 주석에 library 지시문 추가
