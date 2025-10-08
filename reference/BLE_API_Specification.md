# BalanceBot BLE API 사양서

**버전:** 1.0  
**날짜:** 2025-10-08  
**대상:** 모바일 앱 개발자  

---

## 📋 목차

1. [개요](#개요)
2. [BLE 서비스 정보](#ble-서비스-정보)
3. [연결 및 초기화](#연결-및-초기화)
4. [이진 프로토콜 (앱 제어용)](#이진-프로토콜-앱-제어용)
5. [텍스트 프로토콜 (개발자 튜닝용)](#텍스트-프로토콜-개발자-튜닝용)
6. [상태 데이터 수신](#상태-데이터-수신)
7. [에러 코드](#에러-코드)
8. [샘플 코드](#샘플-코드)

---

## 개요

BalanceBot ESP32-C6는 BLE(Bluetooth Low Energy)를 통해 두 가지 통신 프로토콜을 지원합니다:

- **이진 프로토콜**: 모바일 앱에서 로봇 제어용 (실시간 제어)
- **텍스트 프로토콜**: 개발자 도구용 (파라미터 튜닝)

로봇은 자동으로 수신된 데이터가 이진인지 텍스트인지 구분하여 처리합니다.

---

## BLE 서비스 정보

### 서비스 UUID
```
UUID: 0000FF00-0000-1000-8000-00805F9B34FB
Short UUID: 0x00FF
```

### 특성(Characteristics)

#### 1. 명령 특성 (Command Characteristic)
- **UUID**: `0000FF01-0000-1000-8000-00805F9B34FB`
- **Short UUID**: `0xFF01`
- **속성**: Write, Write No Response
- **용도**: 앱에서 로봇으로 제어 명령 전송

#### 2. 상태 특성 (Status Characteristic)  
- **UUID**: `0000FF02-0000-1000-8000-00805F9B34FB`
- **Short UUID**: `0xFF02`
- **속성**: Notify, Read
- **용도**: 로봇에서 앱으로 상태 정보 전송

### 디바이스 정보
- **디바이스명**: "BalanceBot-C6"
- **연결 간격**: 7.5ms - 20ms (권장: 15ms)
- **MTU**: 최대 512 바이트

---

## 연결 및 초기화

### 1. 디바이스 스캔
```kotlin
// Android 예제
val scanFilter = ScanFilter.Builder()
    .setServiceUuid(ParcelUuid.fromString("0000FF00-0000-1000-8000-00805F9B34FB"))
    .build()

bluetoothLeScanner.startScan(listOf(scanFilter), scanSettings, scanCallback)
```

### 2. 연결 설정
```kotlin
// GATT 연결
val bluetoothGatt = device.connectGatt(context, false, gattCallback)

// 특성 찾기
val service = bluetoothGatt.getService(UUID.fromString("0000FF00-0000-1000-8000-00805F9B34FB"))
val commandChar = service.getCharacteristic(UUID.fromString("0000FF01-0000-1000-8000-00805F9B34FB"))
val statusChar = service.getCharacteristic(UUID.fromString("0000FF02-0000-1000-8000-00805F9B34FB"))

// 상태 알림 활성화
bluetoothGatt.setCharacteristicNotification(statusChar, true)
```

---

## 이진 프로토콜 (앱 제어용)

모바일 앱에서 로봇을 제어하기 위한 실시간 프로토콜입니다.

### 명령 패킷 구조

| 바이트 | 필드 | 타입 | 범위 | 설명 |
|--------|------|------|------|------|
| 0 | Header | uint8 | 0x42 | 패킷 시작 표시 ('B') |
| 1 | Direction | int8 | -1, 0, 1 | 이동 방향 |
| 2 | Turn | int8 | -100 ~ 100 | 회전 (-100:좌회전, 100:우회전) |
| 3 | Speed | uint8 | 0 ~ 100 | 속도 (0:정지, 100:최대속도) |
| 4 | Flags | uint8 | bit field | 제어 플래그 |
| 5 | Checksum | uint8 | - | 체크섬 (XOR) |

### Direction 필드
- `-1`: 후진
- `0`: 정지
- `1`: 전진

### Turn 필드
- `-100 ~ -1`: 좌회전 (값이 작을수록 급회전)
- `0`: 직진
- `1 ~ 100`: 우회전 (값이 클수록 급회전)

### Flags 필드 (비트 플래그)
```
Bit 0: Balance 활성화 (1: ON, 0: OFF)
Bit 1: Standup 명령 (1: 기립시도, 0: 일반동작)
Bit 2-7: 예약됨 (0으로 설정)
```

### 체크섬 계산
```kotlin
fun calculateChecksum(data: ByteArray): Byte {
    var checksum: Byte = 0
    for (i in 0 until data.size - 1) {
        checksum = (checksum.toInt() xor data[i].toInt()).toByte()
    }
    return checksum
}
```

### 명령 예제

#### 전진 + 밸런싱 ON
```
[0x42, 0x01, 0x00, 0x50, 0x01, 0x12]
 Header Direction Turn Speed Flags Checksum
```

#### 좌회전
```
[0x42, 0x00, 0xD0, 0x30, 0x01, 0xE3]
 Header Direction Turn(-48) Speed Flags Checksum
```

#### 기립 명령
```
[0x42, 0x00, 0x00, 0x00, 0x03, 0x41]
 Header Direction Turn Speed Flags(Balance+Standup) Checksum
```

---

## 텍스트 프로토콜 (개발자 튜닝용)

개발자가 PID 파라미터 등을 실시간으로 조정하기 위한 ASCII 텍스트 프로토콜입니다.

### 명령 형식

모든 텍스트 명령은 ASCII 문자(0x20-0x7E)로만 구성되며 null-terminated 문자열입니다.

#### SET 명령 - 파라미터 설정
```
SET <param_id> <value>
```

#### GET 명령 - 파라미터 조회
```
GET <param_id>
```

#### SAVE 명령 - NVS에 저장
```
SAVE
```

#### RESET 명령 - 기본값 복원
```
RESET
```

### 파라미터 ID 목록

| ID | 파라미터명 | 설명 | 기본값 | 범위 |
|----|-----------|------|-------|------|
| 0 | balance_kp | 밸런스 PID 비례 게인 | 25.0 | 0.0 ~ 100.0 |
| 1 | balance_ki | 밸런스 PID 적분 게인 | 0.5 | 0.0 ~ 10.0 |
| 2 | balance_kd | 밸런스 PID 미분 게인 | 0.8 | 0.0 ~ 10.0 |
| 3 | velocity_kp | 속도 PID 비례 게인 | 1.2 | 0.0 ~ 10.0 |
| 4 | velocity_ki | 속도 PID 적분 게인 | 0.02 | 0.0 ~ 1.0 |
| 5 | velocity_kd | 속도 PID 미분 게인 | 0.0 | 0.0 ~ 1.0 |
| 6 | kalman_q_angle | 칼만 각도 프로세스 노이즈 | 0.001 | 0.0001 ~ 0.1 |
| 7 | kalman_q_bias | 칼만 바이어스 프로세스 노이즈 | 0.003 | 0.0001 ~ 0.1 |
| 8 | kalman_r_measure | 칼만 측정 노이즈 | 0.03 | 0.001 ~ 1.0 |
| 9 | max_tilt_angle | 최대 기울기 각도 (도) | 45.0 | 10.0 ~ 90.0 |
| 10 | fallen_threshold | 넘어짐 판정 임계값 (도) | 60.0 | 30.0 ~ 90.0 |

### 텍스트 명령 예제

#### PID 파라미터 조정
```
SET 0 30.0       // Balance Kp를 30.0으로 설정
SET 1 0.8        // Balance Ki를 0.8로 설정  
SET 2 1.2        // Balance Kd를 1.2로 설정
```

#### 파라미터 조회
```
GET 0            // Balance Kp 값 조회
GET 3            // Velocity Kp 값 조회
```

#### 설정 관리
```
SAVE             // 현재 설정을 NVS에 저장
RESET            // 모든 설정을 기본값으로 복원
```

---

## 상태 데이터 수신

로봇은 주기적으로(약 20Hz) 상태 정보를 BLE Status Characteristic을 통해 전송합니다.

### 상태 패킷 구조

| 바이트 | 필드 | 타입 | 단위 | 설명 |
|--------|------|------|------|------|
| 0-3 | Angle | float | 도(degree) | 로봇 기울기 각도 |
| 4-7 | Velocity | float | m/s | 로봇 속도 |
| 8-11 | Battery | float | V | 배터리 전압 |

### 데이터 파싱 예제

```kotlin
fun parseStatusData(data: ByteArray): RobotStatus {
    val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
    
    return RobotStatus(
        angle = buffer.float,
        velocity = buffer.float,
        batteryVoltage = buffer.float
    )
}

data class RobotStatus(
    val angle: Float,      // 기울기 각도 (-90 ~ +90도)
    val velocity: Float,   // 속도 (-2.0 ~ +2.0 m/s)
    val batteryVoltage: Float  // 배터리 전압 (3.0 ~ 4.2V)
)
```

---

## 에러 코드

### BLE 연결 에러
- **연결 실패**: 로봇이 꺼져있거나 범위 밖
- **서비스 없음**: 잘못된 디바이스에 연결
- **특성 없음**: 펌웨어 버전 불일치

### 명령 에러
- **체크섬 불일치**: 이진 패킷 손상
- **잘못된 헤더**: 0x42가 아닌 헤더
- **범위 초과**: 파라미터 값이 유효 범위 벗어남

### 텍스트 명령 에러
- **구문 오류**: 명령 형식이 잘못됨
- **알 수 없는 명령**: SET, GET, SAVE, RESET가 아님
- **잘못된 파라미터 ID**: 0-10 범위 벗어남

---

## 샘플 코드

### Android (Kotlin)

```kotlin
class BalanceBotController {
    private var bluetoothGatt: BluetoothGatt? = null
    private var commandCharacteristic: BluetoothGattCharacteristic? = null
    private var statusCharacteristic: BluetoothGattCharacteristic? = null
    
    // 로봇 제어 (이진 프로토콜)
    fun sendControlCommand(direction: Int, turn: Int, speed: Int, balance: Boolean, standup: Boolean) {
        val flags = (if (balance) 0x01 else 0x00) or (if (standup) 0x02 else 0x00)
        val packet = byteArrayOf(
            0x42.toByte(),              // Header
            direction.toByte(),         // Direction
            turn.toByte(),             // Turn  
            speed.toByte(),            // Speed
            flags.toByte(),            // Flags
            0x00                       // Checksum (계산 필요)
        )
        
        // 체크섬 계산
        packet[5] = calculateChecksum(packet)
        
        commandCharacteristic?.let {
            it.value = packet
            bluetoothGatt?.writeCharacteristic(it)
        }
    }
    
    // 파라미터 튜닝 (텍스트 프로토콜)  
    fun setParameter(paramId: Int, value: Float) {
        val command = "SET $paramId $value"
        commandCharacteristic?.let {
            it.value = command.toByteArray()
            bluetoothGatt?.writeCharacteristic(it)
        }
    }
    
    // 상태 데이터 파싱
    fun parseStatusNotification(data: ByteArray): RobotStatus {
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        return RobotStatus(
            angle = buffer.float,
            velocity = buffer.float, 
            batteryVoltage = buffer.float
        )
    }
    
    private fun calculateChecksum(data: ByteArray): Byte {
        var checksum: Byte = 0
        for (i in 0 until data.size - 1) {
            checksum = (checksum.toInt() xor data[i].toInt()).toByte()
        }
        return checksum
    }
}
```

### Flutter (Dart)

```dart
import 'dart:typed_data';

class BalanceBotController {
  static const String serviceUUID = "0000FF00-0000-1000-8000-00805F9B34FB";
  static const String commandUUID = "0000FF01-0000-1000-8000-00805F9B34FB";
  static const String statusUUID = "0000FF02-0000-1000-8000-00805F9B34FB";
  
  BluetoothCharacteristic? commandCharacteristic;
  BluetoothCharacteristic? statusCharacteristic;
  
  // 로봇 제어
  Future<void> sendControlCommand({
    required int direction,
    required int turn, 
    required int speed,
    required bool balance,
    required bool standup
  }) async {
    final flags = (balance ? 0x01 : 0x00) | (standup ? 0x02 : 0x00);
    final packet = Uint8List.fromList([
      0x42,              // Header
      direction & 0xFF,  // Direction
      turn & 0xFF,       // Turn
      speed & 0xFF,      // Speed  
      flags & 0xFF,      // Flags
      0x00               // Checksum
    ]);
    
    // 체크섬 계산
    packet[5] = _calculateChecksum(packet);
    
    await commandCharacteristic?.write(packet, withoutResponse: true);
  }
  
  // 파라미터 설정
  Future<void> setParameter(int paramId, double value) async {
    final command = 'SET $paramId $value';
    await commandCharacteristic?.write(
      command.codeUnits, 
      withoutResponse: true
    );
  }
  
  // 상태 데이터 파싱  
  RobotStatus parseStatusData(List<int> data) {
    final bytes = Uint8List.fromList(data);
    final buffer = ByteData.sublistView(bytes);
    
    return RobotStatus(
      angle: buffer.getFloat32(0, Endian.little),
      velocity: buffer.getFloat32(4, Endian.little), 
      batteryVoltage: buffer.getFloat32(8, Endian.little),
    );
  }
  
  int _calculateChecksum(Uint8List data) {
    int checksum = 0;
    for (int i = 0; i < data.length - 1; i++) {
      checksum ^= data[i];
    }
    return checksum & 0xFF;
  }
}

class RobotStatus {
  final double angle;
  final double velocity;
  final double batteryVoltage;
  
  RobotStatus({
    required this.angle,
    required this.velocity, 
    required this.batteryVoltage,
  });
}
```

---

## 주의사항

1. **연결 안정성**: BLE 연결이 불안정할 수 있으므로 재연결 로직 구현 필요
2. **명령 주기**: 제어 명령은 최대 20Hz로 전송 (50ms 간격)
3. **파라미터 범위**: 텍스트 프로토콜로 파라미터 설정 시 유효 범위 확인 필수
4. **배터리 모니터링**: 배터리 전압이 3.2V 이하로 떨어지면 연결 해제 권장
5. **에러 처리**: 체크섬 오류나 응답 없음에 대한 적절한 에러 처리 구현

---

**문의사항이나 버그 리포트는 GitHub Issue로 등록해주세요.**