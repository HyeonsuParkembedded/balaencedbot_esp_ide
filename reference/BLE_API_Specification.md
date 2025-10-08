# BalanceBot BLE API 사양서

**버전:** 2.0  
**날짜:** 2025-10-08  
**대상:** 모바일 앱 개발자  
**펌웨어 버전:** ESP-IDF v5.5 (NimBLE Stack)

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

**전체 메시지 구조: 헤더 (8바이트) + 페이로드 (8바이트) = 16바이트**

#### 프로토콜 헤더 (8바이트)

| 오프셋 | 필드 | 타입 | 값 | 설명 |
|--------|------|------|-----|------|
| 0 | start_marker | uint8 | 0xAA | 패킷 시작 마커 (고정값) |
| 1 | version | uint8 | 0x01 | 프로토콜 버전 |
| 2 | msg_type | uint8 | 0x01 | 메시지 타입 (MOVE_CMD) |
| 3 | seq_num | uint8 | 0-255 | 시퀀스 번호 (순차 증가) |
| 4-5 | payload_len | uint16 | 8 | 페이로드 길이 (Little Endian) |
| 6-7 | checksum | uint16 | - | CRC16 체크섬 (Little Endian) |

#### 이동 명령 페이로드 (8바이트)

| 오프셋 | 필드 | 타입 | 범위 | 설명 |
|--------|------|------|------|------|
| 8 | direction | int8 | -1, 0, 1 | 이동 방향 |
| 9 | turn | int8 | -100 ~ 100 | 회전 (-100:좌회전, 100:우회전) |
| 10 | speed | uint8 | 0 ~ 100 | 속도 (0:정지, 100:최대속도) |
| 11 | flags | uint8 | bit field | 제어 플래그 |
| 12-15 | timestamp | uint32 | - | 타임스탬프 (ms, Little Endian) |

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
Bit 0 (0x01): Balance 활성화 (1: ON, 0: OFF)
Bit 1 (0x02): Standup 명령 (1: 기립시도, 0: 일반동작)
Bit 2 (0x04): Emergency 비상정지
Bit 3-7: 예약됨 (0으로 설정)
```

### CRC16 체크섬 계산
```kotlin
fun calculateCRC16(data: ByteArray, start: Int, length: Int): Int {
    var crc = 0xFFFF
    
    for (i in start until start + length) {
        crc = crc xor (data[i].toInt() and 0xFF)
        for (j in 0 until 8) {
            if ((crc and 0x0001) != 0) {
                crc = (crc shr 1) xor 0xA001
            } else {
                crc = crc shr 1
            }
        }
    }
    
    return crc and 0xFFFF
}
```

### 명령 예제

#### 전진 + 밸런싱 ON
```kotlin
val header = byteArrayOf(
    0xAA.toByte(),    // start_marker
    0x01,             // version
    0x01,             // msg_type (MOVE_CMD)
    0x00,             // seq_num
    0x08, 0x00,       // payload_len (8 bytes, Little Endian)
    0x00, 0x00        // checksum (계산 필요)
)

val payload = byteArrayOf(
    0x01,             // direction (전진)
    0x00,             // turn (직진)
    0x50,             // speed (80)
    0x01,             // flags (Balance ON)
    0x00, 0x00, 0x00, 0x00  // timestamp
)

val message = header + payload
// checksum 계산: header[6] 이후 데이터 (header[6:] + payload)
val checksumData = message.copyOfRange(6, message.size)
val checksum = calculateCRC16(checksumData, 0, checksumData.size)
message[6] = (checksum and 0xFF).toByte()
message[7] = ((checksum shr 8) and 0xFF).toByte()
```

#### 좌회전
```kotlin
val message = byteArrayOf(
    0xAA.toByte(), 0x01, 0x01, 0x01,    // header (seq_num = 1)
    0x08, 0x00, 0x00, 0x00,             // payload_len + checksum placeholder
    0x00,                                // direction (정지)
    0xD0.toByte(),                       // turn (-48)
    0x30,                                // speed (48)
    0x01,                                // flags (Balance ON)
    0x00, 0x00, 0x00, 0x00              // timestamp
)
// checksum 계산 및 설정
```

#### 기립 명령
```kotlin
val message = byteArrayOf(
    0xAA.toByte(), 0x01, 0x01, 0x02,    // header (seq_num = 2)
    0x08, 0x00, 0x00, 0x00,             // payload_len + checksum placeholder
    0x00,                                // direction (정지)
    0x00,                                // turn (직진)
    0x00,                                // speed (0)
    0x03,                                // flags (Balance + Standup)
    0x00, 0x00, 0x00, 0x00              // timestamp
)
// checksum 계산 및 설정
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
| 0 | balance_kp | 밸런스 PID 비례 게인 | 50.0 | 0.0 ~ 100.0 |
| 1 | balance_ki | 밸런스 PID 적분 게인 | 0.5 | 0.0 ~ 10.0 |
| 2 | balance_kd | 밸런스 PID 미분 게인 | 2.0 | 0.0 ~ 10.0 |
| 3 | velocity_kp | 속도 PID 비례 게인 | 1.0 | 0.0 ~ 10.0 |
| 4 | velocity_ki | 속도 PID 적분 게인 | 0.1 | 0.0 ~ 1.0 |
| 5 | velocity_kd | 속도 PID 미분 게인 | 0.0 | 0.0 ~ 1.0 |
| 6 | kalman_q_angle | 칼만 각도 프로세스 노이즈 | 0.001 | 0.0001 ~ 0.1 |
| 7 | kalman_q_bias | 칼만 바이어스 프로세스 노이즈 | 0.003 | 0.0001 ~ 0.1 |
| 8 | kalman_r_measure | 칼만 측정 노이즈 | 0.03 | 0.001 ~ 1.0 |
| 9 | max_tilt_angle | 최대 기울기 각도 (도) | 45.0 | 10.0 ~ 90.0 |
| 10 | fallen_threshold | 넘어짐 판정 임계값 (도) | 45.0 | 30.0 ~ 90.0 |

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

로봇은 주기적으로(약 1Hz) 상태 정보를 BLE Status Characteristic을 통해 전송합니다.

### 상태 패킷 구조

**전체 메시지 구조: 헤더 (8바이트) + 페이로드 (20바이트) = 28바이트**

#### 프로토콜 헤더 (8바이트)

| 오프셋 | 필드 | 타입 | 값 | 설명 |
|--------|------|------|-----|------|
| 0 | start_marker | uint8 | 0xAA | 패킷 시작 마커 |
| 1 | version | uint8 | 0x01 | 프로토콜 버전 |
| 2 | msg_type | uint8 | 0x03 | 메시지 타입 (STATUS_RESP) |
| 3 | seq_num | uint8 | 0-255 | 시퀀스 번호 |
| 4-5 | payload_len | uint16 | 20 | 페이로드 길이 (Little Endian) |
| 6-7 | checksum | uint16 | - | CRC16 체크섬 (Little Endian) |

#### 상태 응답 페이로드 (20바이트)

| 오프셋 | 필드 | 타입 | 단위 | 설명 |
|--------|------|------|------|------|
| 8-11 | angle | float | 도(degree) | 로봇 기울기 각도 (Little Endian) |
| 12-15 | velocity | float | m/s | 로봇 속도 (Little Endian) |
| 16 | robot_state | uint8 | - | 로봇 상태 코드 |
| 17 | gps_status | uint8 | - | GPS 상태 플래그 |
| 18-21 | latitude | float | 도 | GPS 위도 (Little Endian) |
| 22-25 | longitude | float | 도 | GPS 경도 (Little Endian) |
| 26 | battery_level | uint8 | % | 배터리 잔량 (0-100) |
| 27 | error_flags | uint8 | bit field | 오류 상태 플래그 |

### 로봇 상태 코드 (robot_state)
- `0x00`: IDLE (대기)
- `0x01`: BALANCING (밸런싱 중)
- `0x02`: FALLEN (넘어짐)
- `0x03`: RECOVERY (복구 중)
- `0x04`: ERROR (오류)

### 데이터 파싱 예제

```kotlin
fun parseStatusData(data: ByteArray): RobotStatus? {
    if (data.size < 28) return null
    
    val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
    
    // 헤더 파싱
    val startMarker = buffer.get()
    if (startMarker != 0xAA.toByte()) return null
    
    val version = buffer.get()
    val msgType = buffer.get()
    if (msgType != 0x03.toByte()) return null  // STATUS_RESP
    
    val seqNum = buffer.get()
    val payloadLen = buffer.short
    val checksum = buffer.short
    
    // 체크섬 검증
    val checksumData = data.copyOfRange(6, 8 + payloadLen.toInt())
    val calculatedChecksum = calculateCRC16(checksumData, 0, checksumData.size)
    if (checksum.toInt() != calculatedChecksum) return null
    
    // 페이로드 파싱
    val angle = buffer.float
    val velocity = buffer.float
    val robotState = buffer.get()
    val gpsStatus = buffer.get()
    val latitude = buffer.float
    val longitude = buffer.float
    val batteryLevel = buffer.get().toInt() and 0xFF
    val errorFlags = buffer.get()
    
    return RobotStatus(
        angle = angle,
        velocity = velocity,
        robotState = robotState,
        gpsStatus = gpsStatus,
        latitude = latitude,
        longitude = longitude,
        batteryLevel = batteryLevel,
        errorFlags = errorFlags
    )
}

data class RobotStatus(
    val angle: Float,           // 기울기 각도 (-90 ~ +90도)
    val velocity: Float,        // 속도 (-2.0 ~ +2.0 m/s)
    val robotState: Byte,       // 로봇 상태 코드
    val gpsStatus: Byte,        // GPS 상태
    val latitude: Float,        // GPS 위도
    val longitude: Float,       // GPS 경도
    val batteryLevel: Int,      // 배터리 잔량 (0-100%)
    val errorFlags: Byte        // 오류 플래그
)
```

---

---

## 에러 코드

### BLE 연결 에러
- **연결 실패**: 로봇이 꺼져있거나 범위 밖
- **서비스 없음**: 잘못된 디바이스에 연결
- **특성 없음**: 펌웨어 버전 불일치

### 명령 에러
- **체크섬 불일치**: 이진 패킷 손상 또는 CRC16 계산 오류
- **잘못된 시작 마커**: 0xAA가 아닌 헤더
- **프로토콜 버전 불일치**: 버전 0x01이 아님
- **범위 초과**: 파라미터 값이 유효 범위 벗어남
- **페이로드 길이 오류**: payload_len이 최대값(64) 초과

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
    private var seqNum: Byte = 0
    
    // 로봇 제어 (이진 프로토콜)
    fun sendControlCommand(direction: Int, turn: Int, speed: Int, balance: Boolean, standup: Boolean) {
        val flags = (if (balance) 0x01 else 0x00) or (if (standup) 0x02 else 0x00)
        
        // 헤더 생성
        val header = byteArrayOf(
            0xAA.toByte(),              // start_marker
            0x01,                       // version
            0x01,                       // msg_type (MOVE_CMD)
            seqNum++,                   // seq_num
            0x08, 0x00,                 // payload_len (Little Endian)
            0x00, 0x00                  // checksum placeholder
        )
        
        // 페이로드 생성
        val payload = ByteBuffer.allocate(8).order(ByteOrder.LITTLE_ENDIAN).apply {
            put(direction.toByte())     // direction
            put(turn.toByte())          // turn
            put(speed.toByte())         // speed
            put(flags.toByte())         // flags
            putInt(0)                   // timestamp
        }.array()
        
        // 메시지 결합
        val message = header + payload
        
        // CRC16 체크섬 계산 (header[6] 이후 데이터)
        val checksumData = message.copyOfRange(6, message.size)
        val checksum = calculateCRC16(checksumData, 0, checksumData.size)
        message[6] = (checksum and 0xFF).toByte()
        message[7] = ((checksum shr 8) and 0xFF).toByte()
        
        commandCharacteristic?.let {
            it.value = message
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
    fun parseStatusNotification(data: ByteArray): RobotStatus? {
        if (data.size < 28) return null
        
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        
        // 헤더 검증
        if (buffer.get() != 0xAA.toByte()) return null
        buffer.get() // version
        if (buffer.get() != 0x03.toByte()) return null // STATUS_RESP
        buffer.get() // seq_num
        val payloadLen = buffer.short
        val checksum = buffer.short
        
        // 체크섬 검증
        val checksumData = data.copyOfRange(6, 8 + payloadLen.toInt())
        if (checksum.toInt() != calculateCRC16(checksumData, 0, checksumData.size)) return null
        
        // 페이로드 파싱
        return RobotStatus(
            angle = buffer.float,
            velocity = buffer.float,
            robotState = buffer.get(),
            gpsStatus = buffer.get(),
            latitude = buffer.float,
            longitude = buffer.float,
            batteryLevel = buffer.get().toInt() and 0xFF,
            errorFlags = buffer.get()
        )
    }
    
    private fun calculateCRC16(data: ByteArray, start: Int, length: Int): Int {
        var crc = 0xFFFF
        
        for (i in start until start + length) {
            crc = crc xor (data[i].toInt() and 0xFF)
            for (j in 0 until 8) {
                if ((crc and 0x0001) != 0) {
                    crc = (crc shr 1) xor 0xA001
                } else {
                    crc = crc shr 1
                }
            }
        }
        
        return crc and 0xFFFF
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
  int _seqNum = 0;
  
  // 로봇 제어
  Future<void> sendControlCommand({
    required int direction,
    required int turn, 
    required int speed,
    required bool balance,
    required bool standup
  }) async {
    final flags = (balance ? 0x01 : 0x00) | (standup ? 0x02 : 0x00);
    
    // 헤더 생성
    final header = Uint8List.fromList([
      0xAA,              // start_marker
      0x01,              // version
      0x01,              // msg_type (MOVE_CMD)
      _seqNum++ & 0xFF,  // seq_num
      0x08, 0x00,        // payload_len (Little Endian)
      0x00, 0x00         // checksum placeholder
    ]);
    
    // 페이로드 생성
    final payloadBuffer = ByteData(8);
    payloadBuffer.setInt8(0, direction);
    payloadBuffer.setInt8(1, turn);
    payloadBuffer.setUint8(2, speed);
    payloadBuffer.setUint8(3, flags);
    payloadBuffer.setUint32(4, 0, Endian.little);  // timestamp
    
    final payload = payloadBuffer.buffer.asUint8List();
    
    // 메시지 결합
    final message = Uint8List(16);
    message.setRange(0, 8, header);
    message.setRange(8, 16, payload);
    
    // CRC16 체크섬 계산
    final checksumData = message.sublist(6);
    final checksum = _calculateCRC16(checksumData);
    message[6] = checksum & 0xFF;
    message[7] = (checksum >> 8) & 0xFF;
    
    await commandCharacteristic?.write(message, withoutResponse: true);
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
  RobotStatus? parseStatusData(List<int> data) {
    if (data.length < 28) return null;
    
    final bytes = Uint8List.fromList(data);
    final buffer = ByteData.sublistView(bytes);
    
    // 헤더 검증
    if (buffer.getUint8(0) != 0xAA) return null;
    if (buffer.getUint8(2) != 0x03) return null;  // STATUS_RESP
    
    final payloadLen = buffer.getUint16(4, Endian.little);
    final checksum = buffer.getUint16(6, Endian.little);
    
    // 체크섬 검증
    final checksumData = bytes.sublist(6, 8 + payloadLen);
    if (checksum != _calculateCRC16(checksumData)) return null;
    
    return RobotStatus(
      angle: buffer.getFloat32(8, Endian.little),
      velocity: buffer.getFloat32(12, Endian.little),
      robotState: buffer.getUint8(16),
      gpsStatus: buffer.getUint8(17),
      latitude: buffer.getFloat32(18, Endian.little),
      longitude: buffer.getFloat32(22, Endian.little),
      batteryLevel: buffer.getUint8(26),
      errorFlags: buffer.getUint8(27),
    );
  }
  
  int _calculateCRC16(Uint8List data) {
    int crc = 0xFFFF;
    
    for (int i = 0; i < data.length; i++) {
      crc ^= data[i];
      for (int j = 0; j < 8; j++) {
        if ((crc & 0x0001) != 0) {
          crc = (crc >> 1) ^ 0xA001;
        } else {
          crc = crc >> 1;
        }
      }
    }
    
    return crc & 0xFFFF;
  }
}

class RobotStatus {
  final double angle;
  final double velocity;
  final int robotState;
  final int gpsStatus;
  final double latitude;
  final double longitude;
  final int batteryLevel;
  final int errorFlags;
  
  RobotStatus({
    required this.angle,
    required this.velocity,
    required this.robotState,
    required this.gpsStatus,
    required this.latitude,
    required this.longitude,
    required this.batteryLevel,
    required this.errorFlags,
  });
}
```

---

## 주의사항

1. **연결 안정성**: BLE 연결이 불안정할 수 있으므로 재연결 로직 구현 필요
2. **명령 주기**: 제어 명령은 최대 20Hz로 전송 (50ms 간격)
3. **파라미터 범위**: 텍스트 프로토콜로 파라미터 설정 시 유효 범위 확인 필수
4. **배터리 모니터링**: 배터리 전압이 3.2V 이하로 떨어지면 연결 해제 권장
5. **에러 처리**: CRC16 체크섬 오류나 응답 없음에 대한 적절한 에러 처리 구현
6. **Little Endian**: 모든 멀티바이트 값(uint16, uint32, float)은 Little Endian 바이트 순서 사용
7. **시퀀스 번호**: 명령 패킷마다 시퀀스 번호를 증가시켜 패킷 손실 감지 가능

---

**문의사항이나 버그 리포트는 GitHub Issue로 등록해주세요.**
