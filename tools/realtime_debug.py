#!/usr/bin/env python3
"""
실시간 ESP32 센서 데이터 플로팅 도구
- 시리얼 포트에서 센서 데이터를 실시간으로 읽어서 그래프로 표시
"""

import queue
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import serial
import argparse
import time
import math
import random
from datetime import datetime

# 전역 변수
sensor_queue = queue.Queue()
ser = None
x_vals = []
y_vals = []
z_vals = []
data_count = 0

class RealTimePlotter:
    """실시간 센서 데이터 플로터"""
    
    def __init__(self, port='COM3', baudrate=115200, max_points=1000, demo_mode=False):
        """
        초기화
        
        Args:
            port (str): 시리얼 포트 (Linux에서는 /dev/ttyUSB0 등)
            baudrate (int): 보드레이트
            max_points (int): 그래프에 표시할 최대 데이터 점수
            demo_mode (bool): 데모 모드 (가상 데이터 생성)
        """
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        self.demo_mode = demo_mode
        self.running = False
        self.demo_counter = 0
    
    def read_serial_data(self):
        """시리얼 데이터를 읽는 쓰레드 함수"""
        global ser, sensor_queue
        while self.running:
            try:
                if self.demo_mode:
                    # 데모 모드: 가상 센서 데이터 생성
                    self.demo_counter += 0.1
                    sensor1 = math.sin(self.demo_counter) * 50 + random.random() * 10
                    sensor2 = math.cos(self.demo_counter * 1.5) * 30 + random.random() * 5
                    values = [sensor1, sensor2]
                    sensor_queue.put(values)
                    time.sleep(0.1)  # 100ms 간격
                elif ser and ser.in_waiting > 0:
                    line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        # 쉼표로 구분된 숫자 데이터 파싱
                        try:
                            values = list(map(float, line.split(',')))
                            sensor_queue.put(values)
                            print(f"데이터 수신: {values}")
                        except ValueError:
                            # 숫자가 아닌 데이터는 무시
                            print(f"텍스트 수신: {line}")
            except Exception as e:
                print(f"데이터 읽기 오류: {e}")
                time.sleep(0.1)
            time.sleep(0.01)  # CPU 사용량 제어
    
    def update_plot(self, frame):
        """그래프 업데이트 함수"""
        global sensor_queue, x_vals, y_vals, z_vals, data_count
        
        # 큐에서 새로운 데이터 처리
        while not sensor_queue.empty():
            try:
                data = sensor_queue.get_nowait()
                data_count += 1
                
                # 데이터 길이에 따라 처리
                if len(data) >= 1:
                    x_vals.append(data_count)
                    y_vals.append(data[0])
                    
                if len(data) >= 2:
                    if len(z_vals) < len(y_vals):
                        z_vals.extend([0] * (len(y_vals) - len(z_vals)))
                    z_vals.append(data[1])
                
                # 데이터 점수 제한
                if len(x_vals) > self.max_points:
                    x_vals.pop(0)
                    y_vals.pop(0)
                    if z_vals:
                        z_vals.pop(0)
                        
            except queue.Empty:
                break
            except Exception as e:
                print(f"데이터 처리 오류: {e}")
        
        # 그래프 그리기
        plt.cla()
        if x_vals and y_vals:
            plt.plot(x_vals, y_vals, 'b-', label='센서 1', linewidth=1)
            
        if x_vals and z_vals and len(z_vals) > 0:
            # z_vals 길이를 x_vals와 맞춤
            z_vals_aligned = z_vals[-len(x_vals):] if len(z_vals) >= len(x_vals) else z_vals
            x_vals_aligned = x_vals[-len(z_vals_aligned):]
            plt.plot(x_vals_aligned, z_vals_aligned, 'r-', label='센서 2', linewidth=1)
        
        plt.xlabel('데이터 포인트')
        plt.ylabel('센서 값')
        plt.title('ESP32 실시간 센서 데이터')
        plt.legend()
        plt.grid(True, alpha=0.3)
        
        # Y축 범위 자동 조정
        if y_vals or z_vals:
            all_vals = y_vals + z_vals
            if all_vals:
                margin = (max(all_vals) - min(all_vals)) * 0.1
                plt.ylim(min(all_vals) - margin, max(all_vals) + margin)
    
    def start(self):
        """실시간 플로팅 시작"""
        global ser
        
        try:
            if self.demo_mode:
                print("🎮 데모 모드: 가상 센서 데이터로 테스트")
                ser = None
            else:
                # 시리얼 포트 열기
                ser = serial.Serial(self.port, self.baudrate, timeout=1)
                print(f"📡 시리얼 포트 연결됨: {self.port} @ {self.baudrate} bps")
            
            self.running = True
            
            # 데이터 읽기 쓰레드 시작
            data_thread = threading.Thread(target=self.read_serial_data, daemon=True)
            data_thread.start()
            
            # matplotlib 설정
            plt.style.use('seaborn-v0_8' if 'seaborn-v0_8' in plt.style.available else 'default')
            fig, ax = plt.subplots(figsize=(12, 6))
            
            # 애니메이션 시작
            ani = FuncAnimation(fig, self.update_plot, interval=50, cache_frame_data=False)
            
            print("📊 실시간 그래프 시작됨 (창을 닫으면 종료됩니다)")
            plt.show()
            
        except serial.SerialException as e:
            print(f"❌ 시리얼 포트 오류: {e}")
            print("💡 포트명을 확인하고 다른 프로그램에서 사용중이 아닌지 확인하세요")
        except Exception as e:
            print(f"❌ 오류 발생: {e}")
        finally:
            self.running = False
            if ser:
                ser.close()
                print("� 시리얼 포트 종료됨")

def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(description='ESP32 실시간 센서 데이터 플로팅 도구')
    
    # 선택적 인수
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0',
                      help='시리얼 포트 (기본값: /dev/ttyUSB0, Windows: COM3)')
    parser.add_argument('-b', '--baudrate', type=int, default=115200,
                      help='보드레이트 (기본값: 115200)')
    parser.add_argument('-m', '--max-points', type=int, default=1000,
                      help='그래프에 표시할 최대 데이터 점수 (기본값: 1000)')
    parser.add_argument('--demo', action='store_true',
                      help='데모 모드 (가상 데이터로 테스트)')
    
    args = parser.parse_args()
    
    print("🚀 ESP32 실시간 센서 데이터 플로터")
    print(f"포트: {args.port}")
    print(f"보드레이트: {args.baudrate}")
    print("=" * 50)
    
    # 실시간 플로터 시작
    plotter = RealTimePlotter(
        port=args.port,
        baudrate=args.baudrate,
        max_points=args.max_points,
        demo_mode=args.demo
    )
    
    try:
        plotter.start()
        return 0
    except KeyboardInterrupt:
        print("\n⚠️ 사용자에 의해 중단됨")
        return 0
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        return 1

if __name__ == "__main__":
    exit(main())