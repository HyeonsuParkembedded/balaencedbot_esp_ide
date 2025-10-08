#!/usr/bin/env python3
"""
BalanceBot Real-time Data Plotter
ESP32-C6 균형 로봇의 실시간 데이터를 시리얼 포트에서 읽어와 그래프로 표시합니다.

사용법:
    python realtime_plotter.py --port COM4 --baudrate 115200
    python realtime_plotter.py --port /dev/ttyUSB0

Author: Hyeonsu Park
Date: 2025-10-08
Version: 1.0
"""

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import time
import argparse
import threading
import queue

class BalanceBotPlotter:
    def __init__(self, port, baudrate=115200, max_points=500):
        """
        BalanceBot 실시간 데이터 플로터 초기화
        
        Args:
            port (str): 시리얼 포트 (예: 'COM4', '/dev/ttyUSB0')
            baudrate (int): 통신 속도 (기본값: 115200)
            max_points (int): 표시할 최대 데이터 포인트 수
        """
        self.port = port
        self.baudrate = baudrate
        self.max_points = max_points
        
        # 데이터 저장용 deque (고정 크기)
        self.time_data = deque(maxlen=max_points)
        self.angle_data = deque(maxlen=max_points)
        self.velocity_data = deque(maxlen=max_points)
        self.battery_data = deque(maxlen=max_points)
        self.battery_percent_data = deque(maxlen=max_points)
        
        # 시리얼 통신
        self.serial_conn = None
        self.data_queue = queue.Queue()
        self.running = False
        
        # 시작 시간
        self.start_time = time.time()
        
        # 정규식 패턴 (ESP32 로그 파싱용)
        # 예: "Angle: -2.34 | Velocity: 12.5 | Battery: 7.4V(85%) | GPS: Valid"
        self.data_pattern = re.compile(
            r'Angle:\s*([-+]?\d*\.?\d+)\s*\|\s*'
            r'Velocity:\s*([-+]?\d*\.?\d+)\s*\|\s*'
            r'Battery:\s*([-+]?\d*\.?\d+)V\((\d+)%\)\s*\|\s*'
            r'GPS:\s*(\w+)'
        )
        
    def connect_serial(self):
        """시리얼 포트 연결"""
        try:
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1.0,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            print(f"✅ 시리얼 포트 연결 성공: {self.port} @ {self.baudrate}bps")
            return True
        except Exception as e:
            print(f"❌ 시리얼 포트 연결 실패: {e}")
            return False
    
    def disconnect_serial(self):
        """시리얼 포트 연결 해제"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            print("🔌 시리얼 포트 연결 해제")
    
    def serial_reader_thread(self):
        """시리얼 데이터 읽기 스레드"""
        while self.running:
            try:
                if self.serial_conn and self.serial_conn.is_open:
                    line = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        self.data_queue.put(line)
                else:
                    time.sleep(0.1)
            except Exception as e:
                print(f"⚠️ 시리얼 읽기 오류: {e}")
                time.sleep(0.5)
    
    def parse_data_line(self, line):
        """ESP32 로그 라인을 파싱하여 데이터 추출"""
        match = self.data_pattern.search(line)
        if match:
            try:
                angle = float(match.group(1))
                velocity = float(match.group(2))
                battery_voltage = float(match.group(3))
                battery_percent = int(match.group(4))
                gps_status = match.group(5)
                
                # 현재 시간 (시작점 기준 상대시간)
                current_time = time.time() - self.start_time
                
                # 데이터 추가
                self.time_data.append(current_time)
                self.angle_data.append(angle)
                self.velocity_data.append(velocity)
                self.battery_data.append(battery_voltage)
                self.battery_percent_data.append(battery_percent)
                
                return True
            except (ValueError, IndexError) as e:
                print(f"⚠️ 데이터 파싱 오류: {e}")
        return False
    
    def update_plot(self, frame):
        """matplotlib 애니메이션 업데이트 함수"""
        # 큐에서 새 데이터 처리
        processed_count = 0
        while not self.data_queue.empty() and processed_count < 10:  # 한 번에 최대 10개 처리
            try:
                line = self.data_queue.get_nowait()
                if self.parse_data_line(line):
                    processed_count += 1
            except queue.Empty:
                break
        
        # 데이터가 있으면 그래프 업데이트
        if len(self.time_data) > 0:
            # 서브플롯 클리어
            for ax in self.axes:
                ax.clear()
            
            # 1. 각도 그래프 (위험 구간 표시)
            self.axes[0].plot(self.time_data, self.angle_data, 'b-', linewidth=2, label='Angle')
            self.axes[0].axhline(y=0, color='g', linestyle='--', alpha=0.7, label='Target')
            self.axes[0].axhline(y=45, color='r', linestyle='--', alpha=0.7, label='Danger (+45°)')
            self.axes[0].axhline(y=-45, color='r', linestyle='--', alpha=0.7, label='Danger (-45°)')
            self.axes[0].fill_between(self.time_data, -45, 45, alpha=0.1, color='green', label='Safe Zone')
            self.axes[0].set_ylabel('각도 (°)')
            self.axes[0].set_title('🤖 BalanceBot 실시간 모니터링')
            self.axes[0].legend(loc='upper right')
            self.axes[0].grid(True, alpha=0.3)
            
            # 2. 속도 그래프
            self.axes[1].plot(self.time_data, self.velocity_data, 'g-', linewidth=2, label='Velocity')
            self.axes[1].axhline(y=0, color='gray', linestyle='-', alpha=0.5)
            self.axes[1].set_ylabel('속도 (cm/s)')
            self.axes[1].legend(loc='upper right')
            self.axes[1].grid(True, alpha=0.3)
            
            # 3. 배터리 전압 및 퍼센트
            ax3_twin = self.axes[2].twinx()
            
            # 전압 (왼쪽 축)
            line1 = self.axes[2].plot(self.time_data, self.battery_data, 'r-', linewidth=2, label='Voltage')
            self.axes[2].axhline(y=6.8, color='orange', linestyle='--', alpha=0.7, label='Low (6.8V)')
            self.axes[2].axhline(y=6.4, color='red', linestyle='--', alpha=0.7, label='Critical (6.4V)')
            self.axes[2].set_ylabel('배터리 전압 (V)', color='r')
            self.axes[2].tick_params(axis='y', labelcolor='r')
            
            # 퍼센트 (오른쪽 축)
            line2 = ax3_twin.plot(self.time_data, self.battery_percent_data, 'orange', linewidth=2, label='Percent')
            ax3_twin.set_ylabel('배터리 잔량 (%)', color='orange')
            ax3_twin.tick_params(axis='y', labelcolor='orange')
            ax3_twin.set_ylim(0, 100)
            
            # 범례 통합
            lines = line1 + line2
            labels = [l.get_label() for l in lines]
            self.axes[2].legend(lines, labels, loc='upper right')
            self.axes[2].grid(True, alpha=0.3)
            
            # X축 설정 (모든 서브플롯)
            for ax in self.axes:
                ax.set_xlabel('시간 (초)')
                if len(self.time_data) > 50:  # 50초 이상 데이터가 있으면 최근 데이터만 표시
                    ax.set_xlim(self.time_data[-1] - 30, self.time_data[-1] + 2)
            
            # 현재 상태 표시
            if len(self.time_data) > 0:
                current_angle = self.angle_data[-1]
                current_velocity = self.velocity_data[-1]
                current_battery = self.battery_data[-1]
                current_percent = self.battery_percent_data[-1]
                
                # 상태 색상 결정
                if abs(current_angle) > 45:
                    status_color = 'red'
                    status_text = '⚠️ FALLEN'
                elif abs(current_angle) > 30:
                    status_color = 'orange'
                    status_text = '⚡ UNSTABLE'
                else:
                    status_color = 'green'
                    status_text = '✅ BALANCING'
                
                # 상태 텍스트 표시
                self.fig.suptitle(
                    f'{status_text} | Angle: {current_angle:.2f}° | Vel: {current_velocity:.1f}cm/s | '
                    f'Battery: {current_battery:.1f}V ({current_percent}%)', 
                    fontsize=12, color=status_color, weight='bold'
                )
        
        return self.axes
    
    def start_plotting(self):
        """실시간 플로팅 시작"""
        if not self.connect_serial():
            return
        
        # 스레드 시작
        self.running = True
        serial_thread = threading.Thread(target=self.serial_reader_thread)
        serial_thread.daemon = True
        serial_thread.start()
        
        # matplotlib 설정
        plt.style.use('dark_background')  # 다크 테마
        self.fig, self.axes = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('🤖 BalanceBot ESP32-C6 실시간 모니터링', fontsize=16, weight='bold')
        
        # 서브플롯 간격 조정
        plt.tight_layout()
        plt.subplots_adjust(top=0.93, bottom=0.07)
        
        # 애니메이션 시작 (100ms 간격)
        ani = animation.FuncAnimation(
            self.fig, 
            self.update_plot, 
            interval=100, 
            blit=False,
            cache_frame_data=False
        )
        
        print("📊 실시간 그래프 시작! (Ctrl+C로 종료)")
        print("📈 데이터 수신 대기 중...")
        
        try:
            plt.show()
        except KeyboardInterrupt:
            print("\n🛑 사용자 중단")
        finally:
            self.running = False
            self.disconnect_serial()

def main():
    """메인 함수"""
    parser = argparse.ArgumentParser(
        description="BalanceBot ESP32-C6 실시간 데이터 플로터",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
사용 예시:
  Windows:     python realtime_plotter.py --port COM4
  Linux/Mac:   python realtime_plotter.py --port /dev/ttyUSB0
  
ESP32 시리얼 모니터와 동시 사용 불가! ESP32가 연결된 포트를 정확히 입력하세요.
        """
    )
    
    parser.add_argument(
        '--port', '-p', 
        required=True,
        help='시리얼 포트 (예: COM4, /dev/ttyUSB0)'
    )
    
    parser.add_argument(
        '--baudrate', '-b',
        type=int,
        default=115200,
        help='통신 속도 (기본값: 115200)'
    )
    
    parser.add_argument(
        '--points', 
        type=int,
        default=500,
        help='표시할 최대 데이터 포인트 수 (기본값: 500)'
    )
    
    args = parser.parse_args()
    
    print("🤖 BalanceBot ESP32-C6 실시간 데이터 플로터 v1.0")
    print("=" * 60)
    print(f"📡 포트: {args.port}")
    print(f"⚡ 통신속도: {args.baudrate}bps")
    print(f"📊 최대 포인트: {args.points}개")
    print("=" * 60)
    
    # 플로터 생성 및 실행
    plotter = BalanceBotPlotter(args.port, args.baudrate, args.points)
    plotter.start_plotting()

if __name__ == '__main__':
    main()