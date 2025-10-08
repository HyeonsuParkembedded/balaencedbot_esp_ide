#!/usr/bin/env python3
"""
BalanceBot Simulator Real-time Data Plotter
POSIX 시뮬레이터의 출력을 실시간 그래프로 표시합니다.

사용법:
    cd /home/hyunsu5203/balaencedbot_esp_ide/posix_simulator/build
    ./balancebot_posix | python3 ../../tools/simulator_plotter.py

Author: Hyeonsu Park
Date: 2025-10-08
Version: 1.0
"""

import matplotlib
matplotlib.use('TkAgg')  # GUI 백엔드 강제 설정
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re
import sys
import threading
import queue
import time
import tkinter as tk
from tkinter import ttk

class SimulatorPlotter:
    def __init__(self, max_points=500):
        """
        시뮬레이터 실시간 데이터 플로터 초기화
        
        Args:
            max_points (int): 표시할 최대 데이터 포인트 수
        """
        self.max_points = max_points
        
        # 데이터 저장용 deque (고정 크기)
        self.time_data = deque(maxlen=max_points)
        self.angle_data = deque(maxlen=max_points)
        self.velocity_data = deque(maxlen=max_points)
        self.pid_output_data = deque(maxlen=max_points)
        
        # 데이터 큐 (스레드 간 통신)
        self.data_queue = queue.Queue()
        self.running = False
        
        # 시작 시간
        self.start_time = time.time()
        
    def stdin_reader_thread(self):
        """표준입력에서 시뮬레이터 출력 읽기 스레드"""
        try:
            while self.running:
                line = sys.stdin.readline()
                if not line:  # EOF
                    break
                self.data_queue.put(line.strip())
        except Exception as e:
            print(f"❌ 입력 읽기 오류: {e}")
            
    def parse_data_line(self, line):
        """시뮬레이터 출력 라인을 파싱하여 데이터 추출"""
        try:
            current_time = time.time() - self.start_time
            
            # 각도 데이터 파싱
            angle_match = re.search(r'Angle=(-?\d+\.?\d*)°', line)
            if angle_match:
                angle = float(angle_match.group(1))
                self.time_data.append(current_time)
                self.angle_data.append(angle)
                return True
                
            # 속도 데이터 파싱 (Status 출력에서)
            velocity_match = re.search(r'Velocity:\s*(-?\d+\.?\d*)\s*cm/s', line)
            if velocity_match:
                velocity = float(velocity_match.group(1))
                if len(self.velocity_data) == 0 or len(self.velocity_data) < len(self.time_data):
                    self.velocity_data.append(velocity)
                else:
                    self.velocity_data[-1] = velocity
                return True
                
            # PID 출력 데이터 파싱
            pid_match = re.search(r'Output=(-?\d+\.?\d*)', line)
            if pid_match:
                pid_output = float(pid_match.group(1))
                if len(self.pid_output_data) == 0 or len(self.pid_output_data) < len(self.time_data):
                    self.pid_output_data.append(pid_output)
                else:
                    self.pid_output_data[-1] = pid_output
                return True
                
        except Exception as e:
            print(f"❌ 데이터 파싱 오류: {e}")
            
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
            self.axes[0].plot(self.time_data, self.angle_data, 'b-', linewidth=2.5, label='Robot Tilt', alpha=0.8)
            self.axes[0].axhline(y=45, color='red', linestyle='--', alpha=0.8, linewidth=2, label='Danger (±45°)')
            self.axes[0].axhline(y=-45, color='red', linestyle='--', alpha=0.8, linewidth=2)
            self.axes[0].axhline(y=30, color='orange', linestyle='--', alpha=0.7, linewidth=1.5, label='Warning (±30°)')
            self.axes[0].axhline(y=-30, color='orange', linestyle='--', alpha=0.7, linewidth=1.5)
            self.axes[0].axhline(y=0, color='green', linestyle='-', alpha=0.8, linewidth=2, label='Balance (0°)')
            
            # 위험 구간 색칠
            self.axes[0].axhspan(45, 50, color='red', alpha=0.1)
            self.axes[0].axhspan(-50, -45, color='red', alpha=0.1)
            self.axes[0].axhspan(30, 45, color='orange', alpha=0.1)
            self.axes[0].axhspan(-45, -30, color='orange', alpha=0.1)
            self.axes[0].axhspan(-30, 30, color='green', alpha=0.05)
            
            self.axes[0].set_ylabel('Angle (deg)', fontsize=12, weight='bold')
            self.axes[0].set_title('Robot Angle Monitoring', fontsize=14, weight='bold', pad=15)
            self.axes[0].legend(loc='upper right', framealpha=0.9)
            self.axes[0].grid(True, alpha=0.3, linestyle=':')
            self.axes[0].set_ylim(-50, 50)
            
            # 2. 속도 그래프
            if len(self.velocity_data) > 0:
                time_vel = list(self.time_data)[-len(self.velocity_data):]
                self.axes[1].plot(time_vel, self.velocity_data, 'green', linewidth=2.5, label='Robot Speed', alpha=0.8)
                self.axes[1].axhline(y=0, color='gray', linestyle='-', alpha=0.8, linewidth=2)
                
                # 속도 구간 표시
                if len(self.velocity_data) > 0:
                    current_vel = self.velocity_data[-1]
                    if current_vel > 0:
                        self.axes[1].axhspan(0, max(self.velocity_data) if self.velocity_data else 10, 
                                           color='lightgreen', alpha=0.1)
                    elif current_vel < 0:
                        self.axes[1].axhspan(min(self.velocity_data) if self.velocity_data else -10, 0, 
                                           color='lightcoral', alpha=0.1)
                
                self.axes[1].set_ylabel('Velocity (cm/s)', fontsize=12, weight='bold')
                self.axes[1].set_title('Robot Velocity', fontsize=14, weight='bold', pad=15)
                self.axes[1].legend(loc='upper right', framealpha=0.9)
                self.axes[1].grid(True, alpha=0.3, linestyle=':')
                
            # 3. PID 출력 그래프
            if len(self.pid_output_data) > 0:
                time_pid = list(self.time_data)[-len(self.pid_output_data):]
                self.axes[2].plot(time_pid, self.pid_output_data, 'purple', linewidth=2.5, label='PID Control', alpha=0.8)
                self.axes[2].axhline(y=0, color='gray', linestyle='-', alpha=0.8, linewidth=2)
                
                # PID 출력 구간 표시
                if len(self.pid_output_data) > 0:
                    current_pid = self.pid_output_data[-1]
                    if abs(current_pid) > 50:  # 강한 제어
                        color = 'red'
                        alpha = 0.15
                    elif abs(current_pid) > 20:  # 중간 제어
                        color = 'orange'
                        alpha = 0.1
                    else:  # 약한 제어
                        color = 'lightblue'
                        alpha = 0.08
                    
                    if current_pid > 0:
                        self.axes[2].axhspan(0, max(self.pid_output_data) if self.pid_output_data else 100, 
                                           color=color, alpha=alpha)
                    else:
                        self.axes[2].axhspan(min(self.pid_output_data) if self.pid_output_data else -100, 0, 
                                           color=color, alpha=alpha)
                
                self.axes[2].set_ylabel('PID 출력', fontsize=12, weight='bold')
                self.axes[2].set_title('PID Control Signal', fontsize=14, weight='bold', pad=15)
                self.axes[2].legend(loc='upper right', framealpha=0.9)
                self.axes[2].grid(True, alpha=0.3, linestyle=':')
            
            # X축 설정 (모든 서브플롯)
            for i, ax in enumerate(self.axes):
                ax.set_xlabel('Time (seconds)', fontsize=11, weight='bold')
                if len(self.time_data) > 50:  # 50초 이상 데이터가 있으면 최근 데이터만 표시
                    ax.set_xlim(self.time_data[-1] - 30, self.time_data[-1] + 2)
                
                # 각 그래프의 배경색 약간 다르게
                if i == 0:  # 각도
                    ax.set_facecolor('#fafafa')
                elif i == 1:  # 속도
                    ax.set_facecolor('#f8fff8')
                else:  # PID
                    ax.set_facecolor('#faf8ff')
            
            # 현재 상태 표시
            if len(self.time_data) > 0:
                current_angle = self.angle_data[-1] if self.angle_data else 0
                current_velocity = self.velocity_data[-1] if self.velocity_data else 0
                current_pid = self.pid_output_data[-1] if self.pid_output_data else 0
                
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
                status_info = f'{status_text} | Angle: {current_angle:.2f}° | Vel: {current_velocity:.1f}cm/s | PID: {current_pid:.2f}'
                self.fig.suptitle(
                    f'BalanceBot Real-time Monitoring - {status_info}', 
                    fontsize=14, color=status_color, weight='bold'
                )
                
                # 각 서브플롯에 현재 값 표시
                if len(self.angle_data) > 0:
                    self.axes[0].text(0.02, 0.95, f'Current: {current_angle:.2f}°', 
                                    transform=self.axes[0].transAxes, fontsize=11, weight='bold',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
                
                if len(self.velocity_data) > 0:
                    self.axes[1].text(0.02, 0.95, f'Current: {current_velocity:.1f} cm/s', 
                                    transform=self.axes[1].transAxes, fontsize=11, weight='bold',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
                
                if len(self.pid_output_data) > 0:
                    self.axes[2].text(0.02, 0.95, f'Current: {current_pid:.2f}', 
                                    transform=self.axes[2].transAxes, fontsize=11, weight='bold',
                                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white', alpha=0.8))
        
        return self.axes
        
    def start_plotting(self):
        """실시간 플로팅 시작"""
        print("🚀 시뮬레이터 실시간 플로터 시작")
        print("⌨️  키보드 조작: ↑↓←→ (이동), SPACE (밸런스), s (정지), q (종료)")
        print("📊 그래프: 각도, 속도, PID 출력을 실시간 표시")
        print("🖼️  GUI 창에서 그래프를 확인하세요!")
        print("=" * 60)
        
        self.running = True
        
        # matplotlib GUI 설정
        plt.style.use('default')
        self.fig, self.axes = plt.subplots(3, 1, figsize=(14, 10))
        self.fig.suptitle('🤖 BalanceBot 시뮬레이터 실시간 모니터링', fontsize=16, weight='bold')
        
        # 창 설정
        mng = self.fig.canvas.manager
        mng.set_window_title("BalanceBot 실시간 데이터 모니터")
        
        # 창 위치 및 크기 설정 (화면 중앙)
        try:
            # tkinter 백엔드인 경우 창 위치 설정
            if hasattr(mng, 'window'):
                mng.window.wm_geometry("1200x800+100+50")  # 크기x높이+x위치+y위치
        except:
            pass
        
        # 서브플롯 간격 조정
        plt.subplots_adjust(left=0.1, right=0.95, top=0.93, bottom=0.08, hspace=0.3)
        
        # 표준입력 읽기 스레드 시작
        reader_thread = threading.Thread(target=self.stdin_reader_thread, daemon=True)
        reader_thread.start()
        
        try:
            # matplotlib 애니메이션 시작
            ani = animation.FuncAnimation(self.fig, self.update_plot, interval=50, blit=False, cache_frame_data=False)
            
            # GUI 창 표시
            plt.show(block=True)
            
        except KeyboardInterrupt:
            print("\n🔌 사용자에 의해 종료됨")
        except Exception as e:
            print(f"❌ 플로팅 오류: {e}")
        finally:
            self.running = False
            plt.close('all')
            print("📊 플로터 종료")

def main():
    """메인 함수"""
    print("🤖 BalanceBot 시뮬레이터 실시간 데이터 플로터 v1.0")
    print("=" * 60)
    print("사용법: ./balancebot_posix | python3 simulator_plotter.py")
    print("=" * 60)
    
    # 플로터 생성 및 실행
    plotter = SimulatorPlotter(max_points=1000)
    plotter.start_plotting()

if __name__ == '__main__':
    main()