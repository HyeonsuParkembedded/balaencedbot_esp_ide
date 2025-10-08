#!/usr/bin/env python3
"""
BalanceBot 통합 시뮬레이터
시뮬레이터와 그래프 플로터를 하나의 프로그램으로 실행
"""

import subprocess
import threading
import sys
import os
import time
import signal
import queue
from pathlib import Path

# matplotlib 설정 (GUI 백엔드 사용)
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
from tkinter import ttk

class IntegratedBalanceBotSimulator:
    def __init__(self):
        self.simulator_process = None
        self.data_queue = queue.Queue()
        self.running = False
        
        # 데이터 저장
        self.time_data = []
        self.angle_data = []
        self.velocity_data = []
        self.pid_data = []
        self.max_points = 200
        
        # GUI 초기화
        self.setup_gui()
        
    def setup_gui(self):
        """GUI 창 설정"""
        self.fig, (self.ax1, self.ax2, self.ax3) = plt.subplots(3, 1, figsize=(12, 10))
        self.fig.suptitle('BalanceBot Real-time Monitor', fontsize=16)
        
        # 서브플롯 설정
        self.setup_subplots()
        
        # 애니메이션 설정
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=50, blit=False)
        
        # 창 닫기 이벤트 처리
        self.fig.canvas.mpl_connect('close_event', self.on_close)
        
    def setup_subplots(self):
        """서브플롯 설정"""
        # 각도 그래프
        self.ax1.set_title('Robot Angle (degrees)')
        self.ax1.set_ylabel('Angle (°)')
        self.ax1.grid(True, alpha=0.3)
        self.ax1.axhspan(-15, 15, alpha=0.2, color='green', label='Safe Zone')
        self.ax1.axhspan(-30, -15, alpha=0.2, color='yellow', label='Warning Zone')
        self.ax1.axhspan(15, 30, alpha=0.2, color='yellow')
        self.ax1.axhspan(-45, -30, alpha=0.2, color='red', label='Danger Zone')
        self.ax1.axhspan(30, 45, alpha=0.2, color='red')
        self.ax1.set_ylim(-45, 45)
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=2, label='Angle')
        self.ax1.legend(loc='upper right')
        
        # 속도 그래프
        self.ax2.set_title('Robot Velocity (m/s)')
        self.ax2.set_ylabel('Velocity (m/s)')
        self.ax2.grid(True, alpha=0.3)
        self.ax2.axhspan(-2, 2, alpha=0.2, color='green', label='Normal Speed')
        self.ax2.axhspan(-5, -2, alpha=0.2, color='yellow', label='High Speed')
        self.ax2.axhspan(2, 5, alpha=0.2, color='yellow')
        self.ax2.set_ylim(-5, 5)
        self.line2, = self.ax2.plot([], [], 'g-', linewidth=2, label='Velocity')
        self.ax2.legend(loc='upper right')
        
        # PID 출력 그래프
        self.ax3.set_title('PID Control Output')
        self.ax3.set_ylabel('PID Output')
        self.ax3.set_xlabel('Time (seconds)')
        self.ax3.grid(True, alpha=0.3)
        self.ax3.axhspan(-100, 100, alpha=0.2, color='green', label='Normal Range')
        self.ax3.set_ylim(-200, 200)
        self.line3, = self.ax3.plot([], [], 'r-', linewidth=2, label='PID Output')
        self.ax3.legend(loc='upper right')
        
        plt.tight_layout()
        
    def start_simulator(self):
        """시뮬레이터 프로세스 시작"""
        try:
            # 시뮬레이터 실행 파일 경로
            sim_path = Path(__file__).parent.parent / "posix_simulator" / "build" / "balancebot_posix"
            
            if not sim_path.exists():
                print(f"❌ 시뮬레이터를 찾을 수 없습니다: {sim_path}")
                print("먼저 시뮬레이터를 빌드해주세요:")
                print("cd posix_simulator/build && cmake .. && make")
                return False
                
            print(f"🚀 시뮬레이터 시작: {sim_path}")
            
            # 시뮬레이터 시작 (버퍼링 없이)
            self.simulator_process = subprocess.Popen(
                [str(sim_path)],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,  # stderr도 stdout으로 합침
                stdin=subprocess.PIPE,
                text=True,
                bufsize=0,  # 버퍼링 없음
                universal_newlines=True,
                preexec_fn=None
            )
            
            # 프로세스가 정상 시작되었는지 확인
            import time
            time.sleep(0.5)  # 잠시 대기
            
            if self.simulator_process.poll() is not None:
                print(f"❌ 시뮬레이터 프로세스가 종료됨 (코드: {self.simulator_process.returncode})")
                return False
            
            # 데이터 읽기 스레드 시작
            self.data_thread = threading.Thread(target=self.read_simulator_data, daemon=True)
            self.data_thread.start()
            
            print("✅ 시뮬레이터 시작됨")
            return True
            
        except Exception as e:
            print(f"❌ 시뮬레이터 시작 실패: {e}")
            return False
            
    def read_simulator_data(self):
        """시뮬레이터 데이터 읽기 (별도 스레드)"""
        line_count = 0
        try:
            print("📥 데이터 읽기 스레드 시작됨")
            
            while self.running and self.simulator_process:
                # 프로세스 상태 확인
                if self.simulator_process.poll() is not None:
                    print(f"❌ 시뮬레이터 프로세스 종료됨 (코드: {self.simulator_process.returncode})")
                    break
                
                try:
                    # 타임아웃을 추가하여 블로킹 방지
                    line = self.simulator_process.stdout.readline()
                    if not line:
                        print("❌ 시뮬레이터에서 더 이상 데이터가 없습니다")
                        break
                        
                    line = line.strip()
                    if line:
                        line_count += 1
                        # 처음 몇 줄과 매 50줄마다 디버그 출력
                        if line_count <= 10 or line_count % 50 == 0:
                            print(f"📥 Raw data [{line_count}]: {line[:80]}...")
                        
                        # 데이터 큐에 추가
                        self.data_queue.put(line)
                        
                except Exception as read_error:
                    print(f"❌ 라인 읽기 오류: {read_error}")
                    break
                    
        except Exception as e:
            print(f"❌ 데이터 읽기 스레드 오류: {e}")
        finally:
            print(f"📊 데이터 읽기 스레드 종료 - 총 {line_count}줄 읽음")
            
    def parse_data_line(self, line):
        """시뮬레이터 출력 데이터 파싱"""
        current_time = time.time()
        
        try:
            # POSIX 시뮬레이터 출력 형식 파싱
            # [DEBUG][POSIX_SIM] Balance: Angle=0.00°, Output=0.08
            # [PHYSICS] Pos=-0.00m, Vel=0.00m/s, Angle=0.0°, Target=0.00m/s
            
            if "[DEBUG][POSIX_SIM] Balance:" in line:
                # Balance 출력에서 Angle과 PID Output 추출
                parts = line.split("Balance: ")[1]  # "Angle=0.00°, Output=0.08"
                angle_part, output_part = parts.split(", ")
                
                angle_str = angle_part.split("=")[1].replace("°", "")  # "0.00"
                output_str = output_part.split("=")[1]  # "0.08"
                
                angle_val = float(angle_str)
                pid_val = float(output_str)
                
                # 속도는 기본값으로 설정 (별도 파싱 필요)
                velocity_val = 0.0
                
                return current_time, angle_val, velocity_val, pid_val
                
            elif "[PHYSICS]" in line and "Vel=" in line:
                # Physics 출력에서 위치, 속도, 각도 추출
                # [PHYSICS] Pos=-0.00m, Vel=0.00m/s, Angle=0.0°, Target=0.00m/s
                parts = line.split("[PHYSICS] ")[1]  
                data_parts = parts.split(", ")
                
                pos_str = data_parts[0].split("=")[1].replace("m", "")
                vel_str = data_parts[1].split("=")[1].replace("m/s", "")
                angle_str = data_parts[2].split("=")[1].replace("°", "")
                
                angle_val = float(angle_str)
                velocity_val = float(vel_str)
                pid_val = 0.0  # Physics 출력에는 PID 없음
                
                return current_time, angle_val, velocity_val, pid_val
                
        except Exception as e:
            # 파싱 오류 시 디버그 출력
            if "Balance:" in line or "[PHYSICS]" in line:
                print(f"❌ 파싱 오류: {line[:50]}... -> {e}")
            pass
            
        # 파싱되지 않은 중요한 라인 체크
        if any(keyword in line for keyword in ["Balance:", "[PHYSICS]", "Angle", "Output"]):
            print(f"🔍 파싱 실패한 중요 라인: {line}")
            
        return None
        
    def update_plot(self, frame):
        """그래프 업데이트"""
        # 큐에서 새 데이터 처리
        data_count = 0
        while not self.data_queue.empty():
            try:
                line = self.data_queue.get_nowait()
                parsed = self.parse_data_line(line)
                
                if parsed:
                    time_val, angle_val, velocity_val, pid_val = parsed
                    
                    # 데이터 추가
                    self.time_data.append(time_val)
                    self.angle_data.append(angle_val)
                    self.velocity_data.append(velocity_val)
                    self.pid_data.append(pid_val)
                    
                    # 처음 몇 개 데이터는 로깅
                    data_count += 1
                    if data_count < 5 or len(self.time_data) % 50 == 0:
                        print(f"📊 Data: Angle={angle_val:.2f}°, Vel={velocity_val:.2f}, PID={pid_val:.2f}")
                    
                    # 오래된 데이터 제거 (최대 포인트 수 유지)
                    if len(self.time_data) > self.max_points:
                        self.time_data.pop(0)
                        self.angle_data.pop(0)
                        self.velocity_data.pop(0)
                        self.pid_data.pop(0)
                        
            except queue.Empty:
                break
        
        # 그래프 업데이트
        if self.time_data:
            self.line1.set_data(self.time_data, self.angle_data)
            self.line2.set_data(self.time_data, self.velocity_data)
            self.line3.set_data(self.time_data, self.pid_data)
            
            # X축 범위 자동 조정
            if len(self.time_data) > 1:
                time_range = max(self.time_data) - min(self.time_data)
                if time_range > 0:
                    for ax in [self.ax1, self.ax2, self.ax3]:
                        ax.set_xlim(min(self.time_data), max(self.time_data))
        
        return self.line1, self.line2, self.line3
        
    def send_command(self, command):
        """시뮬레이터에 명령 전송"""
        if self.simulator_process and self.simulator_process.stdin:
            try:
                self.simulator_process.stdin.write(command + '\n')
                self.simulator_process.stdin.flush()
            except:
                pass
                
    def on_close(self, event):
        """창 닫기 이벤트 처리"""
        self.stop()
        
    def stop(self):
        """시뮬레이터 정지"""
        print("\n🛑 시뮬레이터 정지 중...")
        self.running = False
        
        if self.simulator_process:
            self.simulator_process.terminate()
            self.simulator_process.wait(timeout=3)
            if self.simulator_process.poll() is None:
                self.simulator_process.kill()
            self.simulator_process = None
            
        plt.close('all')
        
    def run(self):
        """메인 실행 함수"""
        print("🤖 BalanceBot 통합 시뮬레이터 시작")
        print("=" * 60)
        
        # 시뮬레이터 시작
        if not self.start_simulator():
            return
            
        self.running = True
        
        # 사용법 출력
        print("📋 키보드 조작법:")
        print("  ↑/w : 전진")
        print("  ↓/s : 후진") 
        print("  ←/a : 좌회전")
        print("  →/d : 우회전")
        print("  SPACE : 정지")
        print("  b : 밸런스 토글")
        print("  u : 기립")
        print("  q : 종료")
        print("📊 실시간 그래프가 새 창에서 열립니다...")
        print("=" * 60)
        
        try:
            # 키보드 입력 처리 스레드
            input_thread = threading.Thread(target=self.handle_keyboard_input, daemon=True)
            input_thread.start()
            
            # GUI 표시
            plt.show()
            
        except KeyboardInterrupt:
            print("\n⌨️  Ctrl+C 감지됨")
        finally:
            self.stop()
            
    def handle_keyboard_input(self):
        """키보드 입력 처리 (별도 스레드)"""
        try:
            import termios, tty
            
            # 터미널 설정 백업
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            
            try:
                tty.setraw(sys.stdin.fileno())
                
                while self.running:
                    char = sys.stdin.read(1)
                    
                    if char == 'q':
                        self.stop()
                        break
                    elif char == 'w':  # 전진
                        self.send_command('w')
                    elif char == 's':  # 후진
                        self.send_command('s')
                    elif char == 'a':  # 좌회전
                        self.send_command('a')
                    elif char == 'd':  # 우회전
                        self.send_command('d')
                    elif char == ' ':  # 정지
                        self.send_command(' ')
                    elif char == 'b':  # 밸런스 토글
                        self.send_command('b')
                    elif char == 'u':  # 기립
                        self.send_command('u')
                    elif char == '\x1b':  # ESC 시퀀스 시작 (방향키)
                        char = sys.stdin.read(1)
                        if char == '[':
                            char = sys.stdin.read(1)
                            if char == 'A':    # ↑ (전진)
                                self.send_command('w')
                            elif char == 'B':  # ↓ (후진)
                                self.send_command('s')
                            elif char == 'C':  # → (우회전)
                                self.send_command('d')
                            elif char == 'D':  # ← (좌회전)
                                self.send_command('a')
                                
            finally:
                # 터미널 설정 복원
                termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
                
        except Exception as e:
            print(f"키보드 입력 처리 오류: {e}")

def main():
    """메인 함수"""
    try:
        simulator = IntegratedBalanceBotSimulator()
        simulator.run()
    except Exception as e:
        print(f"❌ 오류 발생: {e}")
        sys.exit(1)

if __name__ == "__main__":
    main()