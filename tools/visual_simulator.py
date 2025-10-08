#!/usr/bin/env python3
"""
BalanceBot 2D Visual Simulator
POSIX 시뮬레이터의 출력을 읽어와 2D 로봇 시각화를 제공합니다.

Dependencies:
    pip install pygame matplotlib

사용법:
    cd /home/hyunsu5203/balaencedbot_esp_ide/posix_simulator/build
    ./balancebot_posix | python3 ../../tools/visual_simulator.py

Author: Hyeonsu Park
Date: 2025-10-08
Version: 1.0
"""

import pygame
import math
import sys
import threading
import queue
import time
import re

# 색상 정의
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
GRAY = (128, 128, 128)
ORANGE = (255, 165, 0)
YELLOW = (255, 255, 0)

class VisualSimulator:
    def __init__(self, width=1000, height=700):
        """
        2D 시각 시뮬레이터 초기화
        
        Args:
            width (int): 창 너비
            height (int): 창 높이
        """
        self.width = width
        self.height = height
        self.running = True
        
        # 로봇 상태
        self.robot_angle = 0.0  # 기울기 각도 (도)
        self.robot_velocity = 0.0  # 속도 (cm/s)
        self.robot_position = 0.0  # 위치 (m)
        self.pid_output = 0.0  # PID 출력
        self.balance_enabled = True
        
        # 데이터 큐
        self.data_queue = queue.Queue()
        
        # pygame 초기화
        pygame.init()
        self.screen = pygame.display.set_mode((width, height))
        pygame.display.set_caption("🤖 BalanceBot 2D 시각 시뮬레이터")
        self.clock = pygame.time.Clock()
        self.font = pygame.font.Font(None, 24)
        self.small_font = pygame.font.Font(None, 18)
        
        # 그래프 데이터
        self.angle_history = []
        self.velocity_history = []
        self.max_history = 200
        
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
            # 각도 데이터 파싱
            angle_match = re.search(r'Angle=(-?\d+\.?\d*)°', line)
            if angle_match:
                self.robot_angle = float(angle_match.group(1))
                
            # 속도 데이터 파싱 (Status 출력에서)
            velocity_match = re.search(r'Velocity:\s*(-?\d+\.?\d*)\s*cm/s', line)
            if velocity_match:
                self.robot_velocity = float(velocity_match.group(1))
                
            # 위치 데이터 파싱 (Physics 출력에서)
            pos_match = re.search(r'Pos=(-?\d+\.?\d*)m', line)
            if pos_match:
                self.robot_position = float(pos_match.group(1))
                
            # PID 출력 데이터 파싱
            pid_match = re.search(r'Output=(-?\d+\.?\d*)', line)
            if pid_match:
                self.pid_output = float(pid_match.group(1))
                
            # 밸런스 상태 파싱
            if "밸런싱 활성화" in line:
                self.balance_enabled = True
            elif "밸런싱 비활성화" in line:
                self.balance_enabled = False
                
        except Exception as e:
            print(f"❌ 데이터 파싱 오류: {e}")
            
    def draw_robot(self, surface, x, y, angle, scale=1.0):
        """로봇 그리기"""
        # 로봇 크기
        body_width = 40 * scale
        body_height = 60 * scale
        wheel_radius = 25 * scale
        
        # 각도를 라디안으로 변환
        angle_rad = math.radians(angle)
        
        # 로봇 몸체 (기울어진 직사각형)
        body_points = []
        for dx, dy in [(-body_width/2, -body_height/2), (body_width/2, -body_height/2),
                       (body_width/2, body_height/2), (-body_width/2, body_height/2)]:
            # 회전 적용
            rotated_x = dx * math.cos(angle_rad) - dy * math.sin(angle_rad)
            rotated_y = dx * math.sin(angle_rad) + dy * math.cos(angle_rad)
            body_points.append((x + rotated_x, y + rotated_y))
        
        # 상태에 따른 색상
        if abs(angle) > 45:
            body_color = RED  # 넘어짐
        elif abs(angle) > 30:
            body_color = ORANGE  # 불안정
        else:
            body_color = GREEN if self.balance_enabled else GRAY  # 안정/비활성
            
        pygame.draw.polygon(surface, body_color, body_points)
        pygame.draw.polygon(surface, BLACK, body_points, 2)
        
        # 바퀴 그리기
        wheel_y = y + body_height/2 + wheel_radius/2
        pygame.draw.circle(surface, BLACK, (int(x), int(wheel_y)), int(wheel_radius))
        pygame.draw.circle(surface, GRAY, (int(x), int(wheel_y)), int(wheel_radius-3))
        
        # 중심점 표시
        pygame.draw.circle(surface, RED, (int(x), int(y)), 3)
        
        # 균형선 표시 (수직선)
        line_length = body_height + 20
        end_x = x + line_length * math.sin(angle_rad)
        end_y = y - line_length * math.cos(angle_rad)
        pygame.draw.line(surface, BLUE, (x, y), (end_x, end_y), 3)
        
    def draw_ground(self, surface):
        """바닥 그리기"""
        ground_y = self.height - 150
        pygame.draw.line(surface, BLACK, (0, ground_y), (self.width, ground_y), 3)
        
        # 격자 무늬
        grid_spacing = 50
        for i in range(0, self.width, grid_spacing):
            pygame.draw.line(surface, GRAY, (i, ground_y), (i, ground_y + 10), 1)
            
    def draw_info_panel(self, surface):
        """정보 패널 그리기"""
        panel_x = 20
        panel_y = 20
        panel_width = 300
        panel_height = 200
        
        # 패널 배경
        pygame.draw.rect(surface, WHITE, (panel_x, panel_y, panel_width, panel_height))
        pygame.draw.rect(surface, BLACK, (panel_x, panel_y, panel_width, panel_height), 2)
        
        # 정보 텍스트
        info_lines = [
            f"🤖 BalanceBot 시뮬레이터",
            f"",
            f"각도: {self.robot_angle:.2f}°",
            f"속도: {self.robot_velocity:.1f} cm/s", 
            f"위치: {self.robot_position:.2f} m",
            f"PID 출력: {self.pid_output:.2f}",
            f"밸런스: {'ON' if self.balance_enabled else 'OFF'}",
            f"",
            f"⌨️ 키보드 조작:",
            f"↑↓←→ 화살표키 (이동)",
            f"SPACE (밸런스 토글)",
            f"s (정지), q (종료)"
        ]
        
        for i, line in enumerate(info_lines):
            if line.startswith("각도:"):
                color = RED if abs(self.robot_angle) > 30 else GREEN
            elif line.startswith("밸런스:"):
                color = GREEN if self.balance_enabled else RED
            else:
                color = BLACK
                
            text = self.small_font.render(line, True, color)
            surface.blit(text, (panel_x + 10, panel_y + 15 + i * 15))
            
    def draw_angle_gauge(self, surface):
        """각도 게이지 그리기"""
        gauge_x = self.width - 150
        gauge_y = 100
        gauge_radius = 60
        
        # 게이지 배경
        pygame.draw.circle(surface, WHITE, (gauge_x, gauge_y), gauge_radius)
        pygame.draw.circle(surface, BLACK, (gauge_x, gauge_y), gauge_radius, 2)
        
        # 각도 눈금
        for angle in range(-45, 46, 15):
            angle_rad = math.radians(angle)
            start_x = gauge_x + (gauge_radius - 10) * math.sin(angle_rad)
            start_y = gauge_y - (gauge_radius - 10) * math.cos(angle_rad)
            end_x = gauge_x + gauge_radius * math.sin(angle_rad)
            end_y = gauge_y - gauge_radius * math.cos(angle_rad)
            
            color = RED if abs(angle) > 30 else BLACK
            pygame.draw.line(surface, color, (start_x, start_y), (end_x, end_y), 2)
            
        # 현재 각도 지시침
        angle_rad = math.radians(self.robot_angle)
        needle_x = gauge_x + (gauge_radius - 5) * math.sin(angle_rad)
        needle_y = gauge_y - (gauge_radius - 5) * math.cos(angle_rad)
        
        needle_color = RED if abs(self.robot_angle) > 30 else BLUE
        pygame.draw.line(surface, needle_color, (gauge_x, gauge_y), (needle_x, needle_y), 4)
        
        # 중심점
        pygame.draw.circle(surface, RED, (gauge_x, gauge_y), 5)
        
        # 각도 텍스트
        angle_text = self.font.render(f"{self.robot_angle:.1f}°", True, BLACK)
        text_rect = angle_text.get_rect(center=(gauge_x, gauge_y + gauge_radius + 20))
        surface.blit(angle_text, text_rect)
        
    def draw_graph(self, surface):
        """간단한 그래프 그리기"""
        graph_x = self.width - 400
        graph_y = self.height - 200
        graph_width = 350
        graph_height = 100
        
        # 그래프 배경
        pygame.draw.rect(surface, WHITE, (graph_x, graph_y, graph_width, graph_height))
        pygame.draw.rect(surface, BLACK, (graph_x, graph_y, graph_width, graph_height), 2)
        
        # 중심선
        center_y = graph_y + graph_height // 2
        pygame.draw.line(surface, GRAY, (graph_x, center_y), (graph_x + graph_width, center_y), 1)
        
        # 각도 히스토리 업데이트
        self.angle_history.append(self.robot_angle)
        if len(self.angle_history) > self.max_history:
            self.angle_history.pop(0)
            
        # 각도 그래프 그리기
        if len(self.angle_history) > 1:
            points = []
            for i, angle in enumerate(self.angle_history):
                x = graph_x + i * graph_width // self.max_history
                y = center_y - angle * graph_height // 90  # -45도~45도 범위
                y = max(graph_y, min(graph_y + graph_height, y))  # 범위 제한
                points.append((x, y))
            
            if len(points) > 1:
                pygame.draw.lines(surface, BLUE, False, points, 2)
                
        # 그래프 제목
        title = self.small_font.render("각도 히스토리", True, BLACK)
        surface.blit(title, (graph_x + 5, graph_y - 20))
        
    def update_data(self):
        """데이터 큐에서 새 데이터 처리"""
        count = 0
        while not self.data_queue.empty() and count < 10:
            try:
                line = self.data_queue.get_nowait()
                self.parse_data_line(line)
                count += 1
            except queue.Empty:
                break
                
    def run(self):
        """메인 실행 루프"""
        print("🚀 2D 시각 시뮬레이터 시작")
        print("⌨️  키보드 조작은 시뮬레이터 터미널에서 하세요!")
        print("🎮 이 창은 시각화 전용입니다")
        
        # 표준입력 읽기 스레드 시작
        reader_thread = threading.Thread(target=self.stdin_reader_thread, daemon=True)
        reader_thread.start()
        
        while self.running:
            # 이벤트 처리
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    self.running = False
                    
            # 데이터 업데이트
            self.update_data()
            
            # 화면 그리기
            self.screen.fill(WHITE)
            
            # 바닥 그리기
            self.draw_ground(self.screen)
            
            # 로봇 그리기 (화면 중앙)
            robot_x = self.width // 2 + self.robot_position * 100  # 위치 스케일링
            robot_y = self.height - 200
            self.draw_robot(self.screen, robot_x, robot_y, self.robot_angle, scale=1.5)
            
            # UI 요소들 그리기
            self.draw_info_panel(self.screen)
            self.draw_angle_gauge(self.screen)
            self.draw_graph(self.screen)
            
            # 화면 업데이트
            pygame.display.flip()
            self.clock.tick(60)  # 60 FPS
            
        pygame.quit()

def main():
    """메인 함수"""
    print("🤖 BalanceBot 2D 시각 시뮬레이터 v1.0")
    print("=" * 60)
    print("사용법: ./balancebot_posix | python3 visual_simulator.py")
    print("Dependencies: pip install pygame")
    print("=" * 60)
    
    try:
        # 시뮬레이터 생성 및 실행
        simulator = VisualSimulator()
        simulator.run()
    except ImportError:
        print("❌ pygame이 설치되지 않았습니다:")
        print("   pip install pygame")
    except Exception as e:
        print(f"❌ 시뮬레이터 오류: {e}")

if __name__ == '__main__':
    main()