#!/usr/bin/env python3
"""
간단한 데이터 파싱 테스트
"""

import subprocess
import time
import re

def parse_simulator_data(line):
    """시뮬레이터 데이터 파싱 테스트"""
    current_time = time.time()
    
    try:
        # Balance 데이터 파싱
        if "[DEBUG][POSIX_SIM] Balance:" in line:
            # [DEBUG][POSIX_SIM] Balance: Angle=0.00°, Output=0.08
            parts = line.split("Balance: ")[1]  # "Angle=0.00°, Output=0.08"
            angle_part, output_part = parts.split(", ")
            
            angle_str = angle_part.split("=")[1].replace("°", "")  # "0.00"
            output_str = output_part.split("=")[1]  # "0.08"
            
            angle_val = float(angle_str)
            pid_val = float(output_str)
            
            return f"Balance -> Angle: {angle_val:.2f}°, PID: {pid_val:.2f}"
            
        elif "[PHYSICS]" in line and "Vel=" in line:
            # [PHYSICS] Pos=-0.00m, Vel=0.00m/s, Angle=0.0°, Target=0.00m/s
            parts = line.split("[PHYSICS] ")[1]
            data_parts = parts.split(", ")
            
            pos_str = data_parts[0].split("=")[1].replace("m", "")
            vel_str = data_parts[1].split("=")[1].replace("m/s", "")
            angle_str = data_parts[2].split("=")[1].replace("°", "")
            
            pos_val = float(pos_str)
            vel_val = float(vel_str)
            angle_val = float(angle_str)
            
            return f"Physics -> Pos: {pos_val:.2f}m, Vel: {vel_val:.2f}m/s, Angle: {angle_val:.2f}°"
            
    except Exception as e:
        return f"파싱 오류: {str(e)} | 원본: {line[:50]}"
        
    return None

def main():
    """메인 테스트 함수"""
    print("🧪 시뮬레이터 데이터 파싱 테스트")
    print("=" * 50)
    
    # 시뮬레이터 시작
    sim_process = subprocess.Popen(
        ["./posix_simulator/build/balancebot_posix"],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        stdin=subprocess.PIPE,
        text=True,
        bufsize=1
    )
    
    start_time = time.time()
    count = 0
    
    try:
        while time.time() - start_time < 10:  # 10초 테스트
            line = sim_process.stdout.readline()
            if not line:
                break
                
            line = line.strip()
            if line:
                parsed = parse_simulator_data(line)
                if parsed:
                    count += 1
                    print(f"[{count:3d}] {parsed}")
                    
                    if count >= 20:  # 20개 샘플만 표시
                        break
                        
    except KeyboardInterrupt:
        print("\n⌨️  중단됨")
    finally:
        sim_process.terminate()
        sim_process.wait(timeout=3)
        print(f"\n✅ 테스트 완료 - 총 {count}개 데이터 파싱")

if __name__ == "__main__":
    main()