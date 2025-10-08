#!/usr/bin/env python3
"""
시뮬레이터 상세 디버깅 테스트
"""

import subprocess
import time
import select
import sys
from pathlib import Path

def test_subprocess_detailed():
    """상세한 subprocess 디버깅"""
    sim_path = Path("/home/hyunsu5203/balaencedbot_esp_ide/posix_simulator/build/balancebot_posix")
    
    print(f"🔍 상세 시뮬레이터 디버깅")
    print(f"시뮬레이터 경로: {sim_path}")
    print(f"파일 존재: {sim_path.exists()}")
    
    # 파일 권한 확인
    if sim_path.exists():
        stat = sim_path.stat()
        print(f"파일 크기: {stat.st_size} bytes")
        print(f"실행 권한: {oct(stat.st_mode)[-3:]}")
    
    print("=" * 60)
    
    try:
        # stdout과 stderr 분리
        process = subprocess.Popen(
            [str(sim_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            stdin=subprocess.PIPE,
            text=True,
            bufsize=0
        )
        
        print(f"✅ 프로세스 시작 - PID: {process.pid}")
        
        # 더 긴 시간 동안 모니터링
        lines_read = 0
        start_time = time.time()
        last_output_time = start_time
        
        while time.time() - start_time < 15:  # 15초 동안 테스트
            # 프로세스 상태 확인
            return_code = process.poll()
            if return_code is not None:
                print(f"⚠️  프로세스가 종료됨 - 코드: {return_code}")
                break
            
            # select를 사용해서 출력 대기 (non-blocking)
            ready, _, _ = select.select([process.stdout, process.stderr], [], [], 0.1)
            
            output_received = False
            
            # stdout 읽기
            if process.stdout in ready:
                line = process.stdout.readline()
                if line:
                    lines_read += 1
                    elapsed = time.time() - start_time
                    print(f"[{lines_read:3d}] ({elapsed:5.1f}s) STDOUT: {line.strip()}")
                    last_output_time = time.time()
                    output_received = True
            
            # stderr 읽기  
            if process.stderr in ready:
                line = process.stderr.readline()
                if line:
                    elapsed = time.time() - start_time
                    print(f"      ({elapsed:5.1f}s) STDERR: {line.strip()}")
                    last_output_time = time.time()
                    output_received = True
            
            # 출력이 없으면 잠시 대기
            if not output_received:
                time.sleep(0.01)
            
            # 5초 이상 출력이 없으면 체크
            if time.time() - last_output_time > 5:
                print(f"⚠️  {time.time() - last_output_time:.1f}초 동안 출력 없음")
                break
        
        print(f"\n📊 총 {lines_read}줄 읽음 (실행 시간: {time.time() - start_time:.1f}초)")
        
        # 프로세스가 아직 살아있으면 종료
        if process.poll() is None:
            print("🔄 프로세스 종료 중...")
            process.terminate()
            try:
                process.wait(timeout=3)
                print("✅ 프로세스 정상 종료")
            except subprocess.TimeoutExpired:
                print("⚠️  강제 종료")
                process.kill()
                process.wait()
        
    except Exception as e:
        print(f"❌ 오류: {e}")
        import traceback
        traceback.print_exc()

def test_direct_execution():
    """터미널에서 직접 실행 테스트"""
    print("\n" + "="*60)
    print("🎯 직접 실행 테스트")
    print("="*60)
    
    sim_path = "/home/hyunsu5203/balaencedbot_esp_ide/posix_simulator/build/balancebot_posix"
    
    # file 명령어로 파일 타입 확인
    try:
        result = subprocess.run(['file', sim_path], capture_output=True, text=True)
        print(f"파일 타입: {result.stdout.strip()}")
    except:
        print("file 명령어 실행 실패")
    
    # ldd로 의존성 확인
    try:
        result = subprocess.run(['ldd', sim_path], capture_output=True, text=True)
        print(f"의존성 라이브러리:")
        for line in result.stdout.strip().split('\n')[:5]:  # 처음 5개만
            print(f"  {line}")
        if len(result.stdout.strip().split('\n')) > 5:
            print(f"  ... ({len(result.stdout.strip().split('\n'))-5}개 더)")
    except:
        print("ldd 명령어 실행 실패")

if __name__ == "__main__":
    test_subprocess_detailed()
    test_direct_execution()