#!/bin/bash
# BalanceBot GUI 시뮬레이터 실행 스크립트

echo "🤖 BalanceBot GUI 시뮬레이터 시작"
echo "=" * 50
echo "📋 사용법:"
echo "  - GUI 창: 실시간 그래프 표시"
echo "  - 터미널: 키보드 조작 (↑↓←→ SPACE s q)"
echo "  - 종료: Ctrl+C 또는 GUI 창 닫기"
echo "=" * 50

cd /home/hyunsu5203/balaencedbot_esp_ide/posix_simulator/build

# GUI 백그라운드로 실행하고 시뮬레이터 연결
./balancebot_posix | python3 ../../tools/simulator_plotter.py

echo "📊 시뮬레이터 종료"