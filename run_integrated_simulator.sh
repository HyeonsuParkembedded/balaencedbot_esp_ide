#!/bin/bash

# BalanceBot 통합 시뮬레이터 실행 스크립트
# 시뮬레이터와 그래프를 하나의 명령으로 실행

echo "🤖 BalanceBot 통합 시뮬레이터"
echo "================================"
echo "시뮬레이터와 실시간 그래프를 통합 실행합니다"
echo ""

# 스크립트 디렉토리로 이동
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

# Python 실행
echo "🚀 통합 시뮬레이터 시작..."
python3 tools/integrated_simulator.py