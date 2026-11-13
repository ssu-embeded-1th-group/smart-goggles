#!/usr/bin/env bash
# ==============================================================================
# auto_start.sh - Arduino UNO Q Smart Goggles 자동 실행 및 프로세스 관리 스크립트
#
# 지원 명령어:
#   ./auto_start.sh start   : 백그라운드 데몬으로 실행 (중복 실행 방지)
#   ./auto_start.sh stop    : 실행 중인 프로세스 안전 종료
#   ./auto_start.sh status  : 실행 상태 확인
#   ./auto_start.sh restart : 재시작
#   ./auto_start.sh run     : 포그라운드 실행 (systemd ExecStart용)
# ==============================================================================

# 1. 작업 디렉터리 설정 (스크립트가 위치한 디렉터리로 이동)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR" || { echo "오류: 작업 디렉터리 이동 실패 ($SCRIPT_DIR)"; exit 1; }

PID_FILE="/tmp/smart_goggles.pid"
LOG_DIR="$SCRIPT_DIR/logs"
LOG_FILE="$LOG_DIR/smart_goggles.log"
ERR_LOG="$LOG_DIR/smart_goggles.err.log"

mkdir -p "$LOG_DIR"

# 2. Python 실행 경로 탐색 (가상환경 우선, 없으면 시스템 python3)
if [ -f "$SCRIPT_DIR/venv/bin/python3" ]; then
    PYTHON_BIN="$SCRIPT_DIR/venv/bin/python3"
elif [ -f "$SCRIPT_DIR/.venv/bin/python3" ]; then
    PYTHON_BIN="$SCRIPT_DIR/.venv/bin/python3"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_BIN="$(command -v python3)"
else
    echo "오류: Python3 실행 파일을 찾을 수 없습니다." | tee -a "$ERR_LOG"
    exit 1
fi

# 3. Arduino Router / Bridge 통신 준비 대기 (최대 15초 대기)
wait_for_bridge() {
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Arduino Bridge 데몬 확인 중..." >> "$LOG_FILE"
    local max_retries=15
    local count=0
    # linux 환경에서 arduino-router 프로세스 또는 통신 가능 상태 확인
    while [ $count -lt $max_retries ]; do
        if pgrep -x "arduino-router" >/dev/null 2>&1 || [ ! -d "/proc" ]; then
            echo "[$(date '+%Y-%m-%d %H:%M:%S')] Arduino Router 준비 완료." >> "$LOG_FILE"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] 경고: Arduino Router 응답 대기 시간 초과, 계속 진행합니다." >> "$LOG_FILE"
    return 0
}

# 4. 프로세스 상태 확인 함수
is_running() {
    if [ -f "$PID_FILE" ]; then
        local pid
        pid=$(cat "$PID_FILE")
        if kill -0 "$pid" >/dev/null 2>&1; then
            return 0
        else
            # PID 파일은 있으나 실제 프로세스는 종료된 상태
            rm -f "$PID_FILE"
            return 1
        fi
    fi
    return 1
}

# ==============================================================================
# 동작 명령 처리
# ==============================================================================
ACTION="${1:-run}"

case "$ACTION" in
    start)
        if is_running; then
            PID=$(cat "$PID_FILE")
            echo "Smart Goggles가 이미 실행 중입니다. (PID: $PID)"
            exit 0
        fi

        echo "Smart Goggles 시작 중..."
        wait_for_bridge

        nohup "$PYTHON_BIN" "$SCRIPT_DIR/main.py" >> "$LOG_FILE" 2>> "$ERR_LOG" &
        NEW_PID=$!
        echo "$NEW_PID" > "$PID_FILE"

        sleep 1
        if is_running; then
            echo "성공적으로 시작되었습니다. (PID: $NEW_PID)"
            echo "로그 확인: tail -f $LOG_FILE"
        else
            echo "시작 실패! 에러 로그를 확인하세요: $ERR_LOG"
            cat "$ERR_LOG" | tail -n 10
            exit 1
        fi
        ;;

    stop)
        if is_running; then
            PID=$(cat "$PID_FILE")
            echo "Smart Goggles 종료 중... (PID: $PID)"
            kill "$PID"
            count=0
            while kill -0 "$PID" >/dev/null 2>&1 && [ $count -lt 5 ]; do
                sleep 1
                count=$((count + 1))
            done
            if kill -0 "$PID" >/dev/null 2>&1; then
                echo "강제 종료(SIGKILL)를 수행합니다..."
                kill -9 "$PID"
            fi
            rm -f "$PID_FILE"
            echo "종료 완료."
        else
            echo "실행 중인 Smart Goggles 프로세스가 없습니다."
            rm -f "$PID_FILE"
        fi
        ;;

    status)
        if is_running; then
            PID=$(cat "$PID_FILE")
            echo "Smart Goggles 상태: 실행 중 (PID: $PID)"
            echo "최근 로그 5줄:"
            tail -n 5 "$LOG_FILE" 2>/dev/null || echo "(로그 없음)"
        else
            echo "Smart Goggles 상태: 중지됨"
        fi
        ;;

    restart)
        "$0" stop
        sleep 1
        "$0" start
        ;;

    run)
        # systemd 또는 포그라운드 실행 모드
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Smart Goggles 포그라운드 실행 (Python: $PYTHON_BIN)" | tee -a "$LOG_FILE"
        wait_for_bridge
        exec "$PYTHON_BIN" "$SCRIPT_DIR/main.py"
        ;;

    *)
        echo "사용법: $0 {start|stop|status|restart|run}"
        exit 1
        ;;
esac
