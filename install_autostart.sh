#!/usr/bin/env bash
# ==============================================================================
# install_autostart.sh - Arduino UNO Q systemd 자동 실행 서비스 등록 스크립트
#
# 실행 방법:
#   chmod +x auto_start.sh install_autostart.sh
#   sudo ./install_autostart.sh
# ==============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CURRENT_USER="$(logname 2>/dev/null || echo "$USER")"
SERVICE_NAME="smart-goggles.service"
TARGET_SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME"

echo "=== Arduino UNO Q Smart Goggles 자동 실행 서비스 등록 ==="
echo "설치 경로: $SCRIPT_DIR"
echo "실행 계정: $CURRENT_USER"

# root 권한 확인
if [ "$EUID" -ne 0 ]; then
    echo "오류: 이 스크립트는 root 권한(sudo)으로 실행해야 합니다."
    exit 1
fi

# systemd 지원 여부 확인
if ! command -v systemctl >/dev/null 2>&1; then
    echo "경고: 시스템에 systemctl 명령이 없습니다."
    echo "systemd 환경이 아닌 경우, /etc/rc.local 또는 crontab (@reboot $SCRIPT_DIR/auto_start.sh start)을 사용하세요."
    exit 1
fi

# 서비스 파일 템플릿 생성 및 경로 치환
cat <<EOF > "$TARGET_SERVICE_PATH"
[Unit]
Description=Arduino UNO Q Smart Goggles Python Navigation Service
After=network.target arduino-router.service
Wants=arduino-router.service

[Service]
Type=simple
User=$CURRENT_USER
WorkingDirectory=$SCRIPT_DIR
ExecStart=/bin/bash $SCRIPT_DIR/auto_start.sh run
Restart=always
RestartSec=3

StandardOutput=append:$SCRIPT_DIR/logs/smart_goggles.log
StandardError=append:$SCRIPT_DIR/logs/smart_goggles.err.log
Environment=PYTHONUNBUFFERED=1

[Install]
WantedBy=multi-user.target
EOF

chmod 644 "$TARGET_SERVICE_PATH"
chmod +x "$SCRIPT_DIR/auto_start.sh"

echo "systemd 데몬 리로드..."
systemctl daemon-reload

echo "서비스 활성화 (부팅 시 자동 실행 등록)..."
systemctl enable "$SERVICE_NAME"

echo "서비스 시작..."
systemctl restart "$SERVICE_NAME"

echo ""
echo "=== 등록 완료 ==="
echo "서비스 상태 확인: sudo systemctl status $SERVICE_NAME"
echo "실시간 로그 확인: tail -f $SCRIPT_DIR/logs/smart_goggles.log"
echo "서비스 중지:     sudo systemctl stop $SERVICE_NAME"
