"""
config.py - Smart Goggles 설정 파일

Arduino UNO Q Smart Goggles 시스템의 전역 설정값을 관리합니다.
하드웨어 연결 및 팀 규격 미확정 항목은 이곳에서 손쉽게 변경할 수 있습니다.
"""

import os

# ==============================================================================
# 1. 파일 및 주기 설정
# ==============================================================================
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 팀 표준 CSV 파일 경로
CSV_FILE = os.environ.get("GOGGLES_CSV_FILE", os.path.join(BASE_DIR, "team.csv"))

# 센서 수집 및 메인 루프 주기 (초)
LOOP_INTERVAL = float(os.environ.get("GOGGLES_INTERVAL", 0.5))

# 로그 디렉터리 및 파일
LOG_DIR = os.path.join(BASE_DIR, "logs")
LOG_FILE = os.path.join(LOG_DIR, "smart_goggles.log")

# ==============================================================================
# 2. 목표 위치 (Target GPS) 설정
# ==============================================================================
# 기본 목표 좌표 (미설정 시 기본값: 숭실대학교 정보과학관 예시)
# 사용 목적에 맞게 변경하거나 환경변수로 주입 가능
TARGET_LATITUDE = float(os.environ.get("GOGGLES_TARGET_LAT", 37.4963))
TARGET_LONGITUDE = float(os.environ.get("GOGGLES_TARGET_LON", 126.9572))

# ==============================================================================
# 3. GPS 수신 방식 설정 [미확정 사항 대응]
# ==============================================================================
# 지원 모드:
#   - "bridge": MCU 스케치에서 Bridge.call("read_gps")를 통해 전달받는 방식
#   - "serial": Linux의 USB/UART 시리얼 포트에서 직접 NMEA 문장을 수신하는 방식
#   - "mock":   하드웨어 미연결 시 테스트/시뮬레이션용 좌표를 사용하는 방식
#   - "none":   GPS 비활성화
GPS_SOURCE = os.environ.get("GOGGLES_GPS_SOURCE", "bridge")

# Linux Serial 모드 사용 시 설정
GPS_SERIAL_PORT = os.environ.get("GOGGLES_GPS_PORT", "/dev/ttyACM0")
GPS_SERIAL_BAUD = int(os.environ.get("GOGGLES_GPS_BAUD", 9600))

# Mock 모드 사용 시 반환할 시뮬레이션 위치 (lat, lon)
MOCK_GPS_LATITUDE = 37.4950
MOCK_GPS_LONGITUDE = 126.9550

# ==============================================================================
# 4. MCU 방향 초기화 및 기존 호출 호환 설정
# ==============================================================================
# 초기 방위각 (도, 0.0 ~ 359.9):
# MPU-6050은 지자기 센서(나침반)가 없어 절대 방위각을 자체적으로 알 수 없습니다.
# read_nav에 MCU 방향이 있으면 그대로 사용합니다 (초기값 불필요).
# MCU 방향이 없을 때만 set_heading(deg)으로 1회 요청합니다.
# 실제 GPS는 이동 방향으로 초기화하고, 가상 GPS는 스케치의 시작 방향을 사용합니다.
# 수동 설정 시 현재 바라보는 진북 기준 시계방향 각도(0 이상 360 미만)를 입력하세요.
INITIAL_HEADING = None
if os.environ.get("GOGGLES_INITIAL_HEADING"):
    try:
        INITIAL_HEADING = float(os.environ.get("GOGGLES_INITIAL_HEADING"))
    except ValueError as e:
        raise ValueError("GOGGLES_INITIAL_HEADING에는 실제 시작 방위각을 숫자로 입력하세요.") from e

# 아래 자이로 설정은 이전 Python 호출과의 호환용입니다.
# 통합 MCU 펌웨어가 자동 영점 보정/적분을 수행하므로 Python에서는 적용/전송하지 않습니다.
GYRO_Z_BIAS = float(os.environ.get("GOGGLES_GYRO_Z_BIAS", 0.0))

# 기존 설정 호환용 (MCU 영점 측정은 이 환경변수와 관계없이 수행)
AUTO_CALIBRATE_GYRO = os.environ.get("GOGGLES_AUTO_CALIBRATE", "False").lower() in ("true", "1", "yes")
CALIBRATION_SAMPLES = 50

# ==============================================================================
# 5. CSV 컬럼 규격 [표준 team.csv 포맷]
# ==============================================================================
# 필수 6개 컬럼 + IMU 센서 및 실행 상태 포함 하위 호환 표준 포맷
CSV_HEADERS = [
    "timestamp",
    "interval_s",
    "status",
    "latitude",
    "longitude",
    "target_bearing",
    "heading",
    "angle_error",
    "ax",
    "ay",
    "az",
    "gx",
    "gy",
    "gz",
    "temp",
    "pitch",
    "error"
]
