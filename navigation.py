"""
navigation.py - 항법 및 센서 융합 수학 알고리즘 모듈

방위각 계산, 자이로 적분을 통한 방향 추정, 각도 오차 계산 및 데이터 파싱을 담당합니다.
외부 의존성이 없는 순수 수학 함수들로 구성되어 독립적인 단위 테스트가 가능합니다.
"""

import math
from typing import Optional, Tuple, Dict, Any


def normalize_angle_360(angle: float) -> float:
    """각도를 0.0 이상 360.0 미만 범위로 정규화합니다."""
    return angle % 360.0


def normalize_angle_180(angle: float) -> float:
    """각도를 -180.0 이상 +180.0 이하 범위로 정규화합니다."""
    return (angle + 180.0) % 360.0 - 180.0


def validate_gps_coordinates(lat: Any, lon: Any) -> bool:
    """
    위도와 경도의 유효성을 검사합니다.
    - 위도: -90.0 ~ +90.0
    - 경도: -180.0 ~ +180.0
    """
    if lat is None or lon is None:
        return False
    try:
        f_lat = float(lat)
        f_lon = float(lon)
        if math.isnan(f_lat) or math.isnan(f_lon) or math.isinf(f_lat) or math.isinf(f_lon):
            return False
        return (-90.0 <= f_lat <= 90.0) and (-180.0 <= f_lon <= 180.0)
    except (ValueError, TypeError):
        return False


def calculate_target_bearing(
    current_lat: Optional[float],
    current_lon: Optional[float],
    target_lat: Optional[float],
    target_lon: Optional[float]
) -> Optional[float]:
    """
    현재 GPS 위치에서 목표 GPS 위치까지의 목표 방위각(Target Bearing)을 계산합니다.
    구면 삼각법(Great-Circle Forward Azimuth) 공식을 사용합니다.

    Returns:
        0.0 ~ 359.99... 도(Degree) 단위의 방위각 (진북 기준 0° 시계방향),
        좌표가 유효하지 않거나 두 지점이 동일하면 None 반환.
    """
    if not (validate_gps_coordinates(current_lat, current_lon) and
            validate_gps_coordinates(target_lat, target_lon)):
        return None

    c_lat = float(current_lat)  # type: ignore
    c_lon = float(current_lon)  # type: ignore
    t_lat = float(target_lat)  # type: ignore
    t_lon = float(target_lon)  # type: ignore

    # 동일 지점일 경우 방위각 계산 불가
    if math.isclose(c_lat, t_lat, abs_tol=1e-7) and math.isclose(c_lon, t_lon, abs_tol=1e-7):
        return None

    phi1 = math.radians(c_lat)
    phi2 = math.radians(t_lat)
    delta_lambda = math.radians(t_lon - c_lon)

    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - (
        math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    )

    bearing_rad = math.atan2(y, x)
    bearing_deg = math.degrees(bearing_rad)
    return normalize_angle_360(bearing_deg)


def calculate_angle_error(
    target_bearing: Optional[float],
    current_heading: Optional[float]
) -> Optional[float]:
    """
    목표 방위각과 현재 방향(Heading)의 차이(Angle Error)를 계산합니다.

    Returns:
        -180.0 ~ +180.0 범위의 오차 각도 (도 단위).
        - 양수 (+): 목표가 현재 진행 방향의 오른쪽 (우회전 필요)
        - 음수 (-): 목표가 현재 진행 방향의 왼쪽 (좌회전 필요)
        - 0.0: 목표를 정확히 향하고 있음
        둘 중 하나라도 None이면 None 반환.
    """
    if target_bearing is None or current_heading is None:
        return None

    try:
        t_b = float(target_bearing)
        c_h = float(current_heading)
        if math.isnan(t_b) or math.isnan(c_h) or math.isinf(t_b) or math.isinf(c_h):
            return None
    except (ValueError, TypeError):
        return None

    diff = t_b - c_h
    return normalize_angle_180(diff)


def update_heading(
    current_heading: Optional[float],
    gz_deg_per_sec: float,
    dt_seconds: float,
    bias_z: float = 0.0,
    relative_yaw: float = 0.0
) -> Tuple[Optional[float], float, float]:
    """
    MPU-6050의 Z축 각속도(gz, deg/s)를 시간에 따라 적분하여 방향을 갱신합니다.

    Args:
        current_heading: 현재 절대 방위각 (도, 0~360). 초기 미설정 시 None.
        gz_deg_per_sec: MPU-6050에서 읽어온 Z축 각속도 (°/s).
        dt_seconds: 직전 측정 이후 경과 시간 (초).
        bias_z: 자이로 Z축 바이어스(오프셋) (°/s).
        relative_yaw: 시작 시점 대비 누적 상대 회전 각도 (도).

    Returns:
        (new_heading, new_relative_yaw, delta_yaw)
        - new_heading: 갱신된 절대 방위각 (초기 heading이 None이면 여전히 None)
        - new_relative_yaw: 시작 시점 기준 누적 회전 각도 (도)
        - delta_yaw: 이번 주기 동안의 회전 각도 (도)
    """
    if dt_seconds <= 0.0 or math.isnan(gz_deg_per_sec):
        return current_heading, relative_yaw, 0.0

    # 바이어스 보정된 유효 각속도 (°/s)
    effective_gz = gz_deg_per_sec - bias_z

    # 회전량 적분: Δθ = ω * Δt
    # (주의: MPU-6050 Z축이 상방을 향할 때 반시계 회전이 양수이면,
    # 진북 시계방향 방위각 체계에서는 부호가 반대일 수 있습니다.
    # 기본적으로 표준 오른손 법칙 z축 기준 회전량을 반영합니다.)
    delta_yaw = effective_gz * dt_seconds

    # 누적 상대 회전량 갱신 (-180 ~ +180 범위로 유지 또는 지속 누적)
    new_relative_yaw = relative_yaw + delta_yaw

    # 절대 방위각 갱신
    new_heading: Optional[float] = None
    if current_heading is not None:
        try:
            val = float(current_heading) + delta_yaw
            new_heading = normalize_angle_360(val)
        except (ValueError, TypeError):
            new_heading = None

    return new_heading, new_relative_yaw, delta_yaw


def parse_imu_string(data_str: str) -> Optional[Dict[str, float]]:
    """
    sketch.ino에서 반환하는 'ax,ay,az,gx,gy,gz,temp' 형식의 문자열을 파싱합니다.

    Returns:
        각 센서값 딕셔너리 또는 형식 불일치 시 None.
    """
    if not data_str or not isinstance(data_str, str):
        return None

    parts = [p.strip() for p in data_str.split(",")]
    if len(parts) != 7:
        return None

    try:
        values = [float(p) for p in parts]
        return {
            "ax": values[0],
            "ay": values[1],
            "az": values[2],
            "gx": values[3],
            "gy": values[4],
            "gz": values[5],
            "temp": values[6]
        }
    except (ValueError, TypeError):
        return None


def parse_nmea_sentence(sentence: str) -> Optional[Tuple[float, float]]:
    """
    간단한 NMEA $GPRMC / $GNRMC 또는 $GPGGA / $GNGGA 문장에서 위도, 경도를 추출합니다.
    """
    if not sentence or not sentence.startswith("$"):
        return None

    parts = sentence.split(",")
    header = parts[0].upper()

    try:
        if header.endswith("RMC"):
            # Format: $GPRMC,hhmmss,status,lat,NS,lon,EW,...
            if len(parts) < 7 or parts[2] != "A":  # 'A' = Data valid
                return None
            raw_lat, ns, raw_lon, ew = parts[3], parts[4], parts[5], parts[6]
        elif header.endswith("GGA"):
            # Format: $GPGGA,hhmmss,lat,NS,lon,EW,quality,...
            if len(parts) < 6 or parts[6] in ("0", ""):  # 0 = Fix not available
                return None
            raw_lat, ns, raw_lon, ew = parts[2], parts[3], parts[4], parts[5]
        else:
            return None

        # NMEA lat format: DDMM.MMMM -> decimal degrees
        lat_deg = float(raw_lat[:2]) + float(raw_lat[2:]) / 60.0
        if ns == "S":
            lat_deg = -lat_deg

        # NMEA lon format: DDDMM.MMMM -> decimal degrees
        lon_deg = float(raw_lon[:3]) + float(raw_lon[3:]) / 60.0
        if ew == "W":
            lon_deg = -lon_deg

        if validate_gps_coordinates(lat_deg, lon_deg):
            return lat_deg, lon_deg
    except (ValueError, IndexError, TypeError):
        return None

    return None


def parse_gps_data(raw_data: Any) -> Optional[Tuple[float, float]]:
    """
    다양한 형식의 GPS 원시 데이터를 파싱하여 (latitude, longitude) 튜플로 반환합니다.
    지원 형식:
      - (lat, lon) 튜플 또는 리스트
      - {'lat': ..., 'lon': ...} 또는 {'latitude': ..., 'longitude': ...} 딕셔너리
      - 'lat,lon' 형태의 콤마 구분 문자열 (예: '37.4963,126.9572')
      - NMEA 문장 (예: '$GNRMC,...')
    """
    if raw_data is None:
        return None

    if isinstance(raw_data, (tuple, list)):
        if len(raw_data) >= 2 and validate_gps_coordinates(raw_data[0], raw_data[1]):
            return float(raw_data[0]), float(raw_data[1])
        return None

    if isinstance(raw_data, dict):
        lat = raw_data.get("lat") or raw_data.get("latitude")
        lon = raw_data.get("lon") or raw_data.get("longitude")
        if validate_gps_coordinates(lat, lon):
            return float(lat), float(lon)
        return None

    if isinstance(raw_data, str):
        trimmed = raw_data.strip()
        if trimmed.startswith("$"):
            return parse_nmea_sentence(trimmed)

        parts = trimmed.split(",")
        if len(parts) >= 2:
            try:
                lat = float(parts[0].strip())
                lon = float(parts[1].strip())
                if validate_gps_coordinates(lat, lon):
                    return lat, lon
            except ValueError:
                return None

    return None
