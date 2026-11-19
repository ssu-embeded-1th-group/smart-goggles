"""
main.py - Arduino UNO Q Smart Goggles 메인 프로그램

Arduino UNO Q(Linux 환경)에서 실행되는 메인 센서 수집, 방위각 계산,
MCU 방향 수신 및 표준 CSV 자동 저장 프로그램입니다.

Bridge 통신 규약:
  - imu_ready  -> bool   : 센서 I2C 응답 여부
  - read_imu   -> String : "ax,ay,az,gx,gy,gz,temp" (g, °/s, °C)
  - read_pitch -> float  : 가속도 기반 피치 각도 (°)
  - read_heading -> float: MCU의 진북 기준 시계방향 방위각, 미설정 시 -999
  - set_heading(heading, bias_z) -> bool: 현재 자세의 기준 방위각과 바이어스 설정
  - read_gps   -> String : "lat,lon" (확장 대비용)
"""

import os
import sys
import time
import csv
import logging
import math
from datetime import datetime
from typing import Optional, Dict, Any, Tuple

# 프로젝트 모듈 임포트
import config
import navigation

# Arduino app_utils 및 Bridge 로드 (실제 하드웨어 우선, 미지원 환경에서는 Mock 제공)
try:
    from arduino.app_utils import App as _ArduinoApp
    from arduino.app_utils import Bridge as _ArduinoBridge
    HAS_REAL_ARDUINO_ENV = True
except ImportError:
    from bridge_interface import bridge_client, MockApp
    _ArduinoApp = MockApp
    _ArduinoBridge = bridge_client
    HAS_REAL_ARDUINO_ENV = False

# 전역 Bridge 객체 (기존 코드 호환)
Bridge = _ArduinoBridge
App = _ArduinoApp

# 로깅 설정
os.makedirs(config.LOG_DIR, exist_ok=True)
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(config.LOG_FILE, encoding="utf-8")
    ]
)
logger = logging.getLogger("SmartGoggles")


class SmartGogglesController:
    """
    Smart Goggles 상태 및 핵심 로직 제어 클래스
    """

    def __init__(
        self,
        target_lat: float = config.TARGET_LATITUDE,
        target_lon: float = config.TARGET_LONGITUDE,
        initial_heading: Optional[float] = config.INITIAL_HEADING,
        gyro_z_bias: float = config.GYRO_Z_BIAS,
        csv_file: str = config.CSV_FILE,
        interval: float = config.LOOP_INTERVAL,
        gps_source: str = config.GPS_SOURCE,
        bridge_instance: Any = Bridge
    ):
        self.target_lat = target_lat
        self.target_lon = target_lon
        if initial_heading is not None and (
            isinstance(initial_heading, bool)
            or not math.isfinite(initial_heading)
            or not 0.0 <= initial_heading < 360.0
        ):
            raise ValueError("초기 방위각은 진북 기준 0 이상 360 미만의 유한한 값이어야 합니다.")
        if not math.isfinite(gyro_z_bias):
            raise ValueError("자이로 바이어스는 유한한 값이어야 합니다.")
        if not math.isfinite(interval) or interval <= 0.0:
            raise ValueError("수집 주기는 0보다 큰 유한한 값이어야 합니다.")
        self.initial_heading = float(initial_heading) if initial_heading is not None else None
        self.current_heading: Optional[float] = None
        self._heading_established = False
        self.relative_yaw: float = 0.0
        self.gyro_z_bias: float = float(gyro_z_bias)
        self.csv_file = csv_file
        self.interval = interval
        self.gps_source = gps_source
        self.bridge = bridge_instance

        self.last_time: Optional[float] = None
        self._is_running: bool = False

    def read_sensor(self) -> Dict[str, float]:
        """
        Bridge.call을 통해 MPU-6050 센서 데이터를 수신합니다.

        Returns:
            ax, ay, az, gx, gy, gz, temp, pitch를 포함한 딕셔너리

        Raises:
            RuntimeError: MPU-6050 응답 없음 또는 데이터 읽기 실패
            ValueError: 센서 데이터 포맷 오류
        """
        if not self.bridge.call("imu_ready"):
            raise RuntimeError("MPU-6050 응답 없음 (I2C 통신 확인 필요)")

        raw_data = self.bridge.call("read_imu")
        if not raw_data:
            raise RuntimeError("센서 데이터 읽기 실패 (read_imu 빈 응답)")

        parsed_imu = navigation.parse_imu_string(str(raw_data))
        if parsed_imu is None:
            raise ValueError(f"센서 데이터 형식 오류: '{raw_data}'")

        pitch_val = self.bridge.call("read_pitch")
        try:
            if isinstance(pitch_val, bool):
                raise ValueError("bool pitch")
            pitch_float = float(pitch_val)
        except (ValueError, TypeError) as e:
            raise ValueError(f"MCU read_pitch 형식 오류: {pitch_val!r}") from e
        if not math.isfinite(pitch_float) or not -90.0 <= pitch_float <= 90.0:
            raise ValueError(f"MCU read_pitch 실패/범위 오류: {pitch_val!r}")

        parsed_imu["pitch"] = pitch_float
        return parsed_imu

    def read_gps(self) -> Optional[Tuple[float, float]]:
        """
        설정된 소스에 따라 GPS 데이터를 수신합니다.

        [미확정 사항 대응]
        - "bridge": MCU Bridge를 통해 'read_gps' RPC 호출
        - "mock": 개발/테스트용 가상 GPS 좌표
        - "serial": 향후 Linux USB-to-Serial GPS 연결 시 확장
        - "none": GPS 사용 안 함

        Returns:
            (latitude, longitude) 튜플 또는 수신 실패 시 None
        """
        if self.gps_source == "none":
            return None

        if self.gps_source == "mock":
            return (config.MOCK_GPS_LATITUDE, config.MOCK_GPS_LONGITUDE)

        if self.gps_source == "bridge":
            try:
                raw_gps = self.bridge.call("read_gps")
                if not raw_gps:
                    return None
                return navigation.parse_gps_data(raw_gps)
            except Exception as e:
                logger.debug(f"Bridge read_gps 호출 중 예외 (스케치 미등록 가능성): {e}")
                return None

        if self.gps_source == "serial":
            # 실제 보드에 USB GPS가 직결된 경우의 fallback 인터페이스
            return None

        return None

    def validate_data(
        self,
        sensor_data: Optional[Dict[str, float]],
        gps_data: Optional[Tuple[float, float]]
    ) -> Tuple[bool, str]:
        """
        수신된 센서 및 GPS 데이터의 유효성을 검사합니다.

        Returns:
            (is_valid, error_message)
        """
        if sensor_data is None:
            return False, "센서 데이터가 없습니다."

        required_keys = ["ax", "ay", "az", "gx", "gy", "gz", "temp", "pitch"]
        for key in required_keys:
            if key not in sensor_data:
                return False, f"필수 센서 키 누락: {key}"
            val = sensor_data[key]
            if not isinstance(val, (int, float)):
                return False, f"센서 값 타입 오류 ({key}={val})"
            if isinstance(val, bool) or not math.isfinite(val):
                return False, f"센서 값이 유한한 숫자가 아닙니다 ({key}={val})"

        # GPS 데이터는 옵션(실내 또는 위성 미수신 상황 고려)이지만,
        # 존재할 경우 유효 좌표 범위(-90~90, -180~180)인지 확인
        if gps_data is not None:
            lat, lon = gps_data
            if not navigation.validate_gps_coordinates(lat, lon):
                return False, f"비정상 GPS 좌표 범위 (lat={lat}, lon={lon})"

        return True, ""

    def calculate_target_bearing(
        self,
        current_lat: Optional[float],
        current_lon: Optional[float]
    ) -> Optional[float]:
        """
        현재 GPS 좌표와 목표 GPS 좌표를 이용해 목표 방위각(0~360°)을 계산합니다.
        GPS가 없으면 None을 반환합니다.
        """
        return navigation.calculate_target_bearing(
            current_lat, current_lon, self.target_lat, self.target_lon
        )

    def read_heading(self) -> Optional[float]:
        """MCU 방위각만 사용한다. 미설정(-999)과 통신/포맷 오류를 구분한다."""
        try:
            raw_heading = self.bridge.call("read_heading")
        except Exception as e:
            raise RuntimeError(f"MCU read_heading RPC 실패 (스케치 업데이트/연결 확인): {e}") from e
        if isinstance(raw_heading, bool) or raw_heading is None or raw_heading == "":
            raise ValueError("MCU read_heading 응답 없음/형식 오류: 업데이트된 스케치를 업로드하세요.")
        try:
            heading = float(raw_heading)
        except (ValueError, TypeError) as e:
            raise ValueError(f"MCU read_heading 형식 오류: {raw_heading!r}") from e
        if heading == -999.0:
            return None
        if not math.isfinite(heading) or not 0.0 <= heading < 360.0:
            raise ValueError(f"MCU read_heading 범위 오류: {raw_heading!r}")
        return heading

    def update_heading(
        self, gz: Optional[float] = None, dt: Optional[float] = None
    ) -> Tuple[Optional[float], float]:
        """
        MCU에서 방향을 수신한다. 기존 호출 호환용 gz/dt는 적분에 사용하지 않는다.
        relative_yaw는 연속 MCU 방위각 사이의 변화량만 추적하며,
        기준/통신을 잃으면 초기화한다 (표본 사이 회전은 180도 미만이어야 함).
        """
        previous_heading = self.current_heading
        self.current_heading = None
        try:
            heading = self.read_heading()
            # 시작할 때 MCU에 기준이 없다면 사용자가 지정한 방향을 한 번 전달한다.
            # 실행 중 기준이 사라지면 과거의 시작 방향을 재사용하지 않는다.
            if heading is None and not self._heading_established and self.initial_heading is not None:
                # 요청이 적용된 뒤 응답만 유실될 수 있으므로 전송 전에 재시도를 막는다.
                self._heading_established = True
                acknowledged = self.bridge.call("set_heading", self.initial_heading, self.gyro_z_bias)
                if acknowledged is not True:
                    raise RuntimeError("MCU set_heading 기준 방향 설정 실패: 스케치/센서 연결을 확인하세요.")
                heading = self.read_heading()
            if heading is not None:
                self._heading_established = True
                if previous_heading is not None:
                    self.relative_yaw += navigation.normalize_angle_180(heading - previous_heading)
                self.current_heading = heading
            else:
                self.relative_yaw = 0.0
        except Exception:
            self.relative_yaw = 0.0
            raise
        return self.current_heading, self.relative_yaw

    def calculate_angle_error(
        self,
        target_bearing: Optional[float],
        current_heading: Optional[float]
    ) -> Optional[float]:
        """
        목표 방위각과 현재 방향의 차이를 계산합니다 (-180° ~ +180°).
        둘 중 하나라도 None이면 None을 반환합니다.
        """
        return navigation.calculate_angle_error(target_bearing, current_heading)

    def save_csv(self, row_dict: Dict[str, Any]) -> bool:
        """
        팀 표준 규격(CSV_HEADERS)에 맞추어 team.csv에 행을 추가합니다.
        파일이 없으면 헤더를 자동 생성하고, 기존 파일이 있으면 내용을 보존합니다.

        Returns:
            저장 성공 여부 (bool)
        """
        file_path = self.csv_file

        # 저장할 레코드 정리 (None은 빈 문자열로 처리, 소수점 자릿수 정리)
        row_to_write = []
        for header in config.CSV_HEADERS:
            val = row_dict.get(header, "")
            if val is None:
                formatted_val = ""
            elif isinstance(val, float):
                formatted_val = f"{val:.3f}" if header in ("interval_s", "ax", "ay", "az") else (
                    f"{val:.6f}" if header in ("latitude", "longitude") else f"{val:.2f}"
                )
            else:
                formatted_val = str(val)
            row_to_write.append(formatted_val)

        try:
            needs_header = not os.path.exists(file_path) or os.path.getsize(file_path) == 0
            parent_dir = os.path.dirname(file_path)
            if parent_dir:
                os.makedirs(parent_dir, exist_ok=True)

            with open(file_path, "a", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                if needs_header:
                    writer.writerow(config.CSV_HEADERS)
                writer.writerow(row_to_write)
                f.flush()
            return True
        except Exception as e:
            logger.error(f"CSV 저장 실패 ({file_path}): {e}")
            return False

    def step(self) -> Dict[str, Any]:
        """
        1회 주기 동안 실행되는 데이터 수집, 계산 및 저장 단위 루틴
        """
        now = time.monotonic()
        if self.last_time is None:
            interval_str: Any = ""
        else:
            dt = max(0.001, now - self.last_time)
            interval_str = round(dt, 3)
        self.last_time = now

        timestamp = datetime.now().astimezone().isoformat(timespec="milliseconds")

        record: Dict[str, Any] = {
            "timestamp": timestamp,
            "interval_s": interval_str,
            "status": "ok",
            "latitude": "",
            "longitude": "",
            "target_bearing": "",
            "heading": "",
            "angle_error": "",
            "ax": "",
            "ay": "",
            "az": "",
            "gx": "",
            "gy": "",
            "gz": "",
            "temp": "",
            "pitch": "",
            "error": ""
        }

        try:
            # 1. IMU 및 GPS 데이터 수신
            sensor_data = self.read_sensor()
            gps_data = self.read_gps()

            # 2. 데이터 유효성 검사
            is_valid, val_err = self.validate_data(sensor_data, gps_data)
            if not is_valid:
                raise ValueError(val_err)

            # 센서 데이터 매핑
            record.update({
                "ax": sensor_data["ax"],
                "ay": sensor_data["ay"],
                "az": sensor_data["az"],
                "gx": sensor_data["gx"],
                "gy": sensor_data["gy"],
                "gz": sensor_data["gz"],
                "temp": sensor_data["temp"],
                "pitch": sensor_data["pitch"],
            })

            # 3. GPS 방위각 계산 (GPS 유효 시에만)
            if gps_data is not None:
                record["latitude"] = gps_data[0]
                record["longitude"] = gps_data[1]
                t_bearing = self.calculate_target_bearing(gps_data[0], gps_data[1])
                if t_bearing is not None:
                    record["target_bearing"] = t_bearing
            else:
                t_bearing = None

            # 4. MCU의 현재 방위각 수신 (Python에서 자이로를 별도 적분하지 않음)
            c_heading, _ = self.update_heading()
            if c_heading is not None:
                record["heading"] = c_heading

            # 5. 목표 방위각과 현재 방향의 차이 계산
            if t_bearing is not None and c_heading is not None:
                angle_err = self.calculate_angle_error(t_bearing, c_heading)
                if angle_err is not None:
                    record["angle_error"] = angle_err

            if c_heading is None:
                record["status"] = "waiting_heading"
                record["error"] = (
                    "MCU 기준 방향 미설정/소실: 실제 바라보는 방위각을 확인한 뒤 "
                    "GOGGLES_INITIAL_HEADING(0=진북, 90=동, 180=남, 270=서)을 설정하고 "
                    "현재 자세에서 다시 시작하거나 MCU set_heading으로 재설정하세요."
                )
            elif t_bearing is None:
                record["status"] = "waiting_gps"
                record["error"] = "목표 방위각 계산 불가: GPS 수신 및 현재/목표 좌표를 확인하세요."

        except Exception as e:
            self.current_heading = None
            self.relative_yaw = 0.0
            record["status"] = "error"
            record["error"] = str(e)
            logger.warning(f"루프 처리 중 에러 발생: {e}")

        # 6. CSV 저장
        if not self.save_csv(record):
            record["status"] = "error"
            record["error"] = "; ".join(filter(None, (record["error"], "CSV 저장 실패")))
        return record


# ==============================================================================
# 모듈 레벨 권장 함수 인터페이스 (요구사항 4 만족 및 단독 호출용)
# ==============================================================================
_controller = SmartGogglesController()


def read_sensor() -> Dict[str, float]:
    """Bridge를 통한 IMU 데이터 수신 권장 함수"""
    return _controller.read_sensor()


def read_gps() -> Optional[Tuple[float, float]]:
    """GPS 데이터 수신 권장 함수"""
    return _controller.read_gps()


def validate_data(sensor_data: Optional[Dict[str, float]], gps_data: Optional[Tuple[float, float]]) -> Tuple[bool, str]:
    """수신 데이터 유효성 검사 권장 함수"""
    return _controller.validate_data(sensor_data, gps_data)


def calculate_target_bearing(current_lat: Optional[float], current_lon: Optional[float]) -> Optional[float]:
    """현재 좌표에서 목표 좌표까지의 방위각 계산 권장 함수"""
    return _controller.calculate_target_bearing(current_lat, current_lon)


def update_heading(gz: Optional[float] = None, dt: Optional[float] = None) -> Tuple[Optional[float], float]:
    """MCU 방향 수신 권장 함수 (gz/dt는 기존 호출 호환용)."""
    return _controller.update_heading(gz, dt)


def calculate_angle_error(target_bearing: Optional[float], current_heading: Optional[float]) -> Optional[float]:
    """목표 방향과 현재 방향의 차이 계산 권장 함수"""
    return _controller.calculate_angle_error(target_bearing, current_heading)


def save_csv(row_dict: Dict[str, Any]) -> bool:
    """표준 CSV 저장 권장 함수"""
    return _controller.save_csv(row_dict)


def main(max_iterations: Optional[int] = None):
    """
    프로그램 초기화 및 메인 루프 실행
    """
    logger.info("==========================================")
    logger.info("Arduino UNO Q Smart Goggles 서비스 시작")
    logger.info(f"실행 환경: {'Arduino UNO Q Linux (Real)' if HAS_REAL_ARDUINO_ENV else 'PC / Mock Mode'}")
    logger.info(f"CSV 저장 경로: {config.CSV_FILE}")
    logger.info(f"목표 좌표: Lat {config.TARGET_LATITUDE}, Lon {config.TARGET_LONGITUDE}")
    logger.info(f"초기 Heading: {config.INITIAL_HEADING} (MCU 기준 미설정 시에만 전달)")
    if config.INITIAL_HEADING is None:
        logger.info("MCU 방향을 우선 사용합니다. 기준이 없으면 GOGGLES_INITIAL_HEADING에 실제 시작 방위각을 설정하세요.")
    logger.info(f"GPS 모드: {config.GPS_SOURCE}")
    logger.info("==========================================")

    last_status: Optional[str] = None

    def loop():
        nonlocal last_status
        record = _controller.step()
        if record["status"].startswith("waiting_") and record["status"] != last_status:
            logger.warning("%s: %s", record["status"], record["error"])
        last_status = record["status"]
        time.sleep(config.LOOP_INTERVAL)

    if HAS_REAL_ARDUINO_ENV and _ArduinoApp is not None:
        _ArduinoApp.run(user_loop=loop)
    else:
        # 가상 환경 또는 단독 테스트 실행
        iterations = 0
        try:
            while True:
                loop()
                iterations += 1
                if max_iterations is not None and iterations >= max_iterations:
                    break
        except KeyboardInterrupt:
            logger.info("프로그램이 사용자에 의해 중단되었습니다.")


if __name__ == "__main__":
    main()
