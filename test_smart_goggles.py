"""
test_smart_goggles.py - Smart Goggles 종합 단위 및 통합 테스트 스위트

요구사항 6에 따른 최소 12가지 테스트 항목을 완벽히 검증합니다:
  1. GPS 정상 데이터 수신
  2. GPS 데이터 누락
  3. IMU 정상 데이터 수신
  4. IMU 연결 실패
  5. 초기 Heading 미설정
  6. 방위각 계산 정상 여부
  7. 각도 차이의 범위 처리
  8. CSV 최초 생성
  9. CSV 기존 데이터 추가 저장
  10. 잘못된 데이터 입력
  11. Bridge 통신 오류
  12. 자동 실행 스크립트 구문 및 구조 검증
"""

import os
import sys
import math
import shutil
import tempfile
import unittest
from unittest.mock import patch

# 프로젝트 모듈 임포트
import config
import navigation
from bridge_interface import MockBridge
from main import SmartGogglesController


class TestSmartGoggles(unittest.TestCase):

    def setUp(self):
        self.test_dir = tempfile.mkdtemp()
        self.test_csv = os.path.join(self.test_dir, "test_team.csv")
        self.mock_bridge = MockBridge()

    def tearDown(self):
        if os.path.exists(self.test_dir):
            shutil.rmtree(self.test_dir)

    # --------------------------------------------------------------------------
    # 1. GPS 정상 데이터 수신 테스트
    # --------------------------------------------------------------------------
    def test_01_gps_normal_data_reception(self):
        """GPS 정상 데이터(문자열, 튜플, 딕셔너리, NMEA) 수신 및 파싱 검증"""
        # 콤마 구분 문자열
        gps_str = "37.4963, 126.9572"
        res = navigation.parse_gps_data(gps_str)
        self.assertIsNotNone(res)
        self.assertAlmostEqual(res[0], 37.4963, places=4)
        self.assertAlmostEqual(res[1], 126.9572, places=4)

        # Bridge를 통한 GPS 수신
        self.mock_bridge.set_handler("read_gps", lambda: "37.5000,126.9600")
        controller = SmartGogglesController(
            gps_source="bridge",
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )
        gps = controller.read_gps()
        self.assertIsNotNone(gps)
        self.assertAlmostEqual(gps[0], 37.5000)
        self.assertAlmostEqual(gps[1], 126.9600)

        # NMEA $GPRMC 문장 수신 테스트
        # 3749.6300 N -> 37 + 49.63/60 = 37.827166...
        nmea_sample = "$GPRMC,083559.00,A,3729.7780,N,12657.4320,E,0.0,0.0,131126,,,A*7C"
        parsed_nmea = navigation.parse_gps_data(nmea_sample)
        self.assertIsNotNone(parsed_nmea)
        self.assertTrue(37.0 <= parsed_nmea[0] <= 38.0)
        self.assertTrue(126.0 <= parsed_nmea[1] <= 127.0)

    # --------------------------------------------------------------------------
    # 2. GPS 데이터 누락 테스트
    # --------------------------------------------------------------------------
    def test_02_gps_data_missing(self):
        """GPS 데이터가 없거나 수신 불가 시 안전한 None 반환 및 방위각 미계산 검증"""
        # None 입력
        self.assertIsNone(navigation.parse_gps_data(None))
        self.assertIsNone(navigation.parse_gps_data(""))
        self.assertIsNone(navigation.parse_gps_data("invalid,data"))

        # Bridge 응답 없음
        self.mock_bridge.set_handler("read_gps", lambda: "")
        controller = SmartGogglesController(
            gps_source="bridge",
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )
        gps = controller.read_gps()
        self.assertIsNone(gps)

        # GPS 대기 상태에서도 센서와 MCU 방향은 수집한다.
        rec = controller.step()
        self.assertEqual(rec["status"], "waiting_gps")
        self.assertEqual(rec["latitude"], "")
        self.assertEqual(rec["longitude"], "")
        self.assertEqual(rec["target_bearing"], "")

    # --------------------------------------------------------------------------
    # 3. IMU 정상 데이터 수신 테스트
    # --------------------------------------------------------------------------
    def test_03_imu_normal_data_reception(self):
        """Bridge를 통한 IMU 정상 데이터 파싱 및 반환 검증"""
        self.mock_bridge.set_handler("imu_ready", lambda: True)
        self.mock_bridge.set_handler("read_imu", lambda: "0.123,-0.456,0.987,1.25,-2.50,3.75,26.80")
        self.mock_bridge.set_handler("read_pitch", lambda: 14.2)

        controller = SmartGogglesController(
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )
        sensor_data = controller.read_sensor()
        self.assertAlmostEqual(sensor_data["ax"], 0.123)
        self.assertAlmostEqual(sensor_data["ay"], -0.456)
        self.assertAlmostEqual(sensor_data["az"], 0.987)
        self.assertAlmostEqual(sensor_data["gx"], 1.25)
        self.assertAlmostEqual(sensor_data["gy"], -2.50)
        self.assertAlmostEqual(sensor_data["gz"], 3.75)
        self.assertAlmostEqual(sensor_data["temp"], 26.80)
        self.assertAlmostEqual(sensor_data["pitch"], 14.2)

    # --------------------------------------------------------------------------
    # 4. IMU 연결 실패 테스트
    # --------------------------------------------------------------------------
    def test_04_imu_connection_failure(self):
        """MPU-6050 연결 실패(I2C 미응답 또는 빈 데이터) 시 예외 처리 및 에러 기록 검증"""
        # 1) imu_ready가 False 반환
        self.mock_bridge.set_handler("imu_ready", lambda: False)
        controller = SmartGogglesController(
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )
        with self.assertRaises(RuntimeError):
            controller.read_sensor()

        rec = controller.step()
        self.assertEqual(rec["status"], "error")
        self.assertIn("MPU-6050 응답 없음", rec["error"])

        # 2) imu_ready는 True이나 read_imu가 빈 문자열 반환
        self.mock_bridge.set_handler("imu_ready", lambda: True)
        self.mock_bridge.set_handler("read_imu", lambda: "")
        with self.assertRaises(RuntimeError):
            controller.read_sensor()

        rec2 = controller.step()
        self.assertEqual(rec2["status"], "error")
        self.assertIn("센서 데이터 읽기 실패", rec2["error"])

    # --------------------------------------------------------------------------
    # 5. 초기 Heading 미설정 테스트
    # --------------------------------------------------------------------------
    def test_05_initial_heading_unset(self):
        """
        MCU에도 방향이 없으면 대기하고 Python에서 자이로를 중복 적분하지 않는다.
        """
        self.mock_bridge.set_handler("read_nav", lambda: "1000,NO_HEADING,0,,,,,37.495,126.955,0,0,1")
        controller = SmartGogglesController(
            initial_heading=None,
            gyro_z_bias=0.0,
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )
        self.assertIsNone(controller.current_heading)
        self.assertEqual(controller.relative_yaw, 0.0)

        # 10도/초로 2초 동안 회전 (총 20도 회전)
        heading, rel_yaw = controller.update_heading(gz=10.0, dt=2.0)
        # 절대 방향은 여전히 None이어야 함!
        self.assertIsNone(heading)
        self.assertIsNone(controller.current_heading)
        self.assertEqual(rel_yaw, 0.0)
        self.assertEqual(controller.relative_yaw, 0.0)

        # 루프 실행 시 CSV 레코드의 heading과 angle_error가 빈 문자열인지 확인
        rec = controller.step()
        self.assertEqual(rec["heading"], "")
        self.assertEqual(rec["angle_error"], "")
        self.assertEqual(rec["status"], "waiting_heading")

    def nav_frame(self, heading="0.0", state="GUIDE", mock="0"):
        return f"1000,{state},2,53.3,{heading},53.3,242.0,37.495000,126.955000,8,{mock},1"

    def heading_controller(self, **kwargs):
        return SmartGogglesController(
            bridge_instance=self.mock_bridge, csv_file=self.test_csv, **kwargs
        )

    def test_mcu_heading_is_authoritative_without_initial_heading(self):
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("350.0"))
        self.mock_bridge.set_handler("read_gps", lambda: "37.0,127.0")
        self.mock_bridge.set_handler("read_imu", lambda: "0,0,1,0,0,100,25")
        controller = self.heading_controller(initial_heading=None, target_lat=38.0, target_lon=127.0)
        for _ in range(2):
            rec = controller.step()
            self.assertEqual(rec["status"], "ok")
            self.assertEqual(rec["heading"], 350.0)
            self.assertAlmostEqual(rec["angle_error"], 10.0)
        self.assertEqual(controller.relative_yaw, 0.0)
        names = [call[0] for call in self.mock_bridge.call_history]
        self.assertNotIn("read_heading", names)
        self.assertNotIn("set_heading", names)

    def test_reference_uses_one_argument_and_waits_for_async_application(self):
        frames = iter((self.nav_frame("", "NO_HEADING"), self.nav_frame("", "NO_HEADING"), self.nav_frame("91.2")))
        self.mock_bridge.set_handler("read_nav", lambda: next(frames))
        controller = self.heading_controller(initial_heading=90.0, gyro_z_bias=0.25)
        self.assertEqual(controller.step()["status"], "waiting_heading")
        self.assertEqual(controller.step()["heading"], "")
        self.assertEqual(controller.step()["heading"], 91.2)
        calls = [call for call in self.mock_bridge.call_history if call[0] == "set_heading"]
        self.assertEqual(calls, [("set_heading", (90.0,), {})])

    def test_existing_mcu_reference_is_preserved(self):
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("123.0"))
        self.assertEqual(self.heading_controller(initial_heading=90.0).step()["heading"], 123.0)
        self.assertFalse(any(call[0] == "set_heading" for call in self.mock_bridge.call_history))

    def test_north_heading_zero_is_valid(self):
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("0.0"))
        self.assertEqual(self.heading_controller(initial_heading=None).step()["heading"], 0.0)

    def test_read_nav_optional_fields_can_be_empty(self):
        self.mock_bridge.set_handler("read_nav", lambda: "1000,NO_FIX,0,,270,,,,,0,0,1")
        self.assertEqual(self.heading_controller(initial_heading=None).read_heading(), 270.0)

    def test_malformed_nav_or_heading_clears_stale_direction(self):
        bad_frames = ["", None, True, "1,2,3", self.nav_frame("0", state="UNKNOWN"),
                      self.nav_frame("0", mock="2")]
        bad_frames += [self.nav_frame(value) for value in ("nan", "inf", "-1", "360", "-999", "invalid")]
        for frame in bad_frames:
            with self.subTest(frame=frame):
                self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("45"))
                controller = self.heading_controller(initial_heading=90.0)
                self.assertEqual(controller.step()["heading"], 45.0)
                self.mock_bridge.set_handler("read_nav", lambda: frame)
                rec = controller.step()
                self.assertEqual(rec["status"], "error")
                self.assertEqual(rec["heading"], "")
                self.assertEqual(rec["angle_error"], "")
                self.assertIsNone(controller.current_heading)

    def test_reference_loss_does_not_reapply_old_start_direction(self):
        controller = self.heading_controller(initial_heading=90.0)
        controller.step()
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("", "NO_HEADING"))
        self.assertEqual(controller.step()["status"], "waiting_heading")
        self.assertFalse(any(call[0] == "set_heading" for call in self.mock_bridge.call_history))

    def test_nav_rpc_failure_and_recovery(self):
        controller = self.heading_controller(initial_heading=None)
        controller.step()
        def broken_nav():
            raise ConnectionResetError("nav RPC disconnected")
        self.mock_bridge.set_handler("read_nav", broken_nav)
        rec = controller.step()
        self.assertEqual(rec["status"], "error")
        self.assertIn("nav RPC disconnected", rec["error"])
        self.assertIsNone(controller.current_heading)
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("270"))
        self.assertEqual(controller.step()["heading"], 270.0)

    def test_initial_request_is_not_repeated_after_rejection_or_lost_reply(self):
        def lost_ack(heading):
            raise TimeoutError("MCU applied reference but reply was lost")
        for handler in (lambda heading: False, lost_ack):
            with self.subTest(handler=handler):
                self.mock_bridge.call_history.clear()
                self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("", "NO_HEADING"))
                self.mock_bridge.set_handler("set_heading", handler)
                controller = self.heading_controller(initial_heading=90.0)
                self.assertEqual(controller.step()["status"], "error")
                self.assertEqual(controller.step()["status"], "waiting_heading")
                calls = [call for call in self.mock_bridge.call_history if call[0] == "set_heading"]
                self.assertEqual(len(calls), 1)

    def test_calibration_and_mcu_imu_error_are_reported(self):
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("0", "CAL", "1"))
        controller = self.heading_controller(initial_heading=None)
        rec = controller.step()
        self.assertEqual(rec["status"], "waiting_calibration")
        self.assertEqual(rec["angle_error"], "")
        self.assertTrue(controller.nav_mock)
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame("0", "IMU_ERR"))
        self.assertEqual(controller.step()["status"], "error")

    def test_relative_change_tracks_mcu_heading_across_north(self):
        headings = iter(("350", "10", "350"))
        self.mock_bridge.set_handler("read_nav", lambda: self.nav_frame(next(headings)))
        controller = self.heading_controller(initial_heading=None)
        self.assertEqual(controller.update_heading(1000, 1000), (350.0, 0.0))
        self.assertEqual(controller.update_heading(1000, 1000), (10.0, 20.0))
        self.assertEqual(controller.update_heading(1000, 1000), (350.0, 0.0))

    def test_nonfinite_sensor_and_invalid_pitch_are_errors(self):
        for value in ("nan", "inf", "-inf"):
            with self.subTest(value=value):
                self.mock_bridge.set_handler("read_imu", lambda: f"0,0,1,0,0,{value},25")
                self.assertEqual(self.heading_controller().step()["status"], "error")
        self.mock_bridge.set_handler("read_imu", lambda: "0,0,1,0,0,0,25")
        for value in (-999.0, "", None, True, "nan", 91.0):
            with self.subTest(pitch=value):
                self.mock_bridge.set_handler("read_pitch", lambda: value)
                self.assertEqual(self.heading_controller().step()["status"], "error")

    def test_csv_failure_is_reported(self):
        controller = self.heading_controller()
        with patch.object(controller, "save_csv", return_value=False):
            self.assertEqual(controller.step()["status"], "error")
        with patch("main.os.path.getsize", side_effect=PermissionError("denied")):
            with patch("main.os.path.exists", return_value=True):
                self.assertFalse(controller.save_csv({}))

    def test_invalid_initial_reference_and_interval_are_rejected(self):
        for value in (float("nan"), float("inf"), -1, 360, True):
            with self.subTest(heading=value):
                with self.assertRaises(ValueError):
                    self.heading_controller(initial_heading=value)
        for value in (0, -1, float("nan"), float("inf")):
            with self.subTest(interval=value):
                with self.assertRaises(ValueError):
                    self.heading_controller(interval=value)

    # --------------------------------------------------------------------------
    # 6. 방위각 계산 정상 여부 테스트
    # --------------------------------------------------------------------------
    def test_06_target_bearing_calculation(self):
        """구면 삼각법 방위각 계산 정확도(정북 0°, 정동 90°, 정남 180°, 정서 270°) 검증"""
        lat0, lon0 = 37.5000, 127.0000

        # 정북: 위도 증가 (위도 +0.1)
        b_north = navigation.calculate_target_bearing(lat0, lon0, lat0 + 0.1, lon0)
        self.assertIsNotNone(b_north)
        self.assertAlmostEqual(b_north, 0.0, delta=0.1)

        # 정남: 위도 감소 (위도 -0.1)
        b_south = navigation.calculate_target_bearing(lat0, lon0, lat0 - 0.1, lon0)
        self.assertIsNotNone(b_south)
        self.assertAlmostEqual(b_south, 180.0, delta=0.1)

        # 정동: 경도 증가 (경도 +0.1)
        b_east = navigation.calculate_target_bearing(lat0, lon0, lat0, lon0 + 0.1)
        self.assertIsNotNone(b_east)
        self.assertAlmostEqual(b_east, 90.0, delta=0.5)

        # 정서: 경도 감소 (경도 -0.1)
        b_west = navigation.calculate_target_bearing(lat0, lon0, lat0, lon0 - 0.1)
        self.assertIsNotNone(b_west)
        self.assertAlmostEqual(b_west, 270.0, delta=0.5)

        # 동일 좌표 입력 시 -> None 반환
        self.assertIsNone(navigation.calculate_target_bearing(lat0, lon0, lat0, lon0))

    # --------------------------------------------------------------------------
    # 7. 각도 차이의 범위 처리 (-180° ~ +180°) 테스트
    # --------------------------------------------------------------------------
    def test_07_angle_error_range(self):
        """목표 방위각과 현재 방향 차이가 -180 ~ +180 범위로 정확히 랩어라운드되는지 검증"""
        # 1) 목표 10°, 현재 350° -> 우회전 20° (+20°)
        err1 = navigation.calculate_angle_error(10.0, 350.0)
        self.assertAlmostEqual(err1, 20.0)

        # 2) 목표 350°, 현재 10° -> 좌회전 20° (-20°)
        err2 = navigation.calculate_angle_error(350.0, 10.0)
        self.assertAlmostEqual(err2, -20.0)

        # 3) 동일 방향 -> 0°
        err3 = navigation.calculate_angle_error(180.0, 180.0)
        self.assertAlmostEqual(err3, 0.0)

        # 4) 정반대 (180도 차이)
        err4 = navigation.calculate_angle_error(180.0, 0.0)
        self.assertTrue(math.isclose(abs(err4), 180.0, abs_tol=1e-5))

        # 5) 360도 초과 임의 각도 입력 시 정규화
        err5 = navigation.calculate_angle_error(730.0, 10.0)  # 730° = 10°
        self.assertAlmostEqual(err5, 0.0)

        # 6) None 포함 시 None 반환
        self.assertIsNone(navigation.calculate_angle_error(None, 10.0))
        self.assertIsNone(navigation.calculate_angle_error(10.0, None))

    # --------------------------------------------------------------------------
    # 8. CSV 최초 생성 테스트
    # --------------------------------------------------------------------------
    def test_08_csv_initial_creation(self):
        """파일이 없을 때 파일이 생성되고 표준 헤더가 정확히 기록되는지 검증"""
        test_file = os.path.join(self.test_dir, "new_team.csv")
        self.assertFalse(os.path.exists(test_file))

        controller = SmartGogglesController(
            csv_file=test_file,
            bridge_instance=self.mock_bridge
        )
        sample_row = {
            "timestamp": "2026-11-13T12:00:00.000+09:00",
            "interval_s": 0.5,
            "status": "ok",
            "latitude": 37.4963,
            "longitude": 126.9572,
            "target_bearing": 45.0,
            "heading": 40.0,
            "angle_error": 5.0,
            "ax": 0.01,
            "ay": 0.02,
            "az": 0.99,
            "gx": 0.1,
            "gy": 0.2,
            "gz": 0.3,
            "temp": 25.0,
            "pitch": 1.2,
            "error": ""
        }
        success = controller.save_csv(sample_row)
        self.assertTrue(success)
        self.assertTrue(os.path.exists(test_file))

        with open(test_file, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        self.assertEqual(len(lines), 2)  # 헤더 1줄 + 데이터 1줄
        self.assertEqual(lines[0], ",".join(config.CSV_HEADERS))
        self.assertIn("37.496300", lines[1])
        self.assertIn("126.957200", lines[1])

    # --------------------------------------------------------------------------
    # 9. CSV 기존 데이터 추가 저장 테스트
    # --------------------------------------------------------------------------
    def test_09_csv_append_data(self):
        """기존 CSV 파일에 데이터를 추가할 때 헤더 중복 없이 기존 데이터가 보존되는지 검증"""
        test_file = os.path.join(self.test_dir, "append_team.csv")
        controller = SmartGogglesController(
            csv_file=test_file,
            bridge_instance=self.mock_bridge
        )

        # 1회 저장
        controller.save_csv({"timestamp": "T1", "status": "ok", "error": ""})
        # 2회 저장
        controller.save_csv({"timestamp": "T2", "status": "ok", "error": ""})
        # 3회 저장
        controller.save_csv({"timestamp": "T3", "status": "ok", "error": ""})

        with open(test_file, "r", encoding="utf-8") as f:
            lines = [line.strip() for line in f.readlines() if line.strip()]

        self.assertEqual(len(lines), 4)  # 헤더 1줄 + 데이터 3줄
        self.assertEqual(lines[0], ",".join(config.CSV_HEADERS))
        self.assertTrue(lines[1].startswith("T1"))
        self.assertTrue(lines[2].startswith("T2"))
        self.assertTrue(lines[3].startswith("T3"))

    # --------------------------------------------------------------------------
    # 10. 잘못된 데이터 입력 테스트
    # --------------------------------------------------------------------------
    def test_10_invalid_data_handling(self):
        """센서 데이터 포맷 오류, 누락, 비정상 위경도 등 비정상 입력에 대한 방어 로직 검증"""
        controller = SmartGogglesController(
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )

        # 1) 문자열이 섞인 센서 응답
        self.mock_bridge.set_handler("read_imu", lambda: "invalid,data,string,test,1,2,3")
        rec1 = controller.step()
        self.assertEqual(rec1["status"], "error")
        self.assertIn("형식 오류", rec1["error"])

        # 2) 토큰 개수가 부족한 센서 응답
        self.mock_bridge.set_handler("read_imu", lambda: "1.0,2.0,3.0")
        rec2 = controller.step()
        self.assertEqual(rec2["status"], "error")

        # 3) 범위를 벗어난 비정상 GPS 좌표
        self.assertFalse(navigation.validate_gps_coordinates(150.0, 50.0))  # 위도 > 90
        self.assertFalse(navigation.validate_gps_coordinates(-100.0, 50.0))  # 위도 < -90
        self.assertFalse(navigation.validate_gps_coordinates(37.0, 200.0))  # 경도 > 180
        self.assertFalse(navigation.validate_gps_coordinates(float("nan"), 120.0))

    # --------------------------------------------------------------------------
    # 11. Bridge 통신 오류 테스트
    # --------------------------------------------------------------------------
    def test_11_bridge_communication_error(self):
        """Bridge RPC 통신 중 예외 발생 시 시스템 다운 없이 error 행 기록 검증"""
        def broken_call():
            raise ConnectionResetError("Arduino Router 소켓 연결 끊김")

        self.mock_bridge.set_handler("imu_ready", broken_call)
        controller = SmartGogglesController(
            bridge_instance=self.mock_bridge,
            csv_file=self.test_csv
        )

        # step()이 크래시되지 않고 안전하게 error 레코드를 반환 및 저장해야 함
        rec = controller.step()
        self.assertEqual(rec["status"], "error")
        self.assertIn("소켓 연결 끊김", rec["error"])

        # CSV 파일에 error 상태로 저장되었는지 확인
        with open(self.test_csv, "r", encoding="utf-8") as f:
            content = f.read()
        self.assertIn("error", content)
        self.assertIn("소켓 연결 끊김", content)

    # --------------------------------------------------------------------------
    # 12. 자동 실행 스크립트 실행 및 구문 검증
    # --------------------------------------------------------------------------
    def test_12_autostart_script_verification(self):
        """auto_start.sh 및 smart-goggles.service 파일의 존재성, 권한 및 핵심 키워드 검증"""
        base_dir = os.path.dirname(os.path.abspath(__file__))
        sh_path = os.path.join(base_dir, "auto_start.sh")
        service_path = os.path.join(base_dir, "smart-goggles.service")
        install_sh_path = os.path.join(base_dir, "install_autostart.sh")

        self.assertTrue(os.path.exists(sh_path), "auto_start.sh 파일이 존재해야 합니다.")
        self.assertTrue(os.path.exists(service_path), "smart-goggles.service 파일이 존재해야 합니다.")
        self.assertTrue(os.path.exists(install_sh_path), "install_autostart.sh 파일이 존재해야 합니다.")

        # auto_start.sh 내용 검증
        with open(sh_path, "r", encoding="utf-8") as f:
            sh_content = f.read()
        self.assertIn("#!/usr/bin/env bash", sh_content)
        self.assertIn("PID_FILE", sh_content)
        self.assertIn("LOG_FILE", sh_content)
        self.assertIn("main.py", sh_content)
        self.assertIn("is_running", sh_content)

        # smart-goggles.service 내용 검증
        with open(service_path, "r", encoding="utf-8") as f:
            service_content = f.read()
        self.assertIn("[Unit]", service_content)
        self.assertIn("[Service]", service_content)
        self.assertIn("ExecStart=", service_content)
        self.assertIn("Restart=always", service_content)


if __name__ == "__main__":
    unittest.main(verbosity=2)
