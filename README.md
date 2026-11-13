# smart-goggles

Arduino UNO Q (STM32U585 MCU + Linux)를 활용한 Smart Goggles 팀 프로젝트입니다.

======
- 이지훈
analysis 와 main.py를 추가 했습니다.<br>
inu_log.csv 파일과 simulate.py 는 테스트용 파일입니다. 실제로 작동되는지는 아두이노에 연결해서 확인해야합니다.<br>
======

---

## 1. 시스템 구조 및 Linux Python 담당 기능

본 프로젝트는 Arduino UNO Q 보드의 STM32U585 MCU(스케치)와 온보드 Linux(Python)가 **Arduino Bridge(RPC)**로 통신하며 동작합니다.

### 주요 담당 기능:
1. **Bridge 통신 데이터 수신**:
   - `Bridge.call("imu_ready")`: MPU-6050 I2C 준비 상태 확인
   - `Bridge.call("read_imu")`: 가속도(ax,ay,az), 각속도(gx,gy,gz), 온도(temp) 수신
   - `Bridge.call("read_pitch")`: 앞뒤 기울기(pitch) 수신
   - `Bridge.call("read_gps")`: GPS 수신 (확장 인터페이스)
2. **목표 방위각(Target Bearing) 계산**:
   - 구면 삼각법(Great-Circle Forward Azimuth) 기반으로 현재 GPS 위치에서 목표 위치까지의 방위각(0° ~ 359.9°)을 정밀 계산
3. **자이로 방향 추정(Heading Update)**:
   - MPU-6050 Z축 각속도(deg/s)를 시간에 따라 적분하여 방향 갱신
   - **초기 Heading 미설정 원칙**: 지자기 센서가 없는 MPU-6050 특성상 초기 절대 방위각이 주어지지 않은 경우 임의의 절대 방향을 출력하지 않고 상대 회전량만 누적 추적
4. **목표 방향과 현재 방향의 차이(Angle Error) 계산**:
   - `target_bearing - current_heading`을 계산하고 `-180° ~ +180°` 범위로 정규화 (양수: 우회전 필요, 음수: 좌회전 필요)
5. **표준 `team.csv` 자동 저장**:
   - 필수 6개 컬럼(`timestamp, latitude, longitude, target_bearing, heading, angle_error`)과 IMU 원시 센서 데이터, 실행 상태를 포함하는 하위 호환 표준 포맷
   - UTF-8 인코딩, 최초 헤더 자동 생성, 기존 파일 데이터 보존
6. **부팅 시 자동 실행(Auto-start)**:
   - Linux 부팅 완료 및 Arduino Router 서비스 준비 후 백그라운드 자동 실행 지원 (`auto_start.sh`, `smart-goggles.service`)

---

## 2. 파일 목록 및 역할

| 파일명 | 역할 |
|---|---|
| `main.py` | 프로그램 진입점 및 권장 함수(`read_sensor`, `read_gps`, `calculate_target_bearing`, `update_heading`, `calculate_angle_error`, `save_csv` 등) 구현 |
| `config.py` | 목표 좌표, 주기, GPS 모드, 자이로 바이어스, CSV 경로 등 전역 설정 관리 |
| `navigation.py` | 방위각, 각도 오차, 자이로 적분, GPS/NMEA 파싱 순수 알고리즘 모듈 |
| `bridge_interface.py` | Arduino Bridge 통신 래퍼 및 PC/테스트용 Mock Bridge 계층 |
| `auto_start.sh` | 보드 부팅 자동 실행, 중복 실행 방지(PID), 프로세스 시작/중지/상태 관리 쉘 스크립트 |
| `smart-goggles.service` | systemd 등록용 서비스 유닛 파일 |
| `install_autostart.sh` | 보드에서 원클릭으로 systemd 서비스를 등록 및 활성화하는 스크립트 |
| `test_smart_goggles.py` | 12가지 요구 항목에 대한 단위 및 통합 테스트 스위트 |
| `sketch/sketch.ino` | MCU (STM32U585) 펌웨어: MPU-6050 읽기 및 Bridge RPC 제공 |
| `analysis/analyze_csv.py` | 팀 표준 CSV 및 imu_log 분석 도구 |
| `analysis/simulate.py` | 가상 센서 데이터 시뮬레이션 스크립트 |

---

## 3. 설정 (`config.py`) 및 환경 변수

하드웨어나 팀 규격이 확정되지 않은 설정값은 `config.py` 또는 환경 변수로 손쉽게 조정할 수 있습니다.

```bash
# 목표 GPS 위치 (위도, 경도)
export GOGGLES_TARGET_LAT=37.4963
export GOGGLES_TARGET_LON=126.9572

# 초기 방위각 (도 단위, 미설정 시 상대 회전각만 추적)
export GOGGLES_INITIAL_HEADING=0.0

# GPS 수신 방식: "bridge" | "mock" | "serial" | "none"
export GOGGLES_GPS_SOURCE="bridge"

# MPU-6050 Z축 자이로 오프셋 바이어스 (deg/s)
export GOGGLES_GYRO_Z_BIAS=0.0
```

---

## 4. 실행 및 테스트 방법

### 4.1 PC 및 테스트 환경에서 단위 테스트 실행
하드웨어 연결 없이도 12가지 핵심 기능과 모의(Mock) 시나리오를 즉시 검증할 수 있습니다.

```bash
python test_smart_goggles.py
```

### 4.2 메인 프로그램 직접 실행 (포그라운드)
```bash
python main.py
```

### 4.3 생성된 CSV 분석
```bash
python analysis/analyze_csv.py team.csv
```

---

## 5. Arduino UNO Q 보드 배포 및 자동 실행 등록

### 5.1 스크립트 실행 권한 부여
```bash
chmod +x auto_start.sh install_autostart.sh
```

### 5.2 부팅 시 자동 실행 서비스 등록 (권장: systemd)
Arduino UNO Q 터미널에서 다음 명령을 실행하면 부팅 완료 후 자동으로 백그라운드 실행됩니다.

```bash
sudo ./install_autostart.sh
```

### 5.3 서비스 관리 및 로그 확인
```bash
# 서비스 상태 확인
sudo systemctl status smart-goggles.service

# 실시간 로그 확인
tail -f logs/smart_goggles.log

# 에러 로그 확인
tail -f logs/smart_goggles.err.log

# 서비스 중지 및 재시작
sudo systemctl stop smart-goggles.service
sudo systemctl restart smart-goggles.service
```

### 5.4 수동 백그라운드 관리 (`auto_start.sh` 직접 사용 시)
```bash
./auto_start.sh start    # 백그라운드 실행
./auto_start.sh status   # 상태 확인
./auto_start.sh stop     # 프로세스 종료
```

---

## 6. 하드웨어 연동 시 확인 필요 사항 (Checklist)

1. **GPS 모듈 연결 경로 확인**:
   - MCU에 연결되어 `sketch.ino`의 Bridge(`read_gps`)로 전달되는지,
   - Linux 프로세서의 USB-to-Serial(`/dev/ttyACM0` 등)로 직접 연결되는지 확인 후 `config.py`의 `GPS_SOURCE` 조정 필요.
2. **초기 절대 방위각(Initial Heading) 결정 방식**:
   - MPU-6050은 지자기 센서가 없으므로 정지 상태에서 절대 방위각을 알 수 없습니다.
   - GPS 이동 궤적을 이용한 초기 헤딩 동기화 또는 출발 전 착용자의 초기 바라보는 방향(예: 0° 진북)을 설정(`GOGGLES_INITIAL_HEADING`)해야 절대 각도가 산출됩니다.
3. **자이로 드리프트 캘리브레이션**:
   - 센서 부착 방향 및 고글 착용 시 Z축이 상방을 향하는지 축 방향 일치 여부 확인 필요.
   - 장시간 주행 시 누적 오차를 줄이기 위해 정지 상태에서 `GYRO_Z_BIAS` 보정값 측정 권장.
