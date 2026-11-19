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
   - `Bridge.call("read_heading")`: MCU가 계산한 진북 기준 시계방향 방위각 수신 (0 이상 360 미만, 기준 없음은 -999)
   - `Bridge.call("set_heading", heading, bias_z)`: 현재 자세의 알려진 방위각과 자이로 바이어스를 MCU에 전달 (bool 성공 응답)
   - `Bridge.call("read_gps")`: GPS 수신 (확장 인터페이스)
2. **목표 방위각(Target Bearing) 계산**:
   - 구면 삼각법(Great-Circle Forward Azimuth) 기반으로 현재 GPS 위치에서 목표 위치까지의 방위각(0° ~ 359.9°)을 정밀 계산
3. **MCU 방향 수신(Heading Update)**:
   - MCU가 약 100Hz로 MPU-6050 Z축 각속도를 적분하며 Python은 `read_heading`으로 받은 현재 방향을 사용
   - MCU에 이미 기준이 있으면 Python의 초기 Heading 설정 없이도 즉시 사용
   - MCU에도 기준이 없으면 `GOGGLES_INITIAL_HEADING`을 `set_heading`으로 한 번 전달한 뒤 MCU의 실제 방위각을 다시 수신
   - 기준 미설정/소실 시 `waiting_heading` 상태와 설정 방법을 기록하며 IMU/GPS 수집은 계속 수행. Python에서 별도의 자이로 적분이나 임의 방위각 대체는 하지 않음
   - MCU 재시작/센서 오류 등으로 기준을 잃으면 과거 시작 방향을 자동 재적용하지 않음. 현재 자세를 기준으로 다시 설정해야 함
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
| `test_smart_goggles.py` | 센서/GPS, MCU 방향 수신, 기준 설정 및 통신 실패에 대한 단위/통합 테스트 |
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

# 실제 현재 바라보는 방위각 (도 단위: 진북 0, 동 90, 남 180, 서 270)
# 아래 예시는 진북을 향한 상태에서 시작할 때만 사용
export GOGGLES_INITIAL_HEADING=0.0

# GPS 수신 방식: "bridge" | "mock" | "serial" | "none"
export GOGGLES_GPS_SOURCE="bridge"

# MPU-6050 Z축 자이로 오프셋 바이어스 (deg/s)
export GOGGLES_GYRO_Z_BIAS=0.0
```

---

### 첫 실행 시 기준 방향 설정

먼저 변경된 [MCU 스케치](sketch/sketch.ino)를 보드에 업로드해야 합니다. 기존 스케치는 `read_heading`과 `set_heading`을 제공하지 않아 Python이 방향을 수신할 수 없습니다.

MPU-6050에는 나침반이 없으므로 기준 없이 현재 동서남북을 알아낼 수 없습니다. 외부 나침반 등으로 착용자가 바라보는 방위각을 확인하고 `GOGGLES_INITIAL_HEADING`에 넣은 뒤, 그 방향을 유지한 상태에서 프로그램을 시작하세요. MCU에 유효한 기준이 이미 있으면 기존 방향을 유지합니다.

```bash
# 예: 고글을 동쪽(90도)으로 향한 상태에서 실행
GOGGLES_INITIAL_HEADING=90 python main.py
```

`heading`과 목표 방위각은 모두 진북 기준 시계방향 각도입니다. 센서 Z축이 위를 향하는 기본 장착에서는 +gz가 반시계 회전이므로 MCU에서 부호를 반전합니다. 반대로 부착했다면 스케치의 `GYRO_TO_HEADING_SIGN`을 조정하세요. 정지 상태의 gz 평균은 `GOGGLES_GYRO_Z_BIAS`로 전달할 수 있습니다.

초기 기준 설정은 프로세스 시작 후 한 번만 시도합니다. 설정 요청이 MCU에 적용된 뒤 응답만 유실됐을 수 있으므로, 통신 오류가 나도 같은 시작 방향을 자동으로 다시 적용하지 않습니다. 기준이 소실되면 실제 현재 방위각을 확인하고 그 자세에서 다시 시작하거나 `set_heading`으로 재설정하세요. IMU의 NaN/무한대, 피치 오류(-999 포함), CSV 저장 실패도 오류로 보고합니다.

Bridge 호출 및 `provide_safe`의 메인 루프 실행 방식은 [Arduino 공식 Bridge API](https://docs.arduino.cc/software/app-lab/bridge/bridge-api/)를 기준으로 합니다.

CSV의 `status`는 정상 항법 시 `ok`, 방향 기준 대기 시 `waiting_heading`, GPS 미수신/목표 방위각 계산 불가 시 `waiting_gps`, RPC/데이터 오류 시 `error`입니다. 대기 원인은 기존 `error` 컬럼과 로그에 표시되며 CSV 컬럼 규격은 유지됩니다. 방향과 오차를 모두 계산하려면 유효한 MCU 방위각과 GPS가 함께 필요합니다. 현재 스케치에는 GPS 수신 구현이 없으므로 별도로 연결/구현해야 합니다.

---

## 4. 실행 및 테스트 방법

### 4.1 PC 및 테스트 환경에서 단위 테스트 실행
하드웨어 연결 없이 센서/GPS 기능과 MCU 방향 수신·초기화·실패/복구 시나리오를 검증할 수 있습니다.

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
   - MCU에 이미 유효한 방위각이 있으면 사용합니다. 없으면 출발 전 착용자의 실제 바라보는 방향을 확인해 `GOGGLES_INITIAL_HEADING`으로 설정해야 합니다. GPS 이동 방향에서 고글이 바라보는 방향을 자동 추정하지 않습니다.
3. **자이로 드리프트 캘리브레이션**:
   - 센서 부착 방향 및 고글 착용 시 Z축이 상방을 향하는지 축 방향 일치 여부 확인 필요.
   - 장시간 주행 시 누적 오차를 줄이기 위해 정지 상태에서 `GYRO_Z_BIAS` 보정값 측정 권장.
