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
   - `Bridge.call("read_gps")`: MCU의 실제 또는 가상 GPS 좌표 수신
   - `Bridge.call("read_nav")`: `ms,state,turn,error,heading,bearing,dist,lat,lon,sats,mock,cal` 형식의 상태 수신. 5번째 필드가 현재 방위각이며 미설정 시 빈칸
   - `Bridge.call("set_heading", heading)`: 현재 자세의 알려진 방위각 설정 요청 (인자 1개, 비동기 반영)
2. **목표 방위각(Target Bearing) 계산**:
   - 구면 삼각법(Great-Circle Forward Azimuth) 기반으로 현재 GPS 위치에서 목표 위치까지의 방위각(0° ~ 359.9°)을 정밀 계산
3. **MCU 방향 수신(Heading Update)**:
   - `feature/fix-main-fin`의 통합 MCU 펌웨어가 자이로 영점 보정, 적분, GPS 진행 방향 보정을 수행하고 Python은 `read_nav`의 방위각을 사용
   - Python에서 별도로 자이로를 적분하지 않음. 기존 `update_heading(gz, dt)` 호출 형식은 유지하되 인자는 방향 계산에 사용하지 않음
   - MCU 방향이 이미 있으면 초기값 없이 사용. MCU에도 방향이 없을 때만 `GOGGLES_INITIAL_HEADING`을 한 번 요청하고 이후 `read_nav`에서 실제 적용 결과를 확인
   - 통신 오류나 기준 소실 시 과거 방향을 기록하거나 시작 방향을 자동 재적용하지 않음
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
| `config.py` | 목표 좌표, 주기, GPS 모드, 선택적 초기 방향, CSV 경로 등 전역 설정 관리 |
| `navigation.py` | 방위각, 각도 오차, 자이로 적분, GPS/NMEA 파싱 순수 알고리즘 모듈 |
| `bridge_interface.py` | Arduino Bridge 통신 래퍼 및 PC/테스트용 Mock Bridge 계층 |
| `auto_start.sh` | 보드 부팅 자동 실행, 중복 실행 방지(PID), 프로세스 시작/중지/상태 관리 쉘 스크립트 |
| `smart-goggles.service` | systemd 등록용 서비스 유닛 파일 |
| `install_autostart.sh` | 보드에서 원클릭으로 systemd 서비스를 등록 및 활성화하는 스크립트 |
| `test_smart_goggles.py` | 기본 수집 기능 및 MCU 통신 규약, 비동기 방향 설정, 실패/복구 테스트 |
| `sketch/sketch.ino` | MCU 통합 펌웨어: MPU-6050, GPS, SSD1309 OLED, 좌우 진동모터 및 Bridge RPC |
| `analysis/analyze_csv.py` | 팀 표준 CSV 및 imu_log 분석 도구 |
| `analysis/simulate.py` | 가상 센서 데이터 시뮬레이션 스크립트 |

---

## 3. 설정 (`config.py`) 및 환경 변수

하드웨어나 팀 규격이 확정되지 않은 설정값은 `config.py` 또는 환경 변수로 손쉽게 조정할 수 있습니다.

```bash
# 목표 GPS 위치 (위도, 경도)
export GOGGLES_TARGET_LAT=37.4963
export GOGGLES_TARGET_LON=126.9572

# 선택 사항: 실제 바라보는 방위각 (0=북, 90=동, 180=남, 270=서)
# MCU에도 방향이 없을 때만 사용. 예시는 진북을 향한 상태에서 실행할 때의 값
export GOGGLES_INITIAL_HEADING=0.0

# GPS 수신 방식: "bridge" | "mock" | "serial" | "none"
export GOGGLES_GPS_SOURCE="bridge"

```

`GOGGLES_GYRO_Z_BIAS`, `GOGGLES_AUTO_CALIBRATE`는 기존 설정 호환용으로 남아 있으며 통합 MCU에는 전송하지 않습니다. 부팅 후 약 2초간 보드를 가만히 두면 MCU가 자동으로 영점을 측정합니다.

### 통합 펌웨어 호환 및 첫 실행

이 Python 코드는 원격 `feature/fix-main-fin`의 통합 [MCU 스케치](sketch/sketch.ino) 규약을 사용합니다. 해당 스케치를 보드에 업로드하고 실행하세요. 이 펌웨어에는 `read_heading` RPC가 없으며 `set_heading`에 자이로 바이어스 인자를 추가하면 호환되지 않습니다.

MCU의 기본 `GPS_MODE=2`는 가상 GPS로 시작한 뒤 유효한 NMEA 문장이 수신되면 실제 GPS로 전환합니다. 가상 모드의 방향은 `MOCK_START_HEADING`에서 시작합니다. 실제 GPS 모드에서는 걸어서 진행 방향을 얻거나 현재 바라보는 방향을 수동 지정해야 합니다. 실행 로그의 `MCU GPS 모드`에서 실제/가상 여부를 확인할 수 있습니다. Linux의 `GOGGLES_GPS_SOURCE=mock`은 Python 좌표만 바꾸므로 보드의 모드를 확인하려면 `bridge`를 사용하세요.

`set_heading`의 `True`는 요청 접수만 뜻하며 다음 MCU 루프에서 반영됩니다. Python은 방향을 바로 만들어 넣지 않고 다음 수집 주기에서 확인합니다. 요청 실패/응답 유실 뒤에도 같은 시작 방향을 자동 재전송하지 않습니다. 방향을 잃었다면 현재 자세의 방위각을 확인한 뒤 다시 시작하세요.

CSV 컬럼 규격은 유지하며 `status`는 `ok`, `waiting_calibration`(영점 측정), `waiting_heading`(방향 대기), `waiting_gps`(GPS/목표 방위각 대기), `error`로 기록합니다. 대기 이유는 `error` 컬럼과 로그에 표시됩니다. `read_nav`의 빈 heading은 정상 대기이며 잘못된 형식, NaN/무한대, 피치 실패(-999)는 오류입니다.

목표 방위각과 오차는 Python의 설정 좌표로 계산합니다. OLED/진동 안내의 목적지도 바꾸려면 MCU의 `set_target(lat, lon)` 또는 스케치의 `TARGET_LAT/LON`을 같은 좌표로 설정하세요.

---

## 4. 실행 및 테스트 방법

### 4.1 PC 및 테스트 환경에서 단위 테스트 실행
하드웨어 연결 없이 센서/GPS 수집, MCU 방향 수신, 비동기 초기화와 통신 실패/복구를 검증할 수 있습니다.

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
   - MCU가 실제 GPS 진행 방향으로 초기화하거나, 출발 전 실제 바라보는 방향을 `GOGGLES_INITIAL_HEADING`으로 요청합니다. MCU에 이미 방향이 있으면 기존 값을 사용합니다.
3. **자이로 드리프트 캘리브레이션**:
   - 센서 부착 방향 및 고글 착용 시 Z축이 상방을 향하는지 축 방향 일치 여부 확인 필요.
   - 통합 MCU가 정지 상태의 자이로 영점을 자동 측정합니다. 부팅 시 OLED의 `HOLD STILL` 표시가 끝날 때까지 가만히 두세요.
