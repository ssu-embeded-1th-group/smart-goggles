/*
 * sketch.ino  —  Arduino UNO Q (MCU 쪽, STM32U585)  Smart Goggles 통합 펌웨어
 * MPU-6050 + GPS + OLED 화살표 + 좌/우 진동모터, 실내 책상 디버깅용 가상 GPS(Mock) 포함
 * 코드 컨벤션: snake_case, 절차지향 (클래스 없이 함수 + 전역 상태)
 * 필요한 라이브러리: Arduino_RouterBridge, Wire, SPI 뿐 (모두 기본 포함, OLED·GPS 라이브러리 설치 불필요)
 *
 * ── 동작 (0.1초마다) ─────────────────────────────────
 *   ① MPU-6050 읽기 → 자이로를 적분해 지금 바라보는 방향(heading) 추정
 *   ② GPS 위치(실제 또는 가상) → 목적지까지 거리와 방위각(bearing) 계산
 *   ③ angle error = bearing − heading (+면 오른쪽, −면 왼쪽으로 돌아야 함) 으로 방향 이탈 판단
 *   ④ OLED: 목적지 쪽을 가리키는 화살표 + 거리 / 진동모터: 돌아야 하는 쪽이 울림
 *
 * ── 배선 (결선도 rev 3.0, UNO Q 핀은 3.3V 로직) ──────
 *   MPU-6050 : VCC→3.3V  GND→GND  SDA→헤더 SDA(D20)  SCL→헤더 SCL(D21)  AD0→GND (주소 0x68, A4/A5 아님)
 *   OLED     : Waveshare 1.51인치 투명 OLED (SSD1309), 4선 SPI (출고 기본값)  VCC→3.3V  GND→GND
 *              DIN→D11  CLK→D13  CS→D10  DC→D7  RES(RST)→D8
 *              ※ 뒷면 0Ω 저항을 옮겨 I2C로 바꾼 모듈이면 OLED_USE_SPI를 0으로 하고 SDA/SCL에 연결
 *   GPS      : GND→GND  GPS의 TX→D0(RX)   VCC는 모듈 규격대로
 *              ※ GPS의 RX→D1은 연결해도, 안 해도 됨 (스케치는 GPS로 아무것도 보내지 않음)
 *   진동모터 : 왼쪽 D5, 오른쪽 D6 → 트랜지스터(모터 드라이버) 모듈의 신호 입력
 *              ※ 모터를 핀에 직접 연결하면 안 됨 (모터가 핀이 낼 수 있는 전류보다 많이 씀)
 *
 * ── GPS 모드 (GPS_MODE) ─────────────────────────────
 *   2 = 자동 (기본값): 부팅 후 가상 GPS로 시작했다가, D0로 GPS 문장(체크섬이 맞는 NMEA)이
 *       한 번이라도 들어오면 그때부터 실제 GPS로 바뀜 (재부팅 전까지 다시 가상으로 돌아가지 않음)
 *       → 책상(GPS 모듈 없음)은 자동으로 가상, 야외 실측(GPS 모듈 연결)은 자동으로 실제
 *   1 = 항상 가상 GPS : GPS 모듈을 꽂은 채로 책상 테스트를 할 때
 *   0 = 항상 실제 GPS : 실측 때 확실하게 고정하고 싶을 때
 *   리눅스에서 set_gps_mock(True/False)를 호출하면 그 값으로 고정됨 (자동 전환 끝)
 *   지금 모드는 화면 맨 아래("MOCK GPS" / "GPS S08")와 read_nav의 mock 칸에서 확인
 *
 * ── 가상 GPS ─────────────────────────────────────────
 *   GPS 모듈 없이 책상에서 테스트하는 모드
 *   · 시작 위치 = MOCK_START_LAT/LON, 부팅 때 바라보는 방향 = MOCK_START_HEADING(0 = 북쪽)으로 가정
 *   · 보드를 돌리면 heading이 바뀌어 화살표와 진동이 따라 바뀜
 *   · 화살표가 정면(GO)인 동안에는 바라보는 방향으로 MOCK_SPEED_MPS 속도로 "걸어감" → 도착 화면까지 확인 가능
 *
 * ── Bridge로 제공하는 함수 (Python: Bridge.call("이름", 인자...)) ──
 *   imu_ready()          → bool   : 최근 0.1초 주기의 MPU-6050 읽기가 성공했으면 true
 *   read_imu()           → String : "ax,ay,az,gx,gy,gz,temp" (g, °/s, °C) / 실패 시 ""
 *   read_pitch()         → float  : 앞뒤 기울기(°). read_imu 직후에 부르면 같은 샘플의 값 / 실패 시 -999
 *   read_gps()           → String : "lat,lon" (소수 6자리, 가상 GPS일 때는 가상 위치) / 위치 없으면 ""
 *   read_nav()           → String : "ms,state,turn,error,heading,bearing,dist,lat,lon,sats,mock,cal"
 *                                    state = IMU_ERR | CAL | NO_FIX | NO_HEADING | GUIDE | ARRIVED
 *                                    turn  = 0 직진, ±1 조금, ±2 크게 (−왼쪽 +오른쪽), 3 뒤로 / 모르는 값은 빈칸
 *   set_target(lat, lon) → bool   : 목적지 변경
 *   set_heading(deg)     → bool   : 지금 바라보는 방향을 직접 지정 (0=북, 90=동)
 *   set_gps_mock(on)     → bool   : 가상 GPS 켜기/끄기
 *   ※ 읽는 함수는 0.1초마다 통째로 복사해 둔 값(published)만 돌려줌 → 함수 안에서 I2C 통신·delay 없음
 *   ※ set_ 함수는 요청만 적어 두고(pending) loop()가 다음 바퀴(수 ms 안)에 반영함
 *     (Bridge 함수는 loop()와 다른 스레드에서 loop() 도중에 실행될 수 있어서, 센서·길안내 변수를 직접 건드리지 않게 함)
 *
 * ── 알려진 한계 ──────────────────────────────────────
 *   · MPU-6050에는 나침반이 없음 → 실제 GPS 모드에서는 걷기 시작해 GPS 진행 방향이 잡혀야 화살표가 나옴
 *     (그 전에는 "WALK" 표시. set_heading으로 직접 지정할 수도 있음)
 *   · 부팅 직후 약 2초는 가만히 둬야 함 (자이로 영점 측정, 화면에 "HOLD STILL")
 *   · pitch는 가속도만으로 계산 → 걷거나 고개를 빨리 움직이면 값이 튐
 *   · temp는 정품 MPU-6050 공식 → 호환 칩(칩 ID 0x72 등)은 실제 온도와 다를 수 있어 참고용
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <SPI.h>

// ═════════ 1. 설정: 하드웨어에 맞게 여기만 고치면 됨 ═════════

// 실행 모드
#define GPS_MODE            2           // 2: 자동 (GPS 데이터가 들어오면 실제, 없으면 가상) / 1: 항상 가상 / 0: 항상 실제

// 목적지 (리눅스 config.py의 TARGET_LATITUDE / TARGET_LONGITUDE 기본값과 같은 값)
#define TARGET_LAT          37.4963
#define TARGET_LON          126.9572

// 가상 GPS (config.py의 MOCK_GPS 좌표와 같음: 목적지까지 약 242m, 방위각 약 53°)
#define MOCK_START_LAT      37.4950
#define MOCK_START_LON      126.9550
#define MOCK_START_HEADING  0.0f        // 부팅 때 바라보는 방향으로 가정할 값 (0 = 북쪽)
#define MOCK_SPEED_MPS      5.0f        // 가상으로 걸어가는 속도 [m/s] (0이면 제자리, 실제 걸음은 약 1.4)

// 진동모터 핀
#define MOTOR_LEFT_PIN      5
#define MOTOR_RIGHT_PIN     6
#define MOTOR_ACTIVE_LOW    0           // 드라이버 모듈이 LOW에서 모터를 켜는 방식이면 1

// I2C 버스 (MPU-6050): 헤더 SDA/SCL → Wire | Qwiic 커넥터 → Wire1 | A4/A5 → Wire2
#define IMU_WIRE            Wire
#define I2C_CLOCK_HZ        0           // 0: 코어 기본 속도 그대로 / 400000: 선이 짧을 때 고속

// OLED (128x64)
#define OLED_USE_SPI        1           // 1: 4선 SPI (결선도 rev 3.0, 출고 기본값) / 0: I2C (0Ω 저항을 옮겨 납땜한 경우)
// SPI일 때 (DIN→D11, CLK→D13은 하드웨어 SPI 핀으로 고정)
#define OLED_DC_PIN         7           // DC: 데이터/명령 선택
#define OLED_RST_PIN        8           // RES(RST): 리셋
#define OLED_CS_PIN         10          // CS: 칩 선택
#define OLED_SPI_HZ         4000000     // SPI 속도 [Hz] (SSD1306 최대 약 10MHz, 화면이 깨지면 1000000으로 낮춤)
// I2C일 때
#define OLED_WIRE           Wire        // MPU-6050과 같은 SDA/SCL
#define OLED_ADDR           0x3C        // 대부분 0x3C, 일부 모듈은 0x3D
// 공통
#define OLED_CHIP           1309        // 1309: SSD1309 (Waveshare 1.51인치 투명 OLED) / 1306: SSD1306 (0.96인치) / 1106: SH1106 (1.3인치)
#define OLED_FLIP_H         0           // 1: 좌우 반전 (반사판·거울로 볼 때)
#define OLED_FLIP_V         0           // 1: 상하 반전

// GPS UART (GPS의 TX → D0)
#define GPS_BAUD            9600
// D0/D1 UART의 이름은 보드 코어 버전에 따라 다름 → 자동 선택
//   코어 0.55 이상: Serial = Monitor(리눅스 콘솔), D0/D1 = Serial1
//   그 이전 코어  : D0/D1 = Serial, Serial1은 Bridge 전용 (GPS에 쓰면 Bridge가 끊김)
#if OLED_CHIP != 1309 && OLED_CHIP != 1306 && OLED_CHIP != 1106
#error "OLED_CHIP은 1309, 1306, 1106 중 하나여야 함"
#endif
#if GPS_MODE < 0 || GPS_MODE > 2
#error "GPS_MODE는 0, 1, 2 중 하나여야 함"
#endif

#ifndef GPS_SERIAL
#ifdef ARDUINO_ROUTER_SERIAL
#define GPS_SERIAL Serial1
#else
#define GPS_SERIAL Serial
#endif
#endif

// ═════════ 2. 판단 기준·주기 (필요하면 조정) ═════════
const unsigned long TICK_MS             = 100;      // 센서 읽기·판단 주기 [ms] (0.1초)
const float ALIGN_DEG                   = 20.0f;    // |angle error|가 이보다 작으면 "정면" (직진)
const float TURN_DEG                    = 60.0f;    // 이보다 크면 "크게 꺾기"
const float BACK_DEG                    = 135.0f;   // 이보다 크면 "뒤로 돌기"
const float HYST_DEG                    = 5.0f;     // 경계에서 깜빡이지 않게 두는 여유 (커질 때 +, 작아질 때 −)
const unsigned long TURN_HOLD_MS        = 300;      // 이탈이 이 시간 이상 이어져야 이탈로 판단 (잠깐 고개 돌림은 무시)
const unsigned long TURN_RELEASE_MS     = 100;      // 방향을 바로잡았을 때는 빨리 해제
const float ARRIVE_RADIUS_M             = 10.0f;    // 목적지에서 이 거리 안이면 도착
const float ARRIVE_EXIT_M               = 15.0f;    // 도착한 뒤 이 거리보다 멀어지면 다시 안내
const unsigned long GPS_TIMEOUT_MS      = 3000;     // 이 시간 동안 새 위치가 없으면 "GPS 없음"
const float COURSE_MIN_SPEED_MPS        = 0.8f;     // 이 속도 이상으로 움직일 때만 GPS 진행 방향을 믿음
const uint8_t COURSE_INIT_COUNT         = 3;        // 진행 방향이 연속으로 이만큼 들어오면 heading을 그 값으로 시작
const float COURSE_GAIN                 = 0.1f;     // GPS 진행 방향 쪽으로 heading을 조금씩 당기는 비율 (자이로 드리프트 보정)
const unsigned long DEBUG_PRINT_MS      = 1000;     // Monitor에 상태 한 줄 출력 주기 [ms] (0이면 끔)

// 진동 패턴: 켜지는 시간 / 한 박자 길이 [ms]
const unsigned long VIB_SLIGHT_ON_MS    = 80;       // 조금 벗어남: 1초에 한 번 짧게
const unsigned long VIB_SLIGHT_PERIOD_MS = 1000;
const unsigned long VIB_HARD_ON_MS      = 120;      // 크게 벗어남: 0.4초마다
const unsigned long VIB_HARD_PERIOD_MS  = 400;
const unsigned long VIB_BACK_ON_MS      = 150;      // 뒤로 돌기: 왼쪽·오른쪽 번갈아
const unsigned long VIB_BACK_PERIOD_MS  = 600;
const unsigned long VIB_ARRIVE_ON_MS    = 300;      // 도착: 양쪽 동시에 3번
const unsigned long VIB_ARRIVE_PERIOD_MS = 500;
const uint8_t VIB_ARRIVE_REPEAT         = 3;

// ═════════ 3. 자료형 (구조체는 첫 함수보다 위에 둠) ═════════
struct imu_sample {
  float ax, ay, az;                      // 가속도 [g]
  float gx, gy, gz;                      // 각속도 [°/s]
  float temp;                            // 온도 [°C]
  float pitch;                           // 앞뒤 기울기 [°] (같은 샘플의 가속도로 계산)
  unsigned long ms;                      // 읽은 시각 (millis)
  bool valid;                            // 실제로 읽은 값이면 true
};

struct gps_fix {
  double lat, lon;                       // 위도·경도 [°] (남반구·서경은 음수)
  float speed_mps;                       // 속도 [m/s]
  float course_deg;                      // 진행 방향 [°] (0=북, 시계방향)
  bool course_valid;                     // 진행 방향 값이 있는지 (멈춰 있으면 GPS가 비워 보냄)
  uint8_t sats;                          // 위성 수
  unsigned long ms;                      // 마지막으로 위치를 받은 시각 (millis)
  bool valid;                            // 지금 위치를 믿을 수 있으면 true (fix)
};

// loop()가 0.1초마다 Bridge 함수용으로 통째로 복사해 두는 값 묶음 ("게시판")
struct nav_snapshot {
  imu_sample imu;                        // 최신 MPU-6050 샘플
  bool fix_valid;                        // GPS 위치가 있는지
  double lat, lon;
  uint8_t sats;
  uint8_t state;                         // NAV_...
  int8_t turn;                           // 0 직진, ±1 조금, ±2 크게, 3 뒤로
  bool heading_ok, geo_ok, mock, cal;
  float error_deg, heading_deg, bearing_deg, distance_m;
};

// Bridge의 set_ 함수가 적어 두는 요청 ("요청함"). loop()가 꺼내서 반영
struct bridge_request {
  bool target_set;
  double target_lat, target_lon;
  bool heading_set;
  float heading_deg;
  bool mock_set;
  bool mock_on;
};

// 길안내 상태
const uint8_t NAV_IMU_ERROR   = 0;       // MPU-6050을 읽지 못함
const uint8_t NAV_CALIBRATING = 1;       // 자이로 영점 측정 중 (가만히 두기)
const uint8_t NAV_NO_FIX      = 2;       // GPS 위치 없음
const uint8_t NAV_NO_HEADING  = 3;       // 위치는 있지만 바라보는 방향을 아직 모름
const uint8_t NAV_GUIDE       = 4;       // 안내 중
const uint8_t NAV_ARRIVED     = 5;       // 도착
const char *const NAV_STATE_NAME[6] = { "IMU_ERR", "CAL", "NO_FIX", "NO_HEADING", "GUIDE", "ARRIVED" };

// 진동 모드
const uint8_t MOTOR_OFF       = 0;
const uint8_t MOTOR_LEFT      = 1;
const uint8_t MOTOR_RIGHT     = 2;
const uint8_t MOTOR_ALTERNATE = 3;       // 왼쪽·오른쪽 번갈아
const uint8_t MOTOR_BOTH      = 4;       // 양쪽 동시에

// ═════════ 4. 상수 ═════════
// MPU-6050
const uint8_t MPU6050_ADDR            = 0x68;     // I2C 주소 (AD0 = HIGH면 0x69). WHO_AM_I 칩 ID(0x68, 호환 칩 0x72 등)와는 다른 값
const uint8_t MPU6050_REG_CONFIG      = 0x1A;
const uint8_t MPU6050_REG_GYRO_CFG    = 0x1B;
const uint8_t MPU6050_REG_ACCEL_CFG   = 0x1C;
const uint8_t MPU6050_REG_ACCEL_XOUT  = 0x3B;     // 여기부터 14바이트: 가속도 XYZ(6) + 온도(2) + 자이로 XYZ(6)
const uint8_t MPU6050_REG_PWR_MGMT_1  = 0x6B;
const uint8_t MPU6050_REG_WHO_AM_I    = 0x75;     // 칩 ID가 들어 있는 레지스터 (I2C 주소 아님)
const uint8_t MPU6050_SLEEP_BIT       = 0x40;     // PWR_MGMT_1의 SLEEP 비트 (전원이 새로 들어오면 1로 시작)
const float MPU6050_ACCEL_LSB_PER_G   = 16384.0f; // ±2 g
const float MPU6050_GYRO_LSB_PER_DPS  = 65.5f;    // ±500 °/s (고개를 빨리 돌려도 포화되지 않게)
const float ERR_VALUE             = -999.0f;  // read_pitch 실패 시 반환값
const unsigned long IMU_WARMUP_MS    = 200;   // 센서를 깨운 뒤 값이 안정될 때까지 건너뛰는 시간
const unsigned long IMU_RETRY_MS     = 1000;  // 응답이 없을 때 다시 설정을 시도하는 간격
const unsigned long SAMPLE_STALE_MS  = 350;   // 이보다 오래된 샘플은 Bridge로 내보내지 않음
const unsigned long PITCH_LATCH_MS   = 200;   // read_imu 뒤 이 시간 안의 read_pitch는 같은 샘플의 값을 돌려줌
const unsigned long HEADING_LOST_MS  = 2000;  // MPU-6050이 이보다 오래 끊기면 heading을 버림 (그동안 돈 양을 모름)

// 자이로 영점(바이어스) 측정: 가만히 있는 구간의 평균을 영점으로 삼음
const uint8_t STILL_TICKS             = 20;     // 연속 2초 (0.1초 × 20)
const float STILL_GYRO_SPREAD_DPS     = 0.5f;   // 그동안 각속도 변동(최대−최소)이 이보다 작아야 "가만히"
const float STILL_ACCEL_SPREAD_G      = 0.04f;  // 가속도 크기 변동 한계
const float GYRO_BIAS_MAX_DPS         = 10.0f;  // 영점 오차로 볼 수 있는 최대 크기 (이보다 크면 "일정한 속도로 도는 중"으로 봄)
const float GYRO_BIAS_STEP_DPS        = 1.0f;   // 이미 잡은 영점에서 이보다 크게 달라진 값은 3번(6초) 연속 확인한 뒤에만 반영
const unsigned long GYRO_CAL_TIMEOUT_MS = 10000; // 이 시간 안에 못 잡으면 영점 0으로 일단 진행 (화면 HDG 뒤에 '?')
const float UP_ALPHA                  = 0.1f;   // 중력 방향 추정 저역통과 계수 (약 1초)

// 지구·각도
const double RAD_PER_DEG       = 0.017453292519943295;
const double EARTH_RADIUS_M    = 6371000.0;
const double METERS_PER_DEG    = 111320.0;      // 위도 1°의 길이 (가상 GPS 이동 계산용)

// OLED
const uint8_t OLED_PAGES              = 8;      // 64줄 = 8픽셀짜리 줄(page) 8개
#if OLED_USE_SPI
const uint8_t OLED_CHUNK              = 128;    // 한 번의 전송에 싣는 화면 데이터 바이트 수 (SPI: 한 줄 통째로)
#else
const uint8_t OLED_CHUNK              = 16;     // (I2C: Wire 버퍼 크기 안에서 작게 나눔)
#endif
const unsigned long OLED_RETRY_MS     = 2000;   // 응답이 없을 때 다시 초기화를 시도하는 간격
const unsigned long OLED_REFRESH_MS   = 5000;   // 설정을 다시 보내고 화면 전체를 새로 그리는 간격 (전원 순간 끊김 복구)
#if OLED_CHIP == 1106
const uint8_t OLED_COLUMN_OFFSET      = 2;      // SH1106은 내부 132칸 중 가운데 128칸을 씀
#else
const uint8_t OLED_COLUMN_OFFSET      = 0;
#endif

// 5x7 글꼴 (ASCII 0x20~0x7E, 글자당 5바이트 = 세로줄 5개, 비트0이 맨 위 픽셀)
const uint8_t FONT_5X7[] = {
  0x00,0x00,0x00,0x00,0x00,  0x00,0x00,0x5F,0x00,0x00,  0x00,0x07,0x00,0x07,0x00,  0x14,0x7F,0x14,0x7F,0x14,   // (space) ! " #
  0x24,0x2A,0x7F,0x2A,0x12,  0x23,0x13,0x08,0x64,0x62,  0x36,0x49,0x55,0x22,0x50,  0x00,0x05,0x03,0x00,0x00,   // $ % & '
  0x00,0x1C,0x22,0x41,0x00,  0x00,0x41,0x22,0x1C,0x00,  0x08,0x2A,0x1C,0x2A,0x08,  0x08,0x08,0x3E,0x08,0x08,   // ( ) * +
  0x00,0x50,0x30,0x00,0x00,  0x08,0x08,0x08,0x08,0x08,  0x00,0x60,0x60,0x00,0x00,  0x20,0x10,0x08,0x04,0x02,   // , - . /
  0x3E,0x51,0x49,0x45,0x3E,  0x00,0x42,0x7F,0x40,0x00,  0x42,0x61,0x51,0x49,0x46,  0x21,0x41,0x45,0x4B,0x31,   // 0 1 2 3
  0x18,0x14,0x12,0x7F,0x10,  0x27,0x45,0x45,0x45,0x39,  0x3C,0x4A,0x49,0x49,0x30,  0x01,0x71,0x09,0x05,0x03,   // 4 5 6 7
  0x36,0x49,0x49,0x49,0x36,  0x06,0x49,0x49,0x29,0x1E,  0x00,0x36,0x36,0x00,0x00,  0x00,0x56,0x36,0x00,0x00,   // 8 9 : ;
  0x00,0x08,0x14,0x22,0x41,  0x14,0x14,0x14,0x14,0x14,  0x41,0x22,0x14,0x08,0x00,  0x02,0x01,0x51,0x09,0x06,   // < = > ?
  0x32,0x49,0x79,0x41,0x3E,  0x7E,0x11,0x11,0x11,0x7E,  0x7F,0x49,0x49,0x49,0x36,  0x3E,0x41,0x41,0x41,0x22,   // @ A B C
  0x7F,0x41,0x41,0x22,0x1C,  0x7F,0x49,0x49,0x49,0x41,  0x7F,0x09,0x09,0x01,0x01,  0x3E,0x41,0x41,0x51,0x32,   // D E F G
  0x7F,0x08,0x08,0x08,0x7F,  0x00,0x41,0x7F,0x41,0x00,  0x20,0x40,0x41,0x3F,0x01,  0x7F,0x08,0x14,0x22,0x41,   // H I J K
  0x7F,0x40,0x40,0x40,0x40,  0x7F,0x02,0x04,0x02,0x7F,  0x7F,0x04,0x08,0x10,0x7F,  0x3E,0x41,0x41,0x41,0x3E,   // L M N O
  0x7F,0x09,0x09,0x09,0x06,  0x3E,0x41,0x51,0x21,0x5E,  0x7F,0x09,0x19,0x29,0x46,  0x46,0x49,0x49,0x49,0x31,   // P Q R S
  0x01,0x01,0x7F,0x01,0x01,  0x3F,0x40,0x40,0x40,0x3F,  0x1F,0x20,0x40,0x20,0x1F,  0x7F,0x20,0x18,0x20,0x7F,   // T U V W
  0x63,0x14,0x08,0x14,0x63,  0x03,0x04,0x78,0x04,0x03,  0x61,0x51,0x49,0x45,0x43,  0x00,0x00,0x7F,0x41,0x41,   // X Y Z [
  0x02,0x04,0x08,0x10,0x20,  0x41,0x41,0x7F,0x00,0x00,  0x04,0x02,0x01,0x02,0x04,  0x40,0x40,0x40,0x40,0x40,   // \ ] ^ _
  0x00,0x01,0x02,0x04,0x00,  0x20,0x54,0x54,0x54,0x78,  0x7F,0x48,0x44,0x44,0x38,  0x38,0x44,0x44,0x44,0x20,   // ` a b c
  0x38,0x44,0x44,0x48,0x7F,  0x38,0x54,0x54,0x54,0x18,  0x08,0x7E,0x09,0x01,0x02,  0x08,0x14,0x54,0x54,0x3C,   // d e f g
  0x7F,0x08,0x04,0x04,0x78,  0x00,0x44,0x7D,0x40,0x00,  0x20,0x40,0x44,0x3D,0x00,  0x00,0x7F,0x10,0x28,0x44,   // h i j k
  0x00,0x41,0x7F,0x40,0x00,  0x7C,0x04,0x18,0x04,0x78,  0x7C,0x08,0x04,0x04,0x78,  0x38,0x44,0x44,0x44,0x38,   // l m n o
  0x7C,0x14,0x14,0x14,0x08,  0x08,0x14,0x14,0x18,0x7C,  0x7C,0x08,0x04,0x04,0x08,  0x48,0x54,0x54,0x54,0x20,   // p q r s
  0x04,0x3F,0x44,0x40,0x20,  0x3C,0x40,0x40,0x20,0x7C,  0x1C,0x20,0x40,0x20,0x1C,  0x3C,0x40,0x30,0x40,0x3C,   // t u v w
  0x44,0x28,0x10,0x28,0x44,  0x0C,0x50,0x50,0x50,0x3C,  0x44,0x64,0x54,0x4C,0x44,  0x00,0x08,0x36,0x41,0x00,   // x y z {
  0x00,0x00,0x7F,0x00,0x00,  0x00,0x41,0x36,0x08,0x00,  0x08,0x04,0x08,0x10,0x08                                // | } ~
};

// ═════════ 5. 전역 상태 ═════════
// MPU-6050
bool imu_ok = false;                     // 센서 설정이 끝났는지
bool sleep_check = true;                 // 슬립 비트로 전원 끊김을 감지할지 (깨운 뒤 0으로 읽히는 칩에서만 켬)
unsigned long imu_wake_ms = 0;           // 마지막으로 센서를 깨운 시각
unsigned long imu_retry_ms = 0;          // 마지막으로 다시 설정을 시도한 시각
unsigned long imu_last_good_ms = 0;      // 마지막으로 읽기에 성공한 시각
imu_sample imu_latest;                   // 0.1초마다 읽어 두는 최신 샘플 (loop 전용)

// 자이로 영점·방향 추정
float gyro_bias[3] = { 0.0f, 0.0f, 0.0f };   // 각 축의 영점 오차 [°/s]
bool gyro_bias_ok = false;               // 영점을 한 번이라도 측정했는지
bool gyro_cal_started = false;
bool gyro_cal_gave_up = false;           // 제한 시간 안에 못 잡아서 0으로 진행 중
unsigned long gyro_cal_start_ms = 0;
uint8_t still_count = 0;                 // "가만히" 구간에 쌓인 샘플 수
uint8_t still_far_count = 0;             // 지금 영점과 크게 다른 "가만히" 구간이 연속으로 나온 횟수
float still_sum[3], still_min[3], still_max[3];
float still_acc_min = 0.0f, still_acc_max = 0.0f;
float up_vec[3] = { 0.0f, 0.0f, 1.0f };  // 센서 기준 "위쪽" 방향 (가만히 있을 때 가속도계가 가리키는 방향)
bool up_ok = false;
float heading_deg = 0.0f;                // 지금 바라보는 방향 [°] (0=북, 시계방향)
bool heading_ok = false;                 // heading을 믿을 수 있는지
float yaw_rate_dps = 0.0f;               // 수직축 기준 회전 속도 [°/s] (위에서 볼 때 시계방향 +)
unsigned long heading_last_us = 0;
bool heading_have_last = false;

// GPS
gps_fix gps_real;                        // 실제 GPS 모듈에서 읽은 값
gps_fix gps_mock;                        // 가상 GPS 값
bool gps_use_mock = (GPS_MODE != 0);       // 지금 가상 GPS를 쓰는지
bool gps_auto = (GPS_MODE == 2);          // 자동 모드: GPS 문장이 들어오면 실제 GPS로 바꿈
char gps_line[100];                      // 받는 중인 NMEA 문장 한 줄
uint8_t gps_len = 0;
unsigned long gps_sentence_count = 0;    // 체크섬이 맞은 문장 수 (배선 확인용)
unsigned long gps_last_rx_ms = 0;        // 마지막으로 올바른 문장을 받은 시각
unsigned long gps_course_seq = 0;        // 진행 방향이 새로 들어올 때마다 +1
unsigned long course_seq_used = 0;
uint8_t course_good_count = 0;
unsigned long mock_last_ms = 0;

// 길안내
double target_lat = TARGET_LAT;
double target_lon = TARGET_LON;
uint8_t nav_state = NAV_IMU_ERROR;
bool nav_geo_ok = false;                 // 거리·방위각을 계산했는지
float nav_distance_m = 0.0f;             // 목적지까지 거리 [m]
float nav_bearing_deg = 0.0f;            // 목적지 방위각 [°]
float nav_error_deg = 0.0f;              // bearing − heading, −180~+180 (+: 오른쪽으로 돌기)
bool arrived = false;
uint8_t turn_level = 0;                  // 0 직진, 1 조금, 2 크게, 3 뒤로 (확정된 판단)
int8_t turn_side = 0;                    // −1 왼쪽, +1 오른쪽, 0 해당 없음
bool turn_pending = false;               // 바뀐 판단이 유지되는지 지켜보는 중
uint8_t turn_pending_level = 0;
int8_t turn_pending_side = 0;
unsigned long turn_pending_ms = 0;

// 진동모터
uint8_t motor_mode = MOTOR_OFF;
unsigned long motor_on_ms = 0;
unsigned long motor_period_ms = 0;
uint8_t motor_repeat = 0;                // 0이면 계속 반복
unsigned long motor_start_ms = 0;
bool motor_left_on = false;
bool motor_right_on = false;

// OLED
uint8_t oled_buf[1024];                  // 그리는 중인 화면 (page 8줄 × 128칸)
uint8_t oled_shadow[1024];               // OLED에 이미 보낸 화면
bool oled_page_force[8];                 // true면 비교하지 않고 그 줄 전체를 다시 보냄
bool oled_ok = false;
uint8_t oled_next_page = 0;
#if OLED_USE_SPI
uint8_t oled_spi_buf[128];               // SPI로 보낼 복사본 (아래 oled_spi_write 설명 참고)
#endif
unsigned long oled_retry_ms = 0;
unsigned long oled_refresh_ms = 0;

// Bridge 연결 (이 변수들만 Bridge 함수와 loop가 함께 씀 → 복사할 때 noInterrupts로 보호)
nav_snapshot published;                  // loop → Bridge 함수: 0.1초마다 갱신되는 값 묶음
bridge_request pending;                  // Bridge 함수 → loop: 설정 요청
imu_sample imu_latch;                    // read_imu가 마지막으로 내보낸 샘플 (read_pitch가 같은 샘플을 쓰도록)
unsigned long imu_latch_ms = 0;
bool imu_latch_valid = false;

// 주기
unsigned long last_tick_ms = 0;
unsigned long debug_last_ms = 0;
unsigned long tick_count = 0;

// ═════════ 6. 공용 도우미 ═════════
float wrap_360(float angle) {                      // 0 이상 360 미만으로
  if (!(angle > -100000.0f && angle < 100000.0f)) return 0.0f;
  while (angle >= 360.0f) angle -= 360.0f;
  while (angle < 0.0f) angle += 360.0f;
  return angle;
}

float wrap_180(float angle) {                      // −180 초과 +180 이하로
  if (!(angle > -100000.0f && angle < 100000.0f)) return 0.0f;
  while (angle > 180.0f) angle -= 360.0f;
  while (angle <= -180.0f) angle += 360.0f;
  return angle;
}

int round_to_int(float value) {
  return (int)(value >= 0.0f ? value + 0.5f : value - 0.5f);
}

// 숫자·문자열을 char 버퍼에 이어 쓰는 함수들 (끝 위치를 돌려줌, printf의 %f에 의존하지 않음)
char *put_str(char *p, const char *s) {
  while (*s) *p++ = *s++;
  *p = 0;
  return p;
}

char *put_uint(char *p, unsigned long value) {
  char digits[12];
  uint8_t count = 0;
  do {
    digits[count++] = (char)('0' + value % 10);
    value /= 10;
  } while (value > 0 && count < sizeof(digits));
  while (count > 0) *p++ = digits[--count];
  *p = 0;
  return p;
}

char *put_uint_padded(char *p, unsigned long value, uint8_t width) {   // 앞을 0으로 채움 (예: 053)
  unsigned long limit = 1;
  for (uint8_t i = 1; i < width; i++) limit *= 10;
  while (limit > 1 && value < limit) {
    *p++ = '0';
    limit /= 10;
  }
  return put_uint(p, value);
}

char *put_fixed(char *p, double value, uint8_t decimals) {             // 소수 자릿수를 고정해서 씀 (반올림)
  unsigned long scale = 1;
  for (uint8_t i = 0; i < decimals; i++) scale *= 10;
  bool negative = value < 0.0;
  if (negative) value = -value;
  double scaled = value * (double)scale + 0.5;
  if (!(scaled < 4294967040.0)) return put_str(p, "nan");              // NaN 또는 너무 큰 값
  unsigned long whole = (unsigned long)scaled;
  if (negative && whole != 0) *p++ = '-';
  p = put_uint(p, whole / scale);
  if (decimals > 0) {
    *p++ = '.';
    p = put_uint_padded(p, whole % scale, decimals);
  }
  return p;
}

char *put_hex8(char *p, uint8_t value) {                               // 1바이트를 16진수 두 글자로 (예: 0x68)
  const char *digits = "0123456789ABCDEF";
  *p++ = '0';
  *p++ = 'x';
  *p++ = digits[value >> 4];
  *p++ = digits[value & 0x0F];
  *p = 0;
  return p;
}

char *put_angle(char *p, float angle) {                                // 방위각 0.0 ~ 359.9 (반올림하면 360.0이 되는 값은 0.0으로)
  float wrapped = wrap_360(angle);
  if (wrapped >= 359.95f) wrapped = 0.0f;
  return put_fixed(p, wrapped, 1);
}

bool parse_decimal(const char *s, double *out) {                       // "123.45" → 123.45 (다른 글자가 섞이면 실패)
  bool negative = false;
  bool any_digit = false;
  double whole = 0.0, frac = 0.0, frac_div = 1.0;
  if (*s == '-') {
    negative = true;
    s++;
  } else if (*s == '+') {
    s++;
  }
  while (*s >= '0' && *s <= '9') {
    whole = whole * 10.0 + (double)(*s - '0');
    any_digit = true;
    s++;
  }
  if (*s == '.') {
    s++;
    while (*s >= '0' && *s <= '9') {
      frac = frac * 10.0 + (double)(*s - '0');
      frac_div *= 10.0;
      any_digit = true;
      s++;
    }
  }
  if (!any_digit || *s != 0) return false;
  double value = whole + frac / frac_div;
  *out = negative ? -value : value;
  return true;
}

// ═════════ 7. MPU-6050 ═════════
bool imu_write_reg(uint8_t reg, uint8_t val) {
  IMU_WIRE.beginTransmission(MPU6050_ADDR);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(val);
  return IMU_WIRE.endTransmission() == 0;          // 0 = 센서가 ACK(응답)함
}

bool imu_read_regs(uint8_t reg, uint8_t *buf, size_t len) {
  IMU_WIRE.beginTransmission(MPU6050_ADDR);
  IMU_WIRE.write(reg);                             // 읽기 시작할 레지스터 주소
  if (IMU_WIRE.endTransmission(false) != 0) return false;
  if (IMU_WIRE.requestFrom(MPU6050_ADDR, len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = (uint8_t)IMU_WIRE.read();
  return true;
}

int16_t to_int16(const uint8_t *p) {               // 상위 바이트, 하위 바이트 → 부호 있는 16비트
  return (int16_t)((p[0] << 8) | p[1]);
}

// 슬립 해제 + 측정 설정 (delay 없음)
bool imu_config() {
  bool ok = imu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x01)    // 슬립 해제 + 자이로 PLL 클럭 (응답 없으면 바로 실패)
         && imu_write_reg(MPU6050_REG_CONFIG, 0x06)        // 디지털 LPF 5Hz: 0.1초 주기로 읽어도 회전량이 빠지지 않게
         && imu_write_reg(MPU6050_REG_GYRO_CFG, 0x08)      // ±500 °/s → 65.5 LSB/(°/s)
         && imu_write_reg(MPU6050_REG_ACCEL_CFG, 0x00);    // ±2 g     → 16384 LSB/g
  if (!ok) return false;                           // 하나라도 실패하면 다음에 처음부터 다시

  // 방금 슬립을 풀었으니 슬립 비트는 0으로 읽혀야 정상.
  // 깨운 뒤에도 1로 읽히는 칩(일부 호환 칩)은 이 비트로 전원 끊김을 판단할 수 없으므로 검사를 끔
  uint8_t pwr = 0;
  if (imu_read_regs(MPU6050_REG_PWR_MGMT_1, &pwr, 1)) sleep_check = !(pwr & MPU6050_SLEEP_BIT);

  imu_wake_ms = millis();                          // 안정화 시간 계산 시작
  return true;
}

// 부팅 때 한 번만: 리셋 후 설정 (setup()에서만 부르므로 delay가 있어도 됨)
bool imu_init() {
  if (!imu_write_reg(MPU6050_REG_PWR_MGMT_1, 0x80)) return false;   // 리셋 (응답 없으면 여기서 바로 실패)
  delay(100);                                               // 리셋 완료 대기
  return imu_config();
}

bool fetch_imu_data(imu_sample *out) {
  // 전원이 순간 끊겼다 돌아오면 센서는 슬립 상태(측정값이 멈춤)로 다시 시작함.
  // I2C 응답은 정상이라 읽기가 "성공"해 버리므로, 슬립 비트를 먼저 확인해서 멈춘 값을 걸러냄
  if (sleep_check) {
    uint8_t pwr = 0;
    if (!imu_read_regs(MPU6050_REG_PWR_MGMT_1, &pwr, 1)) return false;
    if (pwr & MPU6050_SLEEP_BIT) return false;
  }

  uint8_t buf[14];
  if (!imu_read_regs(MPU6050_REG_ACCEL_XOUT, buf, 14)) return false;   // 14바이트를 한 번에 → 7개 값이 같은 순간의 값
  out->ax   = to_int16(&buf[0])  / MPU6050_ACCEL_LSB_PER_G;
  out->ay   = to_int16(&buf[2])  / MPU6050_ACCEL_LSB_PER_G;
  out->az   = to_int16(&buf[4])  / MPU6050_ACCEL_LSB_PER_G;
  out->temp = to_int16(&buf[6])  / 340.0f + 36.53f;            // 정품 MPU-6050 공식 (호환 칩은 다를 수 있음)
  out->gx   = to_int16(&buf[8])  / MPU6050_GYRO_LSB_PER_DPS;
  out->gy   = to_int16(&buf[10]) / MPU6050_GYRO_LSB_PER_DPS;
  out->gz   = to_int16(&buf[12]) / MPU6050_GYRO_LSB_PER_DPS;

  // 기울기도 같은 샘플로 계산. 모듈을 평평하게 눕히면 0°, X축 방향으로 기울이면 ±로 변함
  // (붙이는 방향에 따라 앞뒤 기울기가 다른 축에 나타날 수 있음)
  out->pitch = atan2f(-out->ax, sqrtf(out->ay * out->ay + out->az * out->az)) * 57.29578f;   // rad → deg
  return true;
}

// 0.1초마다: 센서를 읽어 imu_latest에 넣어 둠. Bridge 함수들은 이 값만 돌려줌
void imu_tick(unsigned long now) {
  if (!imu_ok) {                                   // 센서가 응답하지 않는 상태
    imu_latest.valid = false;
    if (now - imu_retry_ms >= IMU_RETRY_MS) {      // 1초에 한 번만 다시 설정 시도 (기다리지 않음)
      imu_retry_ms = now;
      imu_ok = imu_config();
    }
    return;
  }
  if (now - imu_wake_ms < IMU_WARMUP_MS) {         // 깨운 직후에는 값이 안정될 때까지 건너뜀
    imu_latest.valid = false;
    return;
  }
  imu_sample fresh;
  if (!fetch_imu_data(&fresh)) {                   // 읽기 실패 또는 전원 끊김 감지
    imu_ok = false;
    imu_retry_ms = now - IMU_RETRY_MS;             // 다음 주기에 바로 다시 설정
    imu_latest.valid = false;
    return;
  }
  fresh.ms = now;
  fresh.valid = true;
  imu_latest = fresh;
}

// ═════════ 8. 방향(heading) 추정 ═════════
bool gyro_ready() {
  return gyro_bias_ok || gyro_cal_gave_up;
}

void still_restart(const float *rate, float acc_mag) {
  still_count = 1;
  for (uint8_t i = 0; i < 3; i++) {
    still_sum[i] = rate[i];
    still_min[i] = rate[i];
    still_max[i] = rate[i];
  }
  still_acc_min = acc_mag;
  still_acc_max = acc_mag;
}

// 가만히 있는 2초 구간을 찾으면 그 평균을 자이로 영점으로 삼음 (부팅 때와 그 뒤 가만히 있을 때마다)
void gyro_bias_update(const imu_sample *s, unsigned long now) {
  if (!gyro_cal_started) {
    gyro_cal_started = true;
    gyro_cal_start_ms = now;
  }
  if (!gyro_bias_ok && now - gyro_cal_start_ms > GYRO_CAL_TIMEOUT_MS) gyro_cal_gave_up = true;

  float rate[3] = { s->gx, s->gy, s->gz };
  float acc_mag = sqrtf(s->ax * s->ax + s->ay * s->ay + s->az * s->az);
  if (still_count == 0) {
    still_restart(rate, acc_mag);
    return;
  }

  bool moving = (acc_mag < 0.90f || acc_mag > 1.10f);        // 중력(1g)과 다르면 움직이는 중
  for (uint8_t i = 0; i < 3; i++) {
    still_sum[i] += rate[i];
    if (rate[i] < still_min[i]) still_min[i] = rate[i];
    if (rate[i] > still_max[i]) still_max[i] = rate[i];
    if (still_max[i] - still_min[i] > STILL_GYRO_SPREAD_DPS) moving = true;
  }
  if (acc_mag < still_acc_min) still_acc_min = acc_mag;
  if (acc_mag > still_acc_max) still_acc_max = acc_mag;
  if (still_acc_max - still_acc_min > STILL_ACCEL_SPREAD_G) moving = true;
  still_count++;

  if (moving) {                                              // 움직였으면 이 샘플부터 다시 셈
    still_restart(rate, acc_mag);
    still_far_count = 0;
    return;
  }
  if (still_count < STILL_TICKS) return;

  // 2초 동안 변동이 없었음 → 평균이 영점 후보
  float mean[3];
  bool plausible = true, near_current = true;
  for (uint8_t i = 0; i < 3; i++) {
    mean[i] = still_sum[i] / (float)still_count;
    float step = mean[i] - gyro_bias[i];
    if (mean[i] > GYRO_BIAS_MAX_DPS || mean[i] < -GYRO_BIAS_MAX_DPS) plausible = false;
    if (step > GYRO_BIAS_STEP_DPS || step < -GYRO_BIAS_STEP_DPS) near_current = false;
  }
  still_count = 0;
  if (!plausible) {                                          // 변동은 없지만 값이 큼 = 일정한 속도로 도는 중 → 영점으로 쓰지 않음
    still_far_count = 0;
    return;
  }
  if (gyro_bias_ok && !near_current && ++still_far_count < 3) return;   // 크게 달라진 값은 6초 연속으로 같아야 믿음
  still_far_count = 0;
  for (uint8_t i = 0; i < 3; i++) gyro_bias[i] = mean[i];
  gyro_bias_ok = true;
}

// 자이로를 적분해서 heading 갱신.
// 센서가 어느 방향으로 붙어 있든, 중력 방향(위쪽) 둘레의 회전만 골라서 씀
void heading_update(const imu_sample *s) {
  unsigned long now_us = micros();
  float acc_mag = sqrtf(s->ax * s->ax + s->ay * s->ay + s->az * s->az);
  if (acc_mag > 0.7f && acc_mag < 1.3f) {                    // 심하게 흔들릴 때의 가속도는 중력 추정에 쓰지 않음
    if (!up_ok) {
      up_vec[0] = s->ax;
      up_vec[1] = s->ay;
      up_vec[2] = s->az;
      up_ok = true;
    } else {
      up_vec[0] += UP_ALPHA * (s->ax - up_vec[0]);
      up_vec[1] += UP_ALPHA * (s->ay - up_vec[1]);
      up_vec[2] += UP_ALPHA * (s->az - up_vec[2]);
    }
  }
  float up_len = sqrtf(up_vec[0] * up_vec[0] + up_vec[1] * up_vec[1] + up_vec[2] * up_vec[2]);
  if (!up_ok || up_len < 0.1f) return;

  float wx = s->gx - gyro_bias[0];
  float wy = s->gy - gyro_bias[1];
  float wz = s->gz - gyro_bias[2];
  // 자이로는 위에서 볼 때 반시계방향이 +, 방위각은 시계방향이 + 이므로 부호를 뒤집음
  float previous_rate = yaw_rate_dps;
  yaw_rate_dps = -(wx * up_vec[0] + wy * up_vec[1] + wz * up_vec[2]) / up_len;

  if (heading_have_last && heading_ok && gyro_ready()) {
    float dt = (float)(now_us - heading_last_us) * 0.000001f;        // 실제로 지난 시간 [s]
    // 직전 값과 지금 값의 평균 × 시간 (사다리꼴 적분)
    if (dt > 0.0f && dt < 0.5f) heading_deg = wrap_360(heading_deg + 0.5f * (previous_rate + yaw_rate_dps) * dt);
  }
  heading_last_us = now_us;
  heading_have_last = true;
}

// ═════════ 9. GPS ═════════
gps_fix *gps_active() {                            // 지금 길안내에 쓰는 위치 (가상 또는 실제)
  return gps_use_mock ? &gps_mock : &gps_real;
}

int hex_value(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  return -1;
}

// "$....*hh" 형식과 체크섬 확인. 맞으면 '*' 자리를 끊어서 본문만 남김
bool nmea_checksum_ok(char *line) {
  if (line[0] != '$') return false;
  uint8_t sum = 0;
  char *p = line + 1;
  while (*p && *p != '*') sum ^= (uint8_t)*p++;
  if (*p != '*') return false;
  int high = hex_value(p[1]);
  if (high < 0) return false;
  int low = hex_value(p[2]);
  if (low < 0) return false;
  if (((high << 4) | low) != sum) return false;
  *p = 0;
  return true;
}

// 쉼표로 나눠 각 칸의 시작 위치를 fields에 넣고 칸 수를 돌려줌
uint8_t nmea_split(char *line, char **fields, uint8_t max_fields) {
  uint8_t count = 0;
  fields[count++] = line;
  while (*line && count < max_fields) {
    if (*line == ',') {
      *line = 0;
      fields[count++] = line + 1;
    }
    line++;
  }
  return count;
}

bool nmea_is_type(const char *header, char a, char b, char c) {      // "$GPRMC", "$GNRMC" 등: 뒤 세 글자로 구분
  return strlen(header) == 6 && header[3] == a && header[4] == b && header[5] == c;
}

// NMEA의 "ddmm.mmmm"(도+분) 형식을 도 단위로
bool nmea_to_degrees(const char *value, const char *hemisphere, double *out) {
  double raw;
  if (!parse_decimal(value, &raw) || raw < 0.0) return false;
  long whole_deg = (long)(raw / 100.0);
  double minutes = raw - (double)whole_deg * 100.0;
  if (minutes >= 60.0) return false;
  double result = (double)whole_deg + minutes / 60.0;
  char h = hemisphere[0];
  if (h == 'S' || h == 'W') result = -result;
  else if (h != 'N' && h != 'E') return false;
  *out = result;
  return true;
}

void gps_set_position(double lat, double lon, unsigned long now) {
  if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0) return;
  gps_real.lat = lat;
  gps_real.lon = lon;
  gps_real.ms = now;
  gps_real.valid = true;
}

// RMC: 0 머리말, 1 시각, 2 상태(A=유효), 3 위도, 4 N/S, 5 경도, 6 E/W, 7 속도(노트), 8 진행 방향
void gps_handle_rmc(char **f, uint8_t count, unsigned long now) {
  if (count < 7) return;
  if (f[2][0] != 'A') {                            // 'V' = 위치를 아직 못 잡음
    gps_real.valid = false;
    return;
  }
  double lat, lon, number;
  if (!nmea_to_degrees(f[3], f[4], &lat) || !nmea_to_degrees(f[5], f[6], &lon)) return;
  gps_set_position(lat, lon, now);
  if (count > 7 && parse_decimal(f[7], &number)) gps_real.speed_mps = (float)(number * 0.514444);
  if (count > 8 && parse_decimal(f[8], &number)) {
    gps_real.course_deg = wrap_360((float)number);
    gps_real.course_valid = true;
  } else {
    gps_real.course_valid = false;                 // 멈춰 있으면 진행 방향 칸이 비어서 옴
  }
  gps_course_seq++;
}

// GGA: 0 머리말, 1 시각, 2 위도, 3 N/S, 4 경도, 5 E/W, 6 품질(0=없음), 7 위성 수
void gps_handle_gga(char **f, uint8_t count, unsigned long now) {
  if (count < 8) return;
  double lat, lon, number;
  if (parse_decimal(f[7], &number) && number >= 0.0 && number < 100.0) gps_real.sats = (uint8_t)number;
  if (f[6][0] == 0 || f[6][0] == '0') {
    gps_real.valid = false;
    return;
  }
  if (!nmea_to_degrees(f[2], f[3], &lat) || !nmea_to_degrees(f[4], f[5], &lon)) return;
  gps_set_position(lat, lon, now);
}

void gps_parse_line(char *line, unsigned long now) {
  if (!nmea_checksum_ok(line)) return;             // 깨진 문장은 버림
  gps_sentence_count++;
  gps_last_rx_ms = now;
  char *fields[16];
  uint8_t count = nmea_split(line, fields, 16);
  if (nmea_is_type(fields[0], 'R', 'M', 'C')) gps_handle_rmc(fields, count, now);
  else if (nmea_is_type(fields[0], 'G', 'G', 'A')) gps_handle_gga(fields, count, now);
}

// loop()가 돌 때마다: 도착해 있는 글자만 처리 (기다리지 않음)
void gps_poll(unsigned long now) {
  uint8_t budget = 96;                             // 한 바퀴에 처리할 최대 글자 수
  while (budget-- > 0 && GPS_SERIAL.available() > 0) {
    char c = (char)GPS_SERIAL.read();
    if (c == '$') {                                // 새 문장 시작
      gps_len = 0;
      gps_line[gps_len++] = c;
    } else if (c == '\r' || c == '\n') {           // 문장 끝
      if (gps_len > 0) {
        gps_line[gps_len] = 0;
        gps_parse_line(gps_line, now);
      }
      gps_len = 0;
    } else if (gps_len > 0) {
      if (gps_len < sizeof(gps_line) - 1) gps_line[gps_len++] = c;
      else gps_len = 0;                            // 너무 긴 문장은 버림
    }
  }
}

void gps_mock_reset(unsigned long now) {
  gps_mock.lat = MOCK_START_LAT;
  gps_mock.lon = MOCK_START_LON;
  gps_mock.speed_mps = 0.0f;
  gps_mock.course_deg = MOCK_START_HEADING;
  gps_mock.course_valid = false;
  gps_mock.sats = 0;
  gps_mock.ms = now;
  gps_mock.valid = true;
  mock_last_ms = now;
  heading_deg = wrap_360(MOCK_START_HEADING);      // 부팅 때 바라보는 방향을 이 값으로 가정
  heading_ok = true;
}

// 0.1초마다: 실제 GPS는 유효시간 검사, 가상 GPS는 "걷기"
void gps_tick(unsigned long now) {
  if (gps_real.valid && now - gps_real.ms > GPS_TIMEOUT_MS) gps_real.valid = false;
  if (!gps_use_mock) return;

  float dt = (float)(now - mock_last_ms) * 0.001f;
  mock_last_ms = now;
  gps_mock.ms = now;
  gps_mock.valid = true;
  gps_mock.speed_mps = 0.0f;
  // 안내 중이고 화살표가 정면(GO)으로 확정된 동안에만 바라보는 방향으로 이동 (책상에서 도착까지 확인하기 위한 가상 걸음)
  if (nav_state == NAV_GUIDE && turn_level == 0 && !turn_pending && heading_ok && dt > 0.0f && dt < 1.0f) {
    double step_m = (double)MOCK_SPEED_MPS * (double)dt;
    double heading_rad = (double)heading_deg * RAD_PER_DEG;
    gps_mock.lat += step_m * cos(heading_rad) / METERS_PER_DEG;
    gps_mock.lon += step_m * sin(heading_rad) / (METERS_PER_DEG * cos(gps_mock.lat * RAD_PER_DEG));
    gps_mock.speed_mps = MOCK_SPEED_MPS;
  }
}

// 걷는 중이면 GPS 진행 방향으로 heading을 시작·보정 (실제 GPS일 때만)
void heading_fuse_course() {
  if (gps_use_mock) return;
  if (gps_course_seq == course_seq_used) return;   // 새 진행 방향이 들어왔을 때만
  course_seq_used = gps_course_seq;
  if (!gps_real.valid || !gps_real.course_valid || gps_real.speed_mps < COURSE_MIN_SPEED_MPS) {
    course_good_count = 0;                         // 느리거나 멈춰 있으면 진행 방향을 믿지 않음
    return;
  }
  if (!heading_ok) {
    if (++course_good_count >= COURSE_INIT_COUNT) {
      heading_deg = gps_real.course_deg;           // 처음: 걷는 방향 = 바라보는 방향으로 시작
      heading_ok = true;
    }
  } else {
    heading_deg = wrap_360(heading_deg + COURSE_GAIN * wrap_180(gps_real.course_deg - heading_deg));
  }
}

// ═════════ 10. 진동모터 ═════════
void motor_write(bool left_on, bool right_on) {
  if (left_on != motor_left_on) {
    digitalWrite(MOTOR_LEFT_PIN, (left_on != (MOTOR_ACTIVE_LOW != 0)) ? HIGH : LOW);
    motor_left_on = left_on;
  }
  if (right_on != motor_right_on) {
    digitalWrite(MOTOR_RIGHT_PIN, (right_on != (MOTOR_ACTIVE_LOW != 0)) ? HIGH : LOW);
    motor_right_on = right_on;
  }
}

// 패턴 지정. 같은 패턴을 다시 지정하면 박자를 유지하고, 달라지면 바로 첫 진동부터 시작
void motor_set_pattern(uint8_t mode, unsigned long on_ms, unsigned long period_ms, uint8_t repeat, unsigned long now) {
  if (mode == motor_mode && on_ms == motor_on_ms && period_ms == motor_period_ms && repeat == motor_repeat) return;
  motor_mode = mode;
  motor_on_ms = on_ms;
  motor_period_ms = period_ms;
  motor_repeat = repeat;
  motor_start_ms = now;
}

// loop()가 돌 때마다: 지금 시각에 맞춰 핀을 켜고 끔 (기다리지 않음)
void motor_update(unsigned long now) {
  bool left = false, right = false;
  if (motor_mode != MOTOR_OFF && motor_period_ms > 0) {
    unsigned long elapsed = now - motor_start_ms;
    unsigned long cycle = elapsed / motor_period_ms;
    unsigned long phase = elapsed % motor_period_ms;
    bool running = (motor_repeat == 0 || cycle < motor_repeat);
    bool first_half = running && phase < motor_on_ms;
    bool second_half = running && phase >= motor_period_ms / 2 && phase < motor_period_ms / 2 + motor_on_ms;
    if (motor_mode == MOTOR_LEFT) left = first_half;
    else if (motor_mode == MOTOR_RIGHT) right = first_half;
    else if (motor_mode == MOTOR_BOTH) left = right = first_half;
    else if (motor_mode == MOTOR_ALTERNATE) {
      left = first_half;
      right = second_half;
    }
  }
  motor_write(left, right);
}

// 판단 결과 → 진동 패턴
void motor_select(unsigned long now) {
  if (nav_state == NAV_ARRIVED) return;            // 도착 진동은 도착한 순간 한 번만 시작됨 (nav_tick)
  if (nav_state != NAV_GUIDE || turn_level == 0) {
    motor_set_pattern(MOTOR_OFF, 0, 0, 0, now);    // 정면이거나 안내 중이 아니면 끔
    return;
  }
  if (turn_level == 3) {
    motor_set_pattern(MOTOR_ALTERNATE, VIB_BACK_ON_MS, VIB_BACK_PERIOD_MS, 0, now);
    return;
  }
  uint8_t mode = (turn_side < 0) ? MOTOR_LEFT : MOTOR_RIGHT;     // 돌아야 하는 쪽 모터
  if (turn_level == 1) motor_set_pattern(mode, VIB_SLIGHT_ON_MS, VIB_SLIGHT_PERIOD_MS, 0, now);
  else motor_set_pattern(mode, VIB_HARD_ON_MS, VIB_HARD_PERIOD_MS, 0, now);
}

// ═════════ 11. 길안내 판단 ═════════
// 현재 위치에서 목적지까지의 거리와 방위각 (리눅스 navigation.py와 같은 공식)
void nav_compute_geometry(double lat, double lon) {
  double phi_a = lat * RAD_PER_DEG;
  double phi_b = target_lat * RAD_PER_DEG;
  double d_phi = phi_b - phi_a;
  double d_lambda = (target_lon - lon) * RAD_PER_DEG;

  double sin_phi = sin(d_phi / 2.0);
  double sin_lambda = sin(d_lambda / 2.0);
  double a = sin_phi * sin_phi + cos(phi_a) * cos(phi_b) * sin_lambda * sin_lambda;
  if (a > 1.0) a = 1.0;
  nav_distance_m = (float)(2.0 * EARTH_RADIUS_M * atan2(sqrt(a), sqrt(1.0 - a)));

  double east = sin(d_lambda) * cos(phi_b);
  double north = cos(phi_a) * sin(phi_b) - sin(phi_a) * cos(phi_b) * cos(d_lambda);
  nav_bearing_deg = wrap_360((float)(atan2(east, north) / RAD_PER_DEG));
  nav_geo_ok = true;
}

// |angle error|로 이탈 단계 결정. 경계에서 왔다 갔다 하지 않도록 지금 단계를 기준으로 여유(HYST_DEG)를 둠
uint8_t turn_level_for(float abs_error, uint8_t current) {
  const float limit[3] = { ALIGN_DEG, TURN_DEG, BACK_DEG };
  uint8_t level = current;
  while (level < 3 && abs_error > limit[level] + HYST_DEG) level++;
  while (level > 0 && abs_error < limit[level - 1] - HYST_DEG) level--;
  return level;
}

void turn_reset() {
  turn_level = 0;
  turn_side = 0;
  turn_pending = false;
}

// 방향 이탈 판단: 바뀐 단계가 일정 시간 유지돼야 확정
void turn_decide(unsigned long now) {
  float abs_error = nav_error_deg < 0.0f ? -nav_error_deg : nav_error_deg;
  uint8_t level = turn_level_for(abs_error, turn_level);
  int8_t side = 0;
  if (level == 1 || level == 2) side = (nav_error_deg < 0.0f) ? -1 : 1;   // 뒤로 돌기(3)는 좌우 구분 없음

  if (level == turn_level && side == turn_side) {
    turn_pending = false;
    return;
  }
  if (!turn_pending || level != turn_pending_level || side != turn_pending_side) {
    turn_pending = true;
    turn_pending_level = level;
    turn_pending_side = side;
    turn_pending_ms = now;
  }
  unsigned long hold = (level < turn_level) ? TURN_RELEASE_MS : TURN_HOLD_MS;
  if (now - turn_pending_ms >= hold) {
    turn_level = level;
    turn_side = side;
    turn_pending = false;
  }
}

int turn_code() {                                  // read_nav·디버그 출력용: 0, ±1, ±2, 3
  if (nav_state != NAV_GUIDE) return 0;
  if (turn_level == 3) return 3;
  return (int)turn_level * (int)turn_side;
}

// 0.1초마다: 상태 결정 → 거리·방위각 → 이탈 판단
void nav_tick(unsigned long now) {
  gps_fix *fix = gps_active();
  uint8_t state;
  if (!imu_latest.valid) {
    state = NAV_IMU_ERROR;
  } else if (!gyro_ready()) {
    state = NAV_CALIBRATING;
  } else if (!fix->valid) {
    state = NAV_NO_FIX;
    nav_geo_ok = false;
    arrived = false;
  } else {
    nav_compute_geometry(fix->lat, fix->lon);
    if (!arrived && nav_distance_m <= ARRIVE_RADIUS_M) arrived = true;
    else if (arrived && nav_distance_m > ARRIVE_EXIT_M) arrived = false;
    if (arrived) state = NAV_ARRIVED;
    else if (!heading_ok) state = NAV_NO_HEADING;
    else state = NAV_GUIDE;
  }

  if (state == NAV_GUIDE) {
    nav_error_deg = wrap_180(nav_bearing_deg - heading_deg);
    turn_decide(now);
  } else {
    turn_reset();
  }
  if (state == NAV_ARRIVED && nav_state != NAV_ARRIVED) {        // 도착한 순간: 양쪽 3번
    motor_set_pattern(MOTOR_BOTH, VIB_ARRIVE_ON_MS, VIB_ARRIVE_PERIOD_MS, VIB_ARRIVE_REPEAT, now);
  }
  nav_state = state;
}

// ═════════ 12. OLED (SSD1309 / SSD1306 / SH1106, 라이브러리 없이 SPI 또는 Wire로 직접 구동) ═════════
// ── 전송: 이 부분만 SPI / I2C에 따라 다름. 나머지(그리기·화면 갱신)는 공통 ──
#if OLED_USE_SPI
// DC 핀으로 명령(LOW)/데이터(HIGH)를 구분해서 보냄.
// UNO Q의 SPI.transfer(버퍼, 길이)는 보낸 버퍼를 받은 값으로 덮어쓰므로 화면 버퍼를 직접 넘기지 않고 복사본을 보냄
void oled_spi_write(bool is_data, const uint8_t *bytes, uint8_t count) {
  memcpy(oled_spi_buf, bytes, count);
  digitalWrite(OLED_DC_PIN, is_data ? HIGH : LOW);
  SPI.beginTransaction(SPISettings(OLED_SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(OLED_CS_PIN, LOW);
  SPI.transfer(oled_spi_buf, count);
  digitalWrite(OLED_CS_PIN, HIGH);
  SPI.endTransaction();
}

// setup()에서만: 핀 설정 + 하드웨어 리셋 (delay가 있어도 됨)
void oled_hw_setup() {
  SPI.begin();
  pinMode(OLED_CS_PIN, OUTPUT);                    // SPI.begin() 뒤에 설정해야 CS를 직접 제어할 수 있음
  digitalWrite(OLED_CS_PIN, HIGH);
  pinMode(OLED_DC_PIN, OUTPUT);
  pinMode(OLED_RST_PIN, OUTPUT);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(1);
  digitalWrite(OLED_RST_PIN, LOW);                 // RES를 잠깐 LOW로 → 칩 초기화
  delay(10);
  digitalWrite(OLED_RST_PIN, HIGH);
  delay(10);
}
#else
void oled_hw_setup() {}                            // I2C는 Wire.begin()만 하면 됨 (setup의 ②)
#endif

// 명령 여러 개를 보냄. 성공하면 true
bool oled_command_list(const uint8_t *commands, uint8_t count) {
#if OLED_USE_SPI
  oled_spi_write(false, commands, count);
  return true;                                     // SPI는 응답(ACK)이 없어서 연결이 끊겨도 알 수 없음
#else
  OLED_WIRE.beginTransmission(OLED_ADDR);
  OLED_WIRE.write((uint8_t)0x00);                  // 컨트롤 바이트 0x00: 뒤따르는 바이트는 모두 명령
  for (uint8_t i = 0; i < count; i++) OLED_WIRE.write(commands[i]);
  return OLED_WIRE.endTransmission() == 0;         // 0 = OLED가 ACK(응답)함
#endif
}

// 화면 데이터를 보냄 (count ≤ OLED_CHUNK). 성공하면 true
bool oled_data(const uint8_t *bytes, uint8_t count) {
#if OLED_USE_SPI
  oled_spi_write(true, bytes, count);
  return true;
#else
  OLED_WIRE.beginTransmission(OLED_ADDR);
  OLED_WIRE.write((uint8_t)0x40);                  // 컨트롤 바이트 0x40: 뒤따르는 바이트는 화면 데이터
  OLED_WIRE.write(bytes, count);
  return OLED_WIRE.endTransmission() == 0;
#endif
}

// display_off_first = true: 부팅 때 (화면을 껐다가 설정 후 켬) / false: 동작 중 재설정 (깜빡임 없음)
bool oled_send_config(bool display_off_first) {
  static const uint8_t config[] = {
    0xAE,                                          // 화면 끄기 (부팅 때만 보냄)
#if OLED_CHIP == 1309
    0xFD, 0x12,                                    // 명령 잠금 해제 (SSD1309, 혹시 잠겨 있어도 아래 명령이 먹도록)
    0xD5, 0xA0,                                    // 표시 클럭 (SSD1309 권장값)
#else
    0xD5, 0x80,                                    // 표시 클럭
#endif
    0xA8, 0x3F,                                    // 세로 64줄
    0xD3, 0x00,                                    // 세로 오프셋 없음
    0x40,                                          // 시작 줄 0
#if OLED_CHIP == 1106
    0xAD, 0x8B,                                    // 내부 승압 회로 켜기 (SH1106)
#elif OLED_CHIP == 1306
    0x8D, 0x14,                                    // 내부 승압 회로 켜기 (SSD1306)
    0x20, 0x02,                                    // 줄(page) 단위 주소 모드
    0x2E,                                          // 스크롤 끄기
#else
    // SSD1309에는 내부 승압 회로가 없음 → 승압은 모듈 보드의 12.5V 회로가 함 (명령 없음)
    0x20, 0x02,                                    // 줄(page) 단위 주소 모드
    0x2E,                                          // 스크롤 끄기
#endif
    (uint8_t)(OLED_FLIP_H ? 0xA0 : 0xA1),          // 좌우 방향
    (uint8_t)(OLED_FLIP_V ? 0xC0 : 0xC8),          // 상하 방향
    0xDA, 0x12,                                    // 패널 배선 방식 (128x64)
#if OLED_CHIP == 1309
    0x81, 0x6F,                                    // 밝기       ─┐
    0xD9, 0xD3,                                    // 프리차지    │ U8g2 라이브러리의 SSD1309 128x64 값
    0xDB, 0x20,                                    // VCOMH      ─┘
#else
    0x81, 0xCF,                                    // 밝기
    0xD9, 0xF1,                                    // 프리차지
    0xDB, 0x40,                                    // VCOMH
#endif
    0xA4,                                          // RAM 내용을 표시
    0xA6,                                          // 정상 표시 (반전 아님)
    0xAF                                           // 화면 켜기
  };
  uint8_t skip = display_off_first ? 0 : 1;
  return oled_command_list(config + skip, (uint8_t)(sizeof(config) - skip));
}

bool oled_init() {
  if (!oled_send_config(true)) return false;
  for (uint8_t i = 0; i < OLED_PAGES; i++) oled_page_force[i] = true;   // 화면 전체를 한 번 다 보냄
  oled_refresh_ms = millis();
  return true;
}

// 한 줄(page)의 first~last 칸을 OLED로 보냄
bool oled_send_span(uint8_t page, uint8_t first, uint8_t last) {
  uint8_t column = (uint8_t)(first + OLED_COLUMN_OFFSET);
  uint8_t position[3] = { (uint8_t)(0xB0 | page), (uint8_t)(column & 0x0F), (uint8_t)(0x10 | (column >> 4)) };
  if (!oled_command_list(position, 3)) return false;

  uint16_t offset = (uint16_t)(page * 128 + first);
  uint8_t remaining = (uint8_t)(last - first + 1);
  while (remaining > 0) {
    uint8_t chunk = remaining > OLED_CHUNK ? OLED_CHUNK : remaining;
    if (!oled_data(&oled_buf[offset], chunk)) return false;
    memcpy(&oled_shadow[offset], &oled_buf[offset], chunk);
    offset = (uint16_t)(offset + chunk);
    remaining = (uint8_t)(remaining - chunk);
  }
  return true;
}

// loop()가 돌 때마다: 바뀐 줄을 하나만 찾아 바뀐 칸만 보냄 → 한 번에 오래 걸리지 않음
// 보낸 것이 있으면 true
bool oled_flush_step() {
  if (!oled_ok) return false;
  for (uint8_t i = 0; i < OLED_PAGES; i++) {
    uint8_t page = (uint8_t)((oled_next_page + i) % OLED_PAGES);
    const uint8_t *now_row = &oled_buf[page * 128];
    const uint8_t *sent_row = &oled_shadow[page * 128];
    int first = -1, last = -1;
    if (oled_page_force[page]) {
      first = 0;
      last = 127;
    } else {
      for (int x = 0; x < 128; x++) {
        if (now_row[x] != sent_row[x]) {
          if (first < 0) first = x;
          last = x;
        }
      }
    }
    if (first < 0) continue;                       // 이 줄은 바뀐 게 없음
    if (oled_send_span(page, (uint8_t)first, (uint8_t)last)) oled_page_force[page] = false;
    else oled_ok = false;                          // 응답 없음 → oled_maintain이 다시 초기화
    oled_next_page = (uint8_t)((page + 1) % OLED_PAGES);
    return true;
  }
  return false;
}

void oled_flush_all() {                            // setup()에서만: 화면 전체를 바로 보냄
  for (uint8_t i = 0; i < 2 * OLED_PAGES; i++) {
    if (!oled_flush_step()) break;
  }
}

// 0.1초마다: 응답이 없으면 다시 초기화, 주기적으로 설정을 다시 보내 스스로 복구
void oled_maintain(unsigned long now) {
  if (!oled_ok) {
    if (now - oled_retry_ms >= OLED_RETRY_MS) {
      oled_retry_ms = now;
      oled_ok = oled_init();
    }
    return;
  }
  if (now - oled_refresh_ms >= OLED_REFRESH_MS) {
    oled_refresh_ms = now;
    if (!oled_send_config(false)) {
      oled_ok = false;
      return;
    }
    for (uint8_t i = 0; i < OLED_PAGES; i++) oled_page_force[i] = true;
  }
}

// ── 그리기 (oled_buf에만 그림. 실제 전송은 oled_flush_step) ──
void oled_clear() {
  memset(oled_buf, 0, sizeof(oled_buf));
}

void oled_pixel(int x, int y) {
  if (x < 0 || x >= 128 || y < 0 || y >= 64) return;
  oled_buf[(y >> 3) * 128 + x] |= (uint8_t)(1 << (y & 7));
}

void oled_hline(int x_from, int x_to, int y) {
  if (x_from > x_to) {
    int swap = x_from;
    x_from = x_to;
    x_to = swap;
  }
  for (int x = x_from; x <= x_to; x++) oled_pixel(x, y);
}

void oled_fill_rect(int x, int y, int w, int h) {
  for (int row = y; row < y + h; row++) oled_hline(x, x + w - 1, row);
}

long divide_rounded(long numerator, long denominator) {                 // 반올림 나눗셈 (denominator > 0)
  if (numerator >= 0) return (numerator + denominator / 2) / denominator;
  return -((-numerator + denominator / 2) / denominator);
}

void oled_fill_triangle(int xa, int ya, int xb, int yb, int xc, int yc) {
  int swap;
  if (ya > yb) { swap = ya; ya = yb; yb = swap; swap = xa; xa = xb; xb = swap; }   // 위에서 아래 순서로 정렬
  if (yb > yc) { swap = yb; yb = yc; yc = swap; swap = xb; xb = xc; xc = swap; }
  if (ya > yb) { swap = ya; ya = yb; yb = swap; swap = xa; xa = xb; xb = swap; }
  for (int y = ya; y <= yc; y++) {
    long x_long = (yc == ya) ? xa : xa + divide_rounded((long)(xc - xa) * (y - ya), yc - ya);   // 긴 변 (a→c)
    long x_short;
    if (y < yb) x_short = xa + divide_rounded((long)(xb - xa) * (y - ya), yb - ya);            // 위쪽 변 (a→b)
    else if (yc == yb) x_short = xb;
    else x_short = xb + divide_rounded((long)(xc - xb) * (y - yb), yc - yb);                   // 아래쪽 변 (b→c)
    oled_hline((int)x_long, (int)x_short, y);
  }
}

void oled_char(int x, int y, char c, uint8_t scale) {
  if (c < 0x20 || c > 0x7E) c = '?';
  const uint8_t *glyph = &FONT_5X7[(uint8_t)(c - 0x20) * 5];
  for (uint8_t col = 0; col < 5; col++) {
    for (uint8_t row = 0; row < 7; row++) {
      if (glyph[col] & (1 << row)) oled_fill_rect(x + col * scale, y + row * scale, scale, scale);
    }
  }
}

int oled_text_width(const char *s, uint8_t scale) {
  int length = (int)strlen(s);
  return length > 0 ? length * 6 * scale - scale : 0;
}

void oled_text(int x, int y, const char *s, uint8_t scale) {
  while (*s) {
    oled_char(x, y, *s++, scale);
    x += 6 * scale;
  }
}

void oled_text_center(int center_x, int y, const char *s, uint8_t scale) {
  oled_text(center_x - oled_text_width(s, scale) / 2, y, s, scale);
}

// 위(정면)를 가리키는 화살표를 시계방향으로 angle_deg만큼 돌려서 그림
void oled_arrow(int center_x, int center_y, float angle_deg) {
  // 화살표 모양의 꼭짓점 (x: 오른쪽 +, y: 위쪽 +). 0~2 머리, 3~6 몸통 (몸통 위쪽은 머리와 조금 겹침)
  static const int8_t shape[7][2] = { { 0, 27 }, { -17, 3 }, { 17, 3 }, { -6, 7 }, { 6, 7 }, { 6, -25 }, { -6, -25 } };
  float angle_rad = angle_deg * 0.017453293f;
  float sin_a = sinf(angle_rad);
  float cos_a = cosf(angle_rad);
  int px[7], py[7];
  for (uint8_t i = 0; i < 7; i++) {
    float x = (float)shape[i][0];
    float y = (float)shape[i][1];
    px[i] = center_x + round_to_int(x * cos_a + y * sin_a);
    py[i] = center_y - round_to_int(-x * sin_a + y * cos_a);     // 화면은 아래쪽이 +
  }
  oled_fill_triangle(px[0], py[0], px[1], py[1], px[2], py[2]);
  oled_fill_triangle(px[3], py[3], px[4], py[4], px[5], py[5]);
  oled_fill_triangle(px[3], py[3], px[5], py[5], px[6], py[6]);
}

void format_distance(char *out, float meters) {    // "242m", "1.2km", "35km"
  char *p = out;
  if (meters < 999.5f) {
    p = put_uint(p, (unsigned long)(meters + 0.5f));
    put_str(p, "m");
  } else if (meters < 9950.0f) {
    p = put_fixed(p, meters / 1000.0f, 1);
    put_str(p, "km");
  } else if (meters < 999500.0f) {
    p = put_uint(p, (unsigned long)(meters / 1000.0f + 0.5f));
    put_str(p, "km");
  } else {
    put_str(p, "FAR");
  }
}

// 0.1초마다: 상태에 맞는 화면을 oled_buf에 그림
void draw_frame(unsigned long now) {
  char text[32];
  char *p;
  oled_clear();

  if (nav_state == NAV_IMU_ERROR) {
    oled_text_center(64, 12, "IMU ERROR", 2);
    oled_text_center(64, 38, "CHECK MPU-6050 WIRING", 1);
  } else if (nav_state == NAV_CALIBRATING) {
    oled_text_center(64, 12, "HOLD STILL", 2);
    oled_text_center(64, 38, "GYRO CALIBRATION", 1);
  } else if (nav_state == NAV_NO_FIX) {
    oled_text_center(64, 12, "NO GPS FIX", 2);
    if (gps_sentence_count > 0 && now - gps_last_rx_ms < GPS_TIMEOUT_MS) {
      p = put_str(text, "NMEA OK  SAT ");          // 문장은 들어오는데 위치를 아직 못 잡음 (실내에서는 정상)
      put_uint(p, gps_real.sats);
      oled_text_center(64, 38, text, 1);
    } else {
      oled_text_center(64, 34, "NO DATA FROM GPS", 1);   // 문장이 안 들어옴 → 배선·속도 확인
      oled_text_center(64, 44, "CHECK GPS TX - D0", 1);
    }
  } else if (nav_state == NAV_ARRIVED) {
    oled_text_center(64, 10, "ARRIVED", 2);
    p = put_str(text, "DIST ");
    format_distance(p, nav_distance_m);
    oled_text_center(64, 36, text, 1);
  } else {
    // 왼쪽 64x64: 화살표 (방향을 모르면 '?')
    if (nav_state == NAV_GUIDE) oled_arrow(31, 32, 5.0f * (float)round_to_int(nav_error_deg / 5.0f));
    else oled_char(17, 11, '?', 6);

    // 오른쪽: 거리 / 안내 / heading / bearing
    format_distance(text, nav_distance_m);
    oled_text(68, 1, text, 2);

    if (nav_state != NAV_GUIDE) {
      put_str(text, "WALK");                       // 걸어야 GPS 진행 방향으로 heading을 알 수 있음
    } else if (turn_level == 0) {
      put_str(text, "GO");
    } else if (turn_level == 3) {
      put_str(text, "BACK");
    } else {
      unsigned long abs_error = (unsigned long)round_to_int(nav_error_deg < 0.0f ? -nav_error_deg : nav_error_deg);
      p = put_str(text, turn_side < 0 ? "L" : "R");
      if (abs_error < 100) p = put_str(p, " ");
      put_uint(p, abs_error);
    }
    oled_text(68, 19, text, 2);

    p = put_str(text, "HDG ");
    if (heading_ok) p = put_uint_padded(p, (unsigned long)round_to_int(heading_deg) % 360, 3);
    else p = put_str(p, "---");
    if (!gyro_bias_ok) put_str(p, "?");            // 자이로 영점을 못 잡은 상태 (방향이 흐를 수 있음)
    oled_text(68, 38, text, 1);

    p = put_str(text, "BRG ");
    put_uint_padded(p, (unsigned long)round_to_int(nav_bearing_deg) % 360, 3);
    oled_text(68, 47, text, 1);
  }

  // 맨 아랫줄: GPS 종류
  if (gps_use_mock) {
    put_str(text, "MOCK GPS");
  } else {
    p = put_str(text, "GPS S");
    put_uint_padded(p, gps_real.sats, 2);
  }
  if (nav_state == NAV_GUIDE || nav_state == NAV_NO_HEADING) oled_text(68, 56, text, 1);
  else oled_text_center(64, 56, text, 1);
}

// ═════════ 13. Bridge 연결 ═════════
// Bridge 함수는 loop()와 다른 스레드에서, loop() 도중에 끼어들어 실행될 수 있음
// (Arduino_RPClite 0.3.0부터는 provide_safe로 등록해도 Bridge 스레드가 처리하는 경우가 있음)
// 그래서 Bridge 함수는 센서·길안내 변수를 직접 건드리지 않음
//   읽기: loop()가 0.1초마다 통째로 복사해 두는 published만 읽음
//   쓰기: pending에 요청만 적어 둠 → loop()가 다음 바퀴에 반영
// 복사하는 아주 짧은 순간에만 noInterrupts()로 끼어들기를 막음

// loop 쪽: 0.1초마다 현재 값을 published에 복사
void snapshot_publish() {
  gps_fix *fix = gps_active();
  nav_snapshot s;
  s.imu = imu_latest;
  s.fix_valid = fix->valid;
  s.lat = fix->lat;
  s.lon = fix->lon;
  s.sats = gps_real.sats;
  s.state = nav_state;
  s.turn = (int8_t)turn_code();
  s.heading_ok = heading_ok;
  s.geo_ok = nav_geo_ok;
  s.mock = gps_use_mock;
  s.cal = gyro_bias_ok;
  s.error_deg = nav_error_deg;
  s.heading_deg = heading_deg;
  s.bearing_deg = nav_bearing_deg;
  s.distance_m = nav_distance_m;
  noInterrupts();
  published = s;
  interrupts();
}

// 가상 GPS ↔ 실제 GPS 전환 (자동 전환과 set_gps_mock이 함께 씀)
void gps_set_mock(bool on, unsigned long now) {
  if (on == gps_use_mock) return;
  gps_use_mock = on;
  arrived = false;
  turn_reset();
  course_good_count = 0;
  course_seq_used = gps_course_seq;
  if (gps_use_mock) gps_mock_reset(now);           // 가상 위치·방향을 처음 값으로
  else heading_ok = false;                         // 실제 GPS: 걷기 시작해야 방향을 알 수 있음 (가상으로 가정한 방향은 버림)
}

// 0.1초마다: 자동 모드에서 GPS 문장이 들어왔으면 실제 GPS로 바꿈
void gps_auto_select(unsigned long now) {
  if (!gps_auto || gps_sentence_count == 0) return;
  gps_auto = false;                                // 한 번 바꾸면 재부팅 전까지 유지 (실측 도중 가상으로 돌아가지 않게)
  if (gps_use_mock) {
    gps_set_mock(false, now);
    Monitor.print("GPS module detected on D0 -> real GPS mode\n");
  }
}

// loop 쪽: Bridge로 들어온 설정 요청을 꺼내서 반영. 반영한 것이 있으면 true
bool bridge_apply_requests(unsigned long now) {
  noInterrupts();
  bridge_request request = pending;
  pending.target_set = false;
  pending.heading_set = false;
  pending.mock_set = false;
  interrupts();

  bool applied = false;
  if (request.target_set) {
    target_lat = request.target_lat;
    target_lon = request.target_lon;
    arrived = false;                               // 새 목적지 → 도착 판정 다시
    applied = true;
  }
  if (request.mock_set) {
    gps_auto = false;                              // 리눅스가 정하면 그 값으로 고정 (자동 전환 끝)
    gps_set_mock(request.mock_on, now);
    applied = true;
  }
  if (request.heading_set) {
    heading_deg = request.heading_deg;
    heading_ok = true;
    applied = true;
  }
  return applied;
}

// Bridge 쪽: published를 통째로 복사해 옴
void snapshot_read(nav_snapshot *out) {
  noInterrupts();
  *out = published;
  interrupts();
}

bool sample_fresh(const imu_sample *sample) {
  return sample->valid && (millis() - sample->ms <= SAMPLE_STALE_MS);
}

bool imu_ready() {
  nav_snapshot s;
  snapshot_read(&s);
  return sample_fresh(&s.imu);
}

String read_imu() {
  nav_snapshot s;
  snapshot_read(&s);
  if (!sample_fresh(&s.imu)) return String("");
  unsigned long now = millis();
  noInterrupts();
  imu_latch = s.imu;                               // read_pitch가 같은 샘플을 쓰도록 기억
  imu_latch_ms = now;
  imu_latch_valid = true;
  interrupts();

  char text[96];
  char *p = text;
  p = put_fixed(p, s.imu.ax, 3);   *p++ = ',';
  p = put_fixed(p, s.imu.ay, 3);   *p++ = ',';
  p = put_fixed(p, s.imu.az, 3);   *p++ = ',';
  p = put_fixed(p, s.imu.gx, 2);   *p++ = ',';
  p = put_fixed(p, s.imu.gy, 2);   *p++ = ',';
  p = put_fixed(p, s.imu.gz, 2);   *p++ = ',';
  p = put_fixed(p, s.imu.temp, 2);
  return String(text);
}

float read_pitch() {
  nav_snapshot s;
  snapshot_read(&s);
  unsigned long now = millis();
  noInterrupts();
  bool use_latch = imu_latch_valid && (now - imu_latch_ms <= PITCH_LATCH_MS);
  float latched_pitch = imu_latch.pitch;
  interrupts();
  // read_imu 직후라면 그때 내보낸 샘플의 pitch → 한 행의 7개 값과 pitch가 항상 같은 시점
  if (use_latch) return latched_pitch;
  if (!sample_fresh(&s.imu)) return ERR_VALUE;
  return s.imu.pitch;
}

String read_gps() {
  nav_snapshot s;
  snapshot_read(&s);
  if (!s.fix_valid) return String("");
  char text[40];
  char *p = text;
  p = put_fixed(p, s.lat, 6);   *p++ = ',';
  p = put_fixed(p, s.lon, 6);
  return String(text);
}

// "ms,state,turn,error,heading,bearing,dist,lat,lon,sats,mock,cal" (모르는 값은 빈칸)
String read_nav() {
  nav_snapshot s;
  snapshot_read(&s);
  char text[160];
  char *p = text;
  p = put_uint(p, millis());                                        *p++ = ',';
  p = put_str(p, NAV_STATE_NAME[s.state]);                          *p++ = ',';
  if (s.turn < 0) *p++ = '-';
  p = put_uint(p, (unsigned long)(s.turn < 0 ? -s.turn : s.turn));  *p++ = ',';
  if (s.state == NAV_GUIDE) p = put_fixed(p, s.error_deg, 1);
  *p++ = ',';
  if (s.heading_ok) p = put_angle(p, s.heading_deg);
  *p++ = ',';
  if (s.geo_ok) p = put_angle(p, s.bearing_deg);
  *p++ = ',';
  if (s.geo_ok) p = put_fixed(p, s.distance_m, 1);
  *p++ = ',';
  if (s.fix_valid) p = put_fixed(p, s.lat, 6);
  *p++ = ',';
  if (s.fix_valid) p = put_fixed(p, s.lon, 6);
  *p++ = ',';
  p = put_uint(p, s.sats);                                          *p++ = ',';
  p = put_uint(p, s.mock ? 1 : 0);                                  *p++ = ',';
  p = put_uint(p, s.cal ? 1 : 0);
  return String(text);
}

bool set_target(double lat, double lon) {
  if (!(lat >= -90.0 && lat <= 90.0 && lon >= -180.0 && lon <= 180.0)) return false;
  noInterrupts();
  pending.target_lat = lat;
  pending.target_lon = lon;
  pending.target_set = true;
  interrupts();
  return true;
}

bool set_heading(double heading) {
  if (!(heading > -100000.0 && heading < 100000.0)) return false;
  float wrapped = wrap_360((float)heading);
  noInterrupts();
  pending.heading_deg = wrapped;
  pending.heading_set = true;
  interrupts();
  return true;
}

bool set_gps_mock(bool on) {
  noInterrupts();
  pending.mock_on = on;
  pending.mock_set = true;
  interrupts();
  return on;
}

// ═════════ 14. 실행 순서 ═════════
// Monitor(App Lab 콘솔)에 상태 한 줄. 한 줄을 통째로 만들어 한 번에 보냄
void print_status(unsigned long now) {
  if (DEBUG_PRINT_MS == 0 || now - debug_last_ms < DEBUG_PRINT_MS) return;
  debug_last_ms = now;
  gps_fix *fix = gps_active();
  char text[200];
  char *p = text;
  p = put_str(p, "[nav] ");
  p = put_str(p, NAV_STATE_NAME[nav_state]);
  p = put_str(p, " turn=");
  int turn = turn_code();
  if (turn < 0) *p++ = '-';
  p = put_uint(p, (unsigned long)(turn < 0 ? -turn : turn));
  p = put_str(p, " err=");
  if (nav_state == NAV_GUIDE) p = put_fixed(p, nav_error_deg, 1);
  p = put_str(p, " hdg=");
  if (heading_ok) p = put_angle(p, heading_deg);
  p = put_str(p, " brg=");
  if (nav_geo_ok) p = put_angle(p, nav_bearing_deg);
  p = put_str(p, " dist=");
  if (nav_geo_ok) p = put_fixed(p, nav_distance_m, 1);
  p = put_str(p, gps_use_mock ? " gps=MOCK" : (fix->valid ? " gps=FIX" : " gps=NOFIX"));
  p = put_str(p, " sats=");
  p = put_uint(p, gps_real.sats);
  p = put_str(p, " nmea=");
  p = put_uint(p, gps_sentence_count);
  p = put_str(p, " yaw=");
  p = put_fixed(p, yaw_rate_dps, 1);
  p = put_str(p, " cal=");
  p = put_uint(p, gyro_bias_ok ? 1 : 0);
  p = put_str(p, imu_latest.valid ? " imu=OK" : " imu=FAIL");
#if OLED_USE_SPI
  p = put_str(p, " oled=SPI");
#else
  p = put_str(p, oled_ok ? " oled=OK" : " oled=FAIL");
#endif
  put_str(p, "\n");
  Monitor.print(text);
}

// 0.1초마다 실행되는 한 주기
void run_tick(unsigned long now) {
  imu_tick(now);                                   // ① MPU-6050 읽기
  if (imu_latest.valid) {
    imu_last_good_ms = now;
    gyro_bias_update(&imu_latest, now);            // ② 가만히 있으면 자이로 영점 측정
    heading_update(&imu_latest);                   // ③ 자이로 적분 → heading
  } else {
    heading_have_last = false;                     // 못 읽은 구간은 적분하지 않음
    still_count = 0;
    still_far_count = 0;
    if (!gps_use_mock && heading_ok && now - imu_last_good_ms > HEADING_LOST_MS) heading_ok = false;
  }
  gps_auto_select(now);                            // ④ 자동 모드: GPS 문장이 들어왔으면 실제 GPS로
  gps_tick(now);                                   //    GPS 유효시간 검사 / 가상 GPS 걷기
  heading_fuse_course();                           // ⑤ 걷는 중이면 GPS 진행 방향으로 heading 보정
  nav_tick(now);                                   // ⑥ 거리·방위각 계산, 방향 이탈 판단
  motor_select(now);                               // ⑦ 판단 결과 → 진동 패턴
  draw_frame(now);                                 // ⑧ 화면 그리기 (전송은 loop에서 조금씩)
  oled_maintain(now);                              // ⑨ OLED 재연결·주기적 재설정
  snapshot_publish();                              // ⑩ Bridge 함수가 읽을 값 갱신
  tick_count++;
}

void setup() {
  // ① 진동모터 핀: 가장 먼저 꺼 둠 (리눅스 부팅을 기다리는 동안 울리지 않게)
  pinMode(MOTOR_LEFT_PIN, OUTPUT);
  pinMode(MOTOR_RIGHT_PIN, OUTPUT);
  digitalWrite(MOTOR_LEFT_PIN, (MOTOR_ACTIVE_LOW != 0) ? HIGH : LOW);
  digitalWrite(MOTOR_RIGHT_PIN, (MOTOR_ACTIVE_LOW != 0) ? HIGH : LOW);

  // ② I2C 시작, OLED 준비 후 시작 화면
  IMU_WIRE.begin();
#if !OLED_USE_SPI
  OLED_WIRE.begin();                               // 같은 버스(Wire)면 두 번 불러도 문제없음
#endif
#if I2C_CLOCK_HZ > 0
  IMU_WIRE.setClock(I2C_CLOCK_HZ);
#if !OLED_USE_SPI
  OLED_WIRE.setClock(I2C_CLOCK_HZ);
#endif
#endif
  oled_hw_setup();                                 // SPI: 핀 설정 + 리셋 / I2C: 할 일 없음
  oled_ok = oled_init();
  oled_clear();
  oled_text_center(64, 6, "SMART", 2);
  oled_text_center(64, 26, "GOGGLES", 2);
  oled_text_center(64, 52, "WAITING FOR LINUX", 1);
  oled_flush_all();

  // ③ Bridge: 리눅스 쪽 arduino-router가 준비될 때까지 기다림
  Bridge.begin();
  Monitor.begin(115200);

  // ④ MPU-6050 리셋 후 설정 (delay는 여기서만)
  imu_ok = imu_init();

  // ⑤ GPS UART 시작, 가상 GPS 준비
  GPS_SERIAL.begin(GPS_BAUD);
  if (gps_use_mock) gps_mock_reset(millis());

  // ⑥ 함수 등록: 리눅스에서 이 이름으로 호출함 (함수들은 published / pending 만 쓰므로 어느 스레드에서 실행돼도 안전)
  Bridge.provide_safe("imu_ready",    imu_ready);
  Bridge.provide_safe("read_imu",     read_imu);
  Bridge.provide_safe("read_pitch",   read_pitch);
  Bridge.provide_safe("read_gps",     read_gps);
  Bridge.provide_safe("read_nav",     read_nav);
  Bridge.provide_safe("set_target",   set_target);
  Bridge.provide_safe("set_heading",  set_heading);
  Bridge.provide_safe("set_gps_mock", set_gps_mock);

  // ⑦ 부팅 결과 출력
  char text[160];
  char *p = text;
  if (imu_ok) {
    uint8_t who_am_i = 0;
    imu_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1);
    p = put_str(p, "MPU-6050 OK - I2C addr ");
    p = put_hex8(p, MPU6050_ADDR);
    p = put_str(p, ", chip ID(WHO_AM_I) ");                // 정품 0x68, 호환 칩은 0x72 등 (코드는 이 값을 검사하지 않음)
    p = put_hex8(p, who_am_i);
    p = put_str(p, sleep_check ? ", power-glitch check ON\n" : ", power-glitch check OFF\n");
  } else {
    p = put_str(p, "MPU-6050 NOT FOUND - check SDA/SCL header pins (not A4/A5) / AD0\n");
  }
  Monitor.print(text);
#if OLED_USE_SPI
  p = put_str(text, "OLED: SPI (DC D7, RST D8, CS D10) - no ACK, check the screen");
#else
  p = put_str(text, oled_ok ? "OLED: I2C OK" : "OLED: I2C NOT FOUND (check address 0x3C/0x3D)");
#endif
#if GPS_MODE == 2
  p = put_str(p, ", GPS: AUTO (mock until NMEA arrives on D0)\n");
#elif GPS_MODE == 1
  p = put_str(p, ", GPS: MOCK (virtual position)\n");
#else
  p = put_str(p, ", GPS: real module on D0\n");
#endif
  Monitor.print(text);

  last_tick_ms = millis() - TICK_MS;               // 첫 주기를 바로 실행
}

void loop() {
  unsigned long now = millis();
  gps_poll(now);                                   // ① GPS 글자 처리 (도착한 것만)
  if (bridge_apply_requests(now)) last_tick_ms = now - TICK_MS;   // ② Bridge 설정 요청 반영 → 바로 한 주기 실행
  if (now - last_tick_ms >= TICK_MS) {             // ③ 0.1초마다: 센서 읽기 → 판단 → 화면 그리기
    last_tick_ms += TICK_MS;
    if (now - last_tick_ms >= TICK_MS) last_tick_ms = now;   // 많이 밀렸으면 몰아서 하지 않고 지금부터 다시
    run_tick(now);
  }
  motor_update(millis());                          // ④ 진동 박자에 맞춰 핀 켜고 끄기
  oled_flush_step();                               // ⑤ 바뀐 화면을 한 번에 한 줄씩 전송
  print_status(now);                                // ⑥ 1초마다 상태 한 줄 출력
  // ※ delay()를 넣지 말 것: loop가 한 바퀴 돌 때마다 Bridge 호출이 처리되므로 그만큼 응답이 늦어짐
}
