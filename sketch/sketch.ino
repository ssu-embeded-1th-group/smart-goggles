/*
 * sketch.ino  —  Arduino UNO Q (MCU 쪽, STM32U585)
 * MPU-6050(GY-521) 1개를 I2C로 읽어서 Bridge로 리눅스(Python)에 넘겨주는 최소 예제
 * 코드 컨벤션: snake_case, 절차지향 (클래스 없이 함수 + 전역 상태)
 *
 * ── 배선 ─────────────────────────────────────────────
 *   GY-521 VCC → 3.3V        GND → GND
 *   SDA → 디지털 헤더 SDA (AREF 옆)    SCL → 디지털 헤더 SCL
 *   AD0 → 연결 안 함(또는 GND) = I2C 주소 0x68
 *   ※ 우노 Q는 A4/A5가 헤더 SDA/SCL과 "다른" I2C 버스 → 아래 IMU_WIRE 참고
 *
 * ── Bridge로 제공하는 함수 (Python: Bridge.call("이름", ...)) ──
 *   imu_ready  → bool   : 센서가 I2C로 응답하면 true
 *   read_imu   → String : "ax,ay,az,gx,gy,gz,temp"  (g, °/s, °C) / 실패 시 ""
 *   read_pitch → float  : 가속도로 계산한 앞뒤 기울기(°, -90~90) / 실패 시 -999
 *   read_heading → float: 진북 기준 시계방향 방위각(0~360 미만) / 기준 없음: -999
 *   set_heading(heading, bias_z) → bool: 현재 자세의 알려진 방위각/자이로 바이어스 설정
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>
#include <math.h>

// 센서를 꽂은 위치에 맞게 이 한 줄만 바꾸면 됨
//   디지털 헤더 SDA/SCL → Wire   |   Qwiic 커넥터 → Wire1   |   A4/A5 → Wire2
#define IMU_WIRE Wire

const uint8_t MPU_ADDR  = 0x68;          // AD0 = HIGH(3.3V)면 0x69
const float   ERR_VALUE = -999.0f;       // pitch 실패 또는 heading 기준 없음

// MPU-6050 레지스터 주소
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;   // 여기부터 14바이트: 가속도 XYZ(6) + 온도(2) + 자이로 XYZ(6)
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

// 센서 값 한 묶음
struct imu_data {
  float ax, ay, az;                      // 가속도 [g]
  float gx, gy, gz;                      // 각속도 [°/s]
  float temp;                            // 온도 [°C]
};

// Arduino의 자동 함수 원형 생성 시 사용자 정의 타입이 선언보다 앞서지 않도록 명시한다.
bool fetch_imu_data(imu_data *out);

bool imu_ok = false;                     // 초기화 성공 여부
bool heading_valid = false;
float current_heading = 0.0f;            // heading_valid일 때만 의미가 있음
float gyro_z_bias = 0.0f;
float previous_gz = 0.0f;
uint32_t last_heading_us = 0;
const uint32_t HEADING_PERIOD_US = 10000; // 100 Hz (Linux 기록 주기와 독립)

// Z축이 위를 향할 때 MPU 오른손 법칙의 +gz는 반시계 방향이다.
// 방위각은 시계 방향이 양수이므로 -1. 반대로 부착했다면 +1로 변경한다.
const float GYRO_TO_HEADING_SIGN = -1.0f;

// ───────── I2C 헬퍼 ─────────
bool write_reg(uint8_t reg, uint8_t val) {
  IMU_WIRE.beginTransmission(MPU_ADDR);
  IMU_WIRE.write(reg);
  IMU_WIRE.write(val);
  return IMU_WIRE.endTransmission() == 0;          // 0 = 센서가 ACK(응답)함
}

bool read_regs(uint8_t reg, uint8_t *buf, size_t len) {
  IMU_WIRE.beginTransmission(MPU_ADDR);
  IMU_WIRE.write(reg);                             // 읽기 시작할 레지스터 주소
  if (IMU_WIRE.endTransmission(false) != 0) return false;
  if (IMU_WIRE.requestFrom(MPU_ADDR, len) != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = IMU_WIRE.read();
  return true;
}

int16_t to_int16(const uint8_t *p) {               // 상위 바이트, 하위 바이트 → 부호 있는 16비트
  return (int16_t)((p[0] << 8) | p[1]);
}

// ───────── MPU-6050 초기화 / 읽기 ─────────
bool imu_init() {
  if (!write_reg(REG_PWR_MGMT_1, 0x80)) return false;   // 리셋 (응답 없으면 여기서 바로 실패)
  delay(100);
  if (!write_reg(REG_PWR_MGMT_1, 0x01)) return false;   // 슬립 해제 + 자이로 PLL 클럭 사용
  delay(10);
  write_reg(REG_CONFIG, 0x03);                          // 디지털 LPF 약 44Hz (노이즈 감소)
  write_reg(REG_GYRO_CONFIG, 0x00);                     // ±250 °/s → 131 LSB/(°/s)
  write_reg(REG_ACCEL_CONFIG, 0x00);                    // ±2 g     → 16384 LSB/g
  return true;
}

bool fetch_imu_data(imu_data *out) {
  uint8_t buf[14];
  if (!read_regs(REG_ACCEL_XOUT_H, buf, 14)) return false;
  out->ax   = to_int16(&buf[0])  / 16384.0f;
  out->ay   = to_int16(&buf[2])  / 16384.0f;
  out->az   = to_int16(&buf[4])  / 16384.0f;
  out->temp = to_int16(&buf[6])  / 340.0f + 36.53f;
  out->gx   = to_int16(&buf[8])  / 131.0f;
  out->gy   = to_int16(&buf[10]) / 131.0f;
  out->gz   = to_int16(&buf[12]) / 131.0f;
  return true;
}

// ───────── Bridge로 노출하는 함수들 ─────────
bool imu_ready() {
  if (!imu_ok) {
    heading_valid = false;                        // 센서 재초기화 후 과거 방위각 재사용 금지
    imu_ok = imu_init();
  }
  return imu_ok;
}

String read_imu() {
  imu_data d;
  if (!imu_ready() || !fetch_imu_data(&d)) {
    imu_ok = false;                                // 읽기 실패 → 다음 호출 때 재초기화
    heading_valid = false;
    return "";
  }
  String out;
  out.reserve(64);
  out += String(d.ax, 3);   out += ',';
  out += String(d.ay, 3);   out += ',';
  out += String(d.az, 3);   out += ',';
  out += String(d.gx, 2);   out += ',';
  out += String(d.gy, 2);   out += ',';
  out += String(d.gz, 2);   out += ',';
  out += String(d.temp, 2);
  return out;
}

float read_pitch() {
  imu_data d;
  if (!imu_ready() || !fetch_imu_data(&d)) {
    imu_ok = false;
    heading_valid = false;
    return ERR_VALUE;
  }
  // 모듈을 평평하게 눕혀 놓으면 0°, X축 방향으로 기울이면 ±로 변함
  // (붙이는 방향에 따라 앞뒤 기울기가 다른 축에 나타날 수 있음)
  return atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * 57.29578f;   // rad → deg
}

bool set_heading(float heading, float bias_z) {
  if (!isfinite(heading) || heading < 0.0f || heading >= 360.0f || !isfinite(bias_z)) {
    return false;
  }
  imu_data d;
  if (!imu_ready() || !fetch_imu_data(&d)) {
    imu_ok = false;
    heading_valid = false;
    return false;
  }
  current_heading = heading;
  gyro_z_bias = bias_z;
  previous_gz = d.gz - gyro_z_bias;
  last_heading_us = micros();
  heading_valid = true;
  return true;
}

void update_heading() {
  if (!heading_valid) return;
  uint32_t now = micros();
  uint32_t elapsed = now - last_heading_us;       // micros() 오버플로에도 대응
  if (elapsed < HEADING_PERIOD_US) return;
  // 긴 통신/센서 중단 중의 회전은 복원할 수 없으므로 다시 기준을 요청한다.
  if (elapsed > 250000) {
    heading_valid = false;
    return;
  }
  imu_data d;
  if (!imu_ok || !fetch_imu_data(&d)) {
    imu_ok = false;
    heading_valid = false;
    return;
  }
  float gz = d.gz - gyro_z_bias;
  float delta = GYRO_TO_HEADING_SIGN * (previous_gz + gz) * 0.5f * (elapsed / 1000000.0f);
  current_heading = fmodf(current_heading + delta, 360.0f);
  if (current_heading < 0.0f) current_heading += 360.0f;
  previous_gz = gz;
  last_heading_us = now;
}

float read_heading() {
  update_heading();
  return heading_valid ? current_heading : ERR_VALUE;
}

// ───────── 실행 순서 ─────────
void setup() {
  Bridge.begin();                // ① 리눅스 쪽 arduino-router가 준비될 때까지 기다림
  Monitor.begin(115200);         // ② 디버그 출력 (App Lab 시리얼 모니터)

  IMU_WIRE.begin();              // ③ I2C 시작
  imu_ok = imu_init();           // ④ MPU-6050 깨우고 설정

  if (imu_ok) {
    uint8_t who_am_i = 0;
    read_regs(REG_WHO_AM_I, &who_am_i, 1);   // 정품은 0x68 (호환 칩은 다른 값일 수 있음)
    Monitor.print("MPU-6050 OK, WHO_AM_I = 0x");
    Monitor.println(who_am_i, HEX);
  } else {
    Monitor.println("MPU-6050 NOT FOUND - check wiring / IMU_WIRE / address");
  }

  // ⑤ 함수 등록: 리눅스에서 이 이름으로 호출함
  //    provide_safe → loop() 사이사이(메인 스레드)에서 실행되므로 Wire(I2C)를 써도 안전
  Bridge.provide_safe("imu_ready",  imu_ready);
  Bridge.provide_safe("read_imu",   read_imu);
  Bridge.provide_safe("read_pitch", read_pitch);
  Bridge.provide_safe("read_heading", read_heading);
  Bridge.provide_safe("set_heading", set_heading);
}

void loop() {
  // ⑥ MCU가 실제 센서 주기로 방향을 추적한다. 긴 delay()를 넣지 않는다.
  update_heading();
}
