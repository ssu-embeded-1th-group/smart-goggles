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
 * ── Bridge로 제공하는 함수 (전부 인자 없음, Python: Bridge.call("이름")) ──
 *   imu_ready  → bool   : 센서가 I2C로 응답하면 true
 *   read_imu   → String : "ax,ay,az,gx,gy,gz,temp"  (g, °/s, °C) / 실패 시 ""
 *   read_pitch → float  : 가속도로 계산한 앞뒤 기울기(°, -90~90) / 실패 시 -999
 */

#include <Arduino_RouterBridge.h>
#include <Wire.h>

// 센서를 꽂은 위치에 맞게 이 한 줄만 바꾸면 됨
//   디지털 헤더 SDA/SCL → Wire   |   Qwiic 커넥터 → Wire1   |   A4/A5 → Wire2
#define IMU_WIRE Wire

const uint8_t MPU_ADDR  = 0x68;          // AD0 = HIGH(3.3V)면 0x69
const float   ERR_VALUE = -999.0f;       // read_pitch 실패 시 반환값

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

bool imu_ok = false;                     // 초기화 성공 여부

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
  if (!imu_ok) imu_ok = imu_init();                // 부팅 때 실패했으면 호출될 때마다 다시 시도
  return imu_ok;
}

String read_imu() {
  imu_data d;
  if (!imu_ready() || !fetch_imu_data(&d)) {
    imu_ok = false;                                // 읽기 실패 → 다음 호출 때 재초기화
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
    return ERR_VALUE;
  }
  // 모듈을 평평하게 눕혀 놓으면 0°, X축 방향으로 기울이면 ±로 변함
  // (붙이는 방향에 따라 앞뒤 기울기가 다른 축에 나타날 수 있음)
  return atan2f(-d.ax, sqrtf(d.ay * d.ay + d.az * d.az)) * 57.29578f;   // rad → deg
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
}

void loop() {
  // ⑥ 비워 둠. 리눅스에서 Bridge.call이 오면 loop()가 한 바퀴 돌 때마다 위 함수가 실행됨
  //    (여기에 긴 delay()를 넣으면 그만큼 응답이 늦어짐)
}
