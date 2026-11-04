import csv
import time
import math
import random
import os
from datetime import datetime

CSV_FILE = "analysis/imu_log.csv"
INTERVAL = 0.5
DURATION = 24

random.seed(42)


def noise(scale=0.01):
    return random.gauss(0, scale)


def get_sensor_data(t):
    # 0~8초: 평평하게 정지
    if t < 8:
        ax = noise()
        ay = noise()
        az = 1.0 + noise()
        gx = noise(0.5)
        gy = noise(0.5)
        gz = noise(0.5)

    # 8~16초: 약 30도 기울임
    elif t < 16:
        ax = 0.5 + noise()
        ay = noise()
        az = 0.866 + noise()
        gx = noise(0.5)
        gy = noise(0.5)
        gz = noise(0.5)

    # 16~24초: 움직이는 상태
    else:
        ax = 0.5 * math.sin(t * 2) + noise()
        ay = 0.3 * math.cos(t * 2) + noise()
        az = 0.85 + noise()
        gx = 30 * math.sin(t * 2) + noise(1)
        gy = 20 * math.cos(t * 2) + noise(1)
        gz = 10 * math.sin(t) + noise(1)

    temp = 25.0 + 0.1 * math.sin(t / 5) + noise(0.02)

    pitch = math.degrees(
        math.atan2(-ax, math.sqrt(ay * ay + az * az))
    )

    return ax, ay, az, gx, gy, gz, temp, pitch


with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)

    writer.writerow([
        "timestamp", "interval_s", "status",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "temp", "pitch", "error"
    ])

    start = time.monotonic()
    last = None
    count = 0

    print("가상 센서 데이터 기록 시작")

    while True:
        now = time.monotonic()
        elapsed = now - start

        if elapsed >= DURATION:
            break

        interval = "" if last is None else round(now - last, 3)
        last = now

        timestamp = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )

        # 20번째 기록은 오류 상황을 가정
        if count == 20:
            writer.writerow([
                timestamp, interval, "simulated_error",
                "", "", "", "", "", "", "", "",
                "Simulated sensor failure"
            ])
            print("가상 센서 오류 기록")
        else:
            values = get_sensor_data(elapsed)

            writer.writerow([
                timestamp, interval, "ok",
                *[round(v, 3) for v in values],
                ""
            ])

            print(
                f"[{timestamp}] "
                f"ax={values[0]:+.2f} "
                f"ay={values[1]:+.2f} "
                f"az={values[2]:+.2f} "
                f"pitch={values[7]:+.2f}"
            )

        f.flush()
        count += 1

        time.sleep(INTERVAL)

print(f"\n기록 완료: {os.path.abspath(CSV_FILE)}")
print(f"전체 기록: {count}개")