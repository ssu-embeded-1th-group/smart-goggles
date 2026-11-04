
from arduino.app_utils import *
import csv
import time
from datetime import datetime

CSV_FILE = "imu_log.csv"
INTERVAL = 0.5

last_time = None

with open(CSV_FILE, "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)

    writer.writerow([
        "timestamp", "interval_s", "status",
        "ax", "ay", "az",
        "gx", "gy", "gz",
        "temp", "pitch", "error"
    ])

    def loop():
        global last_time

        now = time.monotonic()
        interval = "" if last_time is None else round(now - last_time, 3)
        last_time = now

        timestamp = datetime.now().astimezone().isoformat(
            timespec="milliseconds"
        )

        try:
            if not Bridge.call("imu_ready"):
                raise RuntimeError("MPU-6050 응답 없음")

            data = Bridge.call("read_imu")
            pitch = Bridge.call("read_pitch")

            if not data:
                raise RuntimeError("센서 데이터 읽기 실패")

            values = list(map(float, data.split(",")))

            if len(values) != 7:
                raise ValueError("센서 데이터 형식 오류")

            writer.writerow([
                timestamp, interval, "ok",
                *values, pitch, ""
            ])

        except Exception as e:
            writer.writerow([
                timestamp, interval, "error",
                "", "", "", "", "", "", "", "",
                str(e)
            ])
            print("데이터 수집 오류:", e)

        f.flush()
        time.sleep(INTERVAL)


    print("IMU 데이터 기록 시작")
    App.run(user_loop=loop)