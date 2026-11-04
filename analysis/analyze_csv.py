import csv
import statistics
import sys

CSV_FILE = sys.argv[1] if len(sys.argv) > 1 else "analysis/imu_log.csv" # 파일 경로

with open(CSV_FILE, "r", newline="", encoding="utf-8") as f:
    rows = list(csv.DictReader(f))

if not rows:
    print("데이터가 없습니다.")
    raise SystemExit

print("===== CSV 분석 결과 =====")
print("파일:", CSV_FILE)
print("전체 기록:", len(rows))

# 상태별 기록 수
status_count = {}

for row in rows:
    status = row["status"]
    status_count[status] = status_count.get(status, 0) + 1

print("\n[상태별 기록]")
for status, count in status_count.items():
    print(f"{status}: {count}")

# 기록 간격 분석
intervals = []

for row in rows:
    try:
        value = float(row["interval_s"])
        if value > 0:
            intervals.append(value)
    except (ValueError, TypeError):
        pass

if intervals:
    print("\n[기록 간격]")
    print(f"평균: {statistics.mean(intervals):.3f}초")
    print(f"최소: {min(intervals):.3f}초")
    print(f"최대: {max(intervals):.3f}초")

    long_intervals = [x for x in intervals if x > 0.75]
    print(f"0.75초 초과 간격: {len(long_intervals)}개")

# 센서값 분석
columns = ["ax", "ay", "az", "gx", "gy", "gz", "temp", "pitch"]

print("\n[센서값 분석]")

for column in columns:
    values = []

    for row in rows:
        if row["status"] != "ok":
            continue

        try:
            values.append(float(row[column]))
        except (ValueError, TypeError):
            pass

    if values:
        print(
            f"{column:5s} "
            f"min={min(values):8.3f} "
            f"max={max(values):8.3f} "
            f"avg={statistics.mean(values):8.3f}"
        )
    else:
        print(f"{column}: 유효한 데이터 없음")