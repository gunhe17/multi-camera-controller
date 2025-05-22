import csv

def load_logs(filepath):
    logs = []

    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            logs.append({
                "frame_index": int(row["frame_index"]),
                "timestamp_100ns": int(row["timestamp_100ns"]),
                "is_null_sample": row["is_null_sample"] == "true"
            })

    return logs

def compute_deltas(logs):
    result = []

    for i in range(len(logs)):
        log = logs[i]
        delta_ms = 0.0

        if i > 0:
            prev = logs[i - 1]
            delta_100ns = log["timestamp_100ns"] - prev["timestamp_100ns"]
            delta_ms = delta_100ns / 10_000  # 100ns → ms

        result.append({
            "frame_index": log["frame_index"],
            "timestamp_100ns": log["timestamp_100ns"],
            "is_null_sample": log["is_null_sample"],
            "delta_ms": round(delta_ms, 3)
        })

    return result

def save_to_csv(logs_with_deltas, output_file="lab/lab3/result/delta_log.csv"):
    with open(output_file, 'w', newline='', encoding='utf-8') as f:
        writer = csv.DictWriter(f, fieldnames=["frame_index", "timestamp_100ns", "is_null_sample", "delta_ms"])
        writer.writeheader()
        writer.writerows(logs_with_deltas)

    print(f"[Info] CSV 저장 완료: {output_file}")

def main():
    input_file = "lab/lab3/result/log_2nd-2.csv"  # 입력 파일 경로
    logs = load_logs(input_file)

    if len(logs) < 2:
        print("기록이 충분하지 않습니다.")
        return

    logs_with_deltas = compute_deltas(logs)
    save_to_csv(logs_with_deltas)

if __name__ == "__main__":
    main()
