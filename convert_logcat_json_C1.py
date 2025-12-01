import json
import csv
import re
import argparse
import logging
from datetime import datetime
from typing import Any, Dict


def load_json_any(path: str) -> Any:
    """Try loading a JSON file with multiple common encodings."""
    for enc in ("utf-8", "utf-8-sig", "utf-16", "utf-16-le", "utf-16-be"):
        try:
            with open(path, "r", encoding=enc) as f:
                return json.load(f)
        except (UnicodeDecodeError, json.JSONDecodeError):
            continue
        except FileNotFoundError:
            logging.error(f"File not found: {path}")
            raise
    raise UnicodeError("Could not decode file with common encodings.")


def extract_motor_data(message: str) -> tuple[str, str, str, str, str] | None:
    """Extract motor data using regex."""
    num = r"[+-]?(?:\d+(?:\.\d+)?|\.\d+)(?:[eE][+-]?\d+)?"
    pattern = re.compile(
        rf"Target\s*=\s*({num})\s+Vel\s*=\s*({num})\s+V\s*=\s*({num})\s+A\s*=\s*({num})\s+Error\s*=\s*({num})"
    )
    match = pattern.search(message)
    return match.groups() if match else None


def process_logcat(input_file: str, output_file: str) -> None:
    """Process the JSON logcat file and write filtered FTC data to CSV."""
    data = load_json_any(input_file)

    if "logcatMessages" not in data:
        logging.error("Invalid JSON format: 'logcatMessages' key not found.")
        return

    with open(output_file, "w", newline="") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow(["date", "time", "pid", "tid", "Target", "Vel", "V", "A", "Error"])

        for entry in data["logcatMessages"]:
            header = entry.get("header", {})
            if header.get("tag") != "FTC":
                continue

            timestamp = header.get("timestamp")
            if not timestamp:
                continue

            ts_sec = timestamp.get("seconds", 0)
            ts_nano = timestamp.get("nanos", 0)
            dt = datetime.fromtimestamp(ts_sec + ts_nano / 1e9)
            date_str = dt.date().isoformat()
            time_str = dt.strftime("%H:%M:%S.%f")  # microsecond precision

            message = entry.get("message", "")
            motor_data = extract_motor_data(message)

            if motor_data:
                writer.writerow([
                    date_str,
                    time_str,
                    header.get("pid", ""),
                    header.get("tid", ""),
                    *motor_data
                ])

    logging.info(f"Filtered FTC log saved to: {output_file}")


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description="Filter FTC logcat messages and extract motor data to CSV."
    )
    parser.add_argument("input", help="Path to input logcat JSON file")
    parser.add_argument("output", help="Path to output CSV file")
    return parser.parse_args()


def main():
    logging.basicConfig(level=logging.INFO, format="%(levelname)s: %(message)s")
    args = parse_args()
    try:
        process_logcat(args.input, args.output)
    except Exception as e:
        logging.error(f"Processing failed: {e}")


if __name__ == "__main__":
    main()
