import json
import csv
from datetime import datetime
import re

input_file = "logcat.logcat"
output_file = "logcat_ftc.csv"

# Load JSON
with open(input_file, "r") as f:
    data = json.load(f)

# Prepare CSV
with open(output_file, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    
    # Write CSV header
    writer.writerow(["date", "time", "pid", "tid", "Phase", "Target", "Vel", "V", "A","Error"])
    
    # Regex to extract motor data
    pattern = re.compile(r"Phase=(\w+)\s+Target=([\d\.]+)\s+Vel=([\d\.]+)\s+V=([\d\.]+)\s+A=([\d\.]+)\s+Error=([\d\.]+)")
    
    for entry in data.get("logcatMessages", []):
        header = entry["header"]
        if header.get("tag") != "FTC":
            continue  # skip non-FTC entries
        
        ts_sec = header["timestamp"]["seconds"]
        ts_nano = header["timestamp"]["nanos"]
        dt = datetime.fromtimestamp(ts_sec + ts_nano / 1e9)
        date_str = dt.date().isoformat()
        # Force full microseconds display
        time_str = dt.strftime("%H:%M:%S.%f")
        
        message = entry.get("message", "")
        match = pattern.search(message)
        if match:
            Phase, Target, Vel, V, A, Error = match.groups()
            writer.writerow([date_str, time_str, header["pid"], header["tid"], Phase, Target, Vel, V, A,  Error])

print(f"Filtered FTC log saved to {output_file}")
