import csv
import re

# Regex for logcat line format
log_pattern = re.compile(r"^(\d{2}-\d{2}) (\d{2}:\d{2}:\d{2}\.\d{3})\s+(\d+)\s+(\d+)\s+([VDIWEF])\s+(\S+): (.*)$")

with open("logcat2.txt", "r", encoding="utf-8") as infile, open("logcat2.csv", "w", newline="", encoding="utf-8") as outfile:
    writer = csv.writer(outfile)
    writer.writerow(["Date", "Time", "PID", "TID", "Level", "Tag", "Message"])  # CSV header
    
    for line in infile:
        match = log_pattern.match(line)
        if match:
            writer.writerow(match.groups())
