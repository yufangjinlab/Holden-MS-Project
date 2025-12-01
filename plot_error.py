import pandas as pd
import matplotlib.pyplot as plt
import os

def plot_error_from_log():
    # Automatically open log_cat.csv in the same directory as the script
    csv_path = os.path.join(os.path.dirname(__file__), "log_cat.csv")

    # Read the CSV (auto-detect tab or comma separator)
    df = pd.read_csv(csv_path, sep=None, engine="python")

    # Combine date and time into one timestamp column
    df["timestamp"] = pd.to_datetime(
        df["date"].astype(str) + " " + df["time"].astype(str),
        errors="coerce",
        infer_datetime_format=True
    )

    # Convert Error column to numeric
    df["Error"] = pd.to_numeric(df["Error"], errors="coerce")

    # Drop any rows with missing timestamp or error values
    df = df.dropna(subset=["timestamp", "Error"]).sort_values("timestamp")

    # Plot Error vs timestamp
    plt.figure()
    plt.plot(df["timestamp"], df["Error"])
    plt.xlabel("Time")
    plt.ylabel("Error")
    plt.title("Error vs Time")
    plt.gcf().autofmt_xdate()
    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    plot_error_from_log()
