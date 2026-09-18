#!/usr/bin/env python3
import pandas as pd

# ─── EDIT THESE ────────────────────────────────────────────────
INPUT_CSV  = '/home/mrlam/colcon_ws/bagfiles_ros2/hils/June_07/e4tow2/ublox_gps_node_fix.csv'
OUTPUT_CSV = '/home/mrlam/colcon_ws/bagfiles_ros2/hils/June_07/e4tow2/ublox_gps_node_fixed.csv'
# ────────────────────────────────────────────────────────────────

def main():
    # 1) Try to load just latitude, longitude, altitude
    # Sometimes there are spaces in column names, so let's handle that
    try:
        df = pd.read_csv(
            INPUT_CSV,
            usecols=['latitude', 'longitude', 'altitude'],
            dtype={
                'latitude': float,
                'longitude': float,
                'altitude': float
            }
        )
    except ValueError:
        # If columns have leading/trailing spaces, handle that
        df = pd.read_csv(INPUT_CSV)
        # Normalize column names (remove spaces)
        df.columns = [c.strip() for c in df.columns]
        df = df[['latitude', 'longitude', 'altitude']]
        df = df.astype({'latitude': float, 'longitude': float, 'altitude': float})

    # 2) Drop rows with missing or clearly invalid data
    df = df.dropna(subset=['latitude', 'longitude', 'altitude'])

    # 3) Optional: filter out rows with latitude/longitude outside valid ranges
    df = df[
        (df['latitude'] >= -90) & (df['latitude'] <= 90) &
        (df['longitude'] >= -180) & (df['longitude'] <= 180)
    ]

    # 4) Reset index and save
    df = df.reset_index(drop=True)
    df.to_csv(OUTPUT_CSV, index=False)
    print(f"Cleaned data written to: {OUTPUT_CSV}")

if __name__ == '__main__':
    main()
