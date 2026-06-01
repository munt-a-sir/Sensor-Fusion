import serial
import pandas as pd
import numpy as np
import time

import plotly.graph_objs as go
from plotly.subplots import make_subplots

SERIAL_PORT = '/dev/ttyUSB0'
BAUD_RATE = 115200

RAWIMUSXA_COLS = [
    'head', 'week_num1', 'seconds1', 'IMU_info', 'week_num2', 'seconds2',
    'Status', 'AccelZ', '-AccelY', 'AccelX', 'GyroZ', '-GyroY', 'GyroX', 'CRC'
]

INSPVAXA_COLS = [
    'head', 'port', 'sequence', 'Idle_perc', 'time_status', 'week', 'seconds',
    'receiver_status', 'reserved', 'sw_version', 'pose_type', 'lat', 'long',
    'height', 'Undulation', 'vel_N', 'vel_E', 'vel_U', 'roll', 'pitch',
    'azimuth', 'lat_sig', 'long_sig', 'height_sig', 'velN_sig', 'velE_sig',
    'velU_sig', 'roll_sig', 'pitch_sig', 'azimuth_sig', 'ext_status', 'CRC'
]


def parse_line(line, header):
    if header not in line:
        return None
    fields = line.strip().split(',')
    if header == "RAWIMUSXA":
        if len(fields) >= 13:
            fields[12:13] = fields[12].split('*')
        if len(fields) == len(RAWIMUSXA_COLS):
            return dict(zip(RAWIMUSXA_COLS, fields))
    elif header == "INSPVAXA":
        if len(fields) >= 32:
            fields[31] = fields[31].split('*')[0]
        if len(fields) == len(INSPVAXA_COLS):
            return dict(zip(INSPVAXA_COLS, fields))
    return None


def read_from_file(path, header):
    rows = []
    with open(path, 'r') as f:
        for line in f:
            row = parse_line(line, header)
            if row:
                rows.append(row)
    return pd.DataFrame(rows)


def read_live(port=SERIAL_PORT, baud=BAUD_RATE, duration_sec=30, headers=None):
    if headers is None:
        headers = ["RAWIMUSXA", "INSPVAXA"]

    results = {h: [] for h in headers}
    print(f"Opening {port} at {baud} baud...")

    with serial.Serial(port, baud, timeout=1) as ser:
        for header in headers:
            rate = 0.01 if header == "RAWIMUSXA" else 0.1
            cmd = f'log {header.lower()} ontime {rate}\r\n'
            ser.write(cmd.encode())
            print(f"Sent: {cmd.strip()}")

        print(f"Reading for {duration_sec}s... (Ctrl+C to stop early)")
        start = time.time()
        try:
            while time.time() - start < duration_sec:
                raw = ser.readline()
                if not raw:
                    continue
                line = raw.decode('ascii', errors='ignore')
                for header in headers:
                    row = parse_line(line, header)
                    if row:
                        results[header].append(row)
                        print(f"[{header}] {line.strip()}")
        except KeyboardInterrupt:
            print("Stopped early.")

    return {h: pd.DataFrame(v) for h, v in results.items()}


def plot_imu(df):
    if df.empty:
        print("No RAWIMUSXA data to plot.")
        return

    for col in ['AccelX', 'AccelZ', 'GyroX', 'GyroZ']:
        df[col] = pd.to_numeric(df[col], errors='coerce')
    df['AccelY'] = pd.to_numeric(df['-AccelY'], errors='coerce') * -1
    df['GyroY'] = pd.to_numeric(df['-GyroY'], errors='coerce') * -1

    fig = make_subplots(rows=2, cols=1, subplot_titles=('Accelerometer', 'Gyroscope'))

    for col, row in [('AccelX', 1), ('AccelY', 1), ('AccelZ', 1),
                     ('GyroX', 2), ('GyroY', 2), ('GyroZ', 2)]:
        fig.add_trace(go.Scatter(y=df[col], name=col, mode='lines'), row=row, col=1)

    fig.update_layout(template='plotly_dark', title='PwrPak7 Raw IMU')
    fig.show()


if __name__ == '__main__':
    data = read_live(port=SERIAL_PORT, baud=BAUD_RATE, duration_sec=30)

    imu_df = data.get("RAWIMUSXA", pd.DataFrame())
    ins_df = data.get("INSPVAXA", pd.DataFrame())

    print(f"\nRAWIMUSXA rows: {len(imu_df)}")
    print(f"INSPVAXA rows:  {len(ins_df)}")

    if not imu_df.empty:
        print(imu_df.head())
        plot_imu(imu_df)
