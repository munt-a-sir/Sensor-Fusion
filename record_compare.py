#!/usr/bin/env python3
"""
record_compare.py — Record raw IMU from EVK-M9DR + NovAtel, then plot.

Stop the EVK dashboard before running this (it owns /dev/ttyACM0).

Usage:
    python3 record_compare.py
    python3 record_compare.py --save
"""
import argparse
import threading
import time
from pathlib import Path

import matplotlib.dates as mdates
import matplotlib.pyplot as plt
import pandas as pd
import serial
from pyubx2 import UBX_PROTOCOL, NMEA_PROTOCOL, UBXMessage, UBXReader, SET_LAYER_RAM, TXN_NONE

_log_lines: list = []
_log_lock = threading.Lock()

def _log(line: str) -> None:
    print(line)
    with _log_lock:
        _log_lines.append(line)

EVK_PORT = '/dev/ttyACM0'
EVK_BAUD = 115200
NOV_PORT = '/dev/ttyUSB0'
NOV_BAUD = 460800

EVK_ACCEL_SCALE = 1 / 1024.0
EVK_GYRO_SCALE  = 1 / 4096.0
NOV_ACCEL_SCALE = (0.400 / 65536) * (9.80665 / 1000)
NOV_GYRO_SCALE  = 0.0151515 / 65536

ACCEL_TYPES = {16, 17, 18}
GYRO_TYPES  = {14, 13, 5}
ALL_TYPES   = ACCEL_TYPES | GYRO_TYPES

RAWIMUSXA_COLS = [
    'head', 'week_num1', 'seconds1', 'imu_info', 'week_num2', 'seconds2',
    'status', 'accel_z', 'neg_accel_y', 'accel_x', 'gyro_z', 'neg_gyro_y', 'gyro_x', 'crc',
]

PANELS = [
    ('Accel X', 'accel_x', 'm/s²'),
    ('Accel Y', 'accel_y', 'm/s²'),
    ('Accel Z', 'accel_z', 'm/s²'),
    ('Gyro X',  'gyro_x',  '°/s'),
    ('Gyro Y',  'gyro_y',  '°/s'),
    ('Gyro Z',  'gyro_z',  '°/s'),
]


def signed24(v: int) -> int:
    return v - (1 << 24) if v & (1 << 23) else v


# ── EVK thread ─────────────────────────────────────────────────────────────────

def evk_thread(stop: threading.Event, rows: list) -> None:
    try:
        ser = serial.Serial(EVK_PORT, EVK_BAUD, timeout=1)
    except serial.SerialException as e:
        print(f'[EVK] Cannot open {EVK_PORT}: {e}')
        return

    # Exact same config as the dashboard — proven to work on this device
    cfg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, [
        ('CFG-SFCORE-USE_SF',             1),
        ('CFG-SFIMU-IMU_EN',              1),
        ('CFG-SFIMU-AUTO_MNTALG_ENA',     1),
        ('CFG-MSGOUT-UBX_ESF_MEAS_USB',   1),
        ('CFG-MSGOUT-UBX_ESF_ALG_USB',    1),
        ('CFG-MSGOUT-UBX_ESF_INS_USB',    1),
        ('CFG-MSGOUT-UBX_ESF_STATUS_USB', 1),
        ('CFG-MSGOUT-UBX_NAV_PVT_USB',    1),
        ('CFG-MSGOUT-UBX_NAV_ATT_USB',    1),
        ('CFG-MSGOUT-UBX_NAV_STATUS_USB', 1),
        ('CFG-MSGOUT-NMEA_ID_GGA_USB',    1),
        ('CFG-MSGOUT-NMEA_ID_GSA_USB',    1),
        ('CFG-MSGOUT-NMEA_ID_RMC_USB',    1),
        ('CFG-MSGOUT-NMEA_ID_THS_USB',    1),
        ('CFG-MSGOUT-NMEA_ID_ZDA_USB',    1),
        ('CFG-RATE-MEAS',    10),
        ('CFG-RATE-NAV',      1),
        ('CFG-RATE-TIMEREF',  0),
    ])
    ser.write(cfg.serialize())
    print(f'[EVK] Config sent, reading {EVK_PORT}')

    ubr    = UBXReader(ser, protfilter=UBX_PROTOCOL | NMEA_PROTOCOL)
    latest = {}
    fresh: set = set()
    evk_count = 0
    evk_t0: float | None = None
    evk_last_report = 0.0

    while not stop.is_set():
        try:
            _, parsed = ubr.read()
        except Exception:
            continue
        if parsed is None:
            continue
        if parsed.identity != 'ESF-MEAS':
            continue

        t = time.time()
        i = 1
        while True:
            dtype_val  = getattr(parsed, f'dataType_{i:02d}', None)
            dfield_val = getattr(parsed, f'dataField_{i:02d}', None)
            if dtype_val is None:
                break
            dtype = int(dtype_val)
            if dtype in ALL_TYPES:
                latest[dtype] = signed24(int(dfield_val))
                fresh.add(dtype)
            i += 1

        if ALL_TYPES.issubset(fresh):
            fresh.clear()
            ax_l, ay_l, az_l = latest[16], latest[17], latest[18]
            gx_l, gy_l, gz_l = latest[14], latest[13], latest[5]
            if evk_t0 is None:
                evk_t0 = t
                evk_last_report = t
            ts = f'{t % 1000:.3f}'
            _log(f'[EVK {ts}]  m/s² ax={ax_l*EVK_ACCEL_SCALE:8.4f} ay={ay_l*EVK_ACCEL_SCALE:8.4f} az={az_l*EVK_ACCEL_SCALE:8.4f} | °/s gx={gx_l*EVK_GYRO_SCALE:8.4f} gy={gy_l*EVK_GYRO_SCALE:8.4f} gz={gz_l*EVK_GYRO_SCALE:8.4f}')
            rows.append({'ts': t, 'accel_x': ax_l*EVK_ACCEL_SCALE, 'accel_y': ay_l*EVK_ACCEL_SCALE, 'accel_z': az_l*EVK_ACCEL_SCALE, 'gyro_x': gx_l*EVK_GYRO_SCALE, 'gyro_y': gy_l*EVK_GYRO_SCALE, 'gyro_z': gz_l*EVK_GYRO_SCALE})
            evk_count += 1
            if t - evk_last_report >= 1.0:
                elapsed = t - evk_t0 if evk_t0 else 1.0
                hz = evk_count / elapsed
                print(f'[EVK] {evk_count} pkts total @ {hz:.1f} Hz')
                evk_last_report = t

    ser.close()
    print('[EVK] Stopped')


# ── NovAtel thread ─────────────────────────────────────────────────────────────

def _parse_rawimusxa(line: str):
    for prefix in ('%', '#'):
        idx = line.find(f'{prefix}RAWIMUSXA')
        if idx >= 0:
            line = line[idx:]
            break
    else:
        return None

    fields = line.strip().split(',')
    if len(fields) >= 13 and '*' in fields[12]:
        val, crc = fields[12].split('*', 1)
        fields[12:13] = [val, crc.split()[0]]
    if len(fields) != len(RAWIMUSXA_COLS):
        return None

    d = dict(zip(RAWIMUSXA_COLS, fields))
    try:
        ax = int(d['accel_x']);  ay = -int(d['neg_accel_y']); az = int(d['accel_z'])
        gx = int(d['gyro_x']);   gy = -int(d['neg_gyro_y']);  gz = int(d['gyro_z'])
        gps_sec = float(d['seconds2'])
    except (ValueError, KeyError):
        return None

    return {
        '_ax': ax, '_ay': ay, '_az': az, '_gx': gx, '_gy': gy, '_gz': gz,
        '_gps_sec': gps_sec,
        'accel_x': ax * NOV_ACCEL_SCALE, 'accel_y': ay * NOV_ACCEL_SCALE, 'accel_z': az * NOV_ACCEL_SCALE,
        'gyro_x':  gx * NOV_GYRO_SCALE,  'gyro_y':  gy * NOV_GYRO_SCALE,  'gyro_z':  gz * NOV_GYRO_SCALE,
    }


def nov_thread(stop: threading.Event, rows: list) -> None:
    try:
        ser = serial.Serial(NOV_PORT, NOV_BAUD, timeout=0.05)
    except serial.SerialException as e:
        print(f'[NovAtel] Cannot open {NOV_PORT}: {e}')
        return

    ser.write(b'unlogall all\r\n')
    time.sleep(0.5)
    ser.reset_input_buffer()
    ser.write(b'log rawimusxa ontime 0.01\r\n')
    print(f'[NovAtel] Config sent, reading {NOV_PORT}')

    # Use packet count × 10 ms for timestamps: readline() stalls corrupt
    # wall-clock times, but the NovAtel outputs at a rock-solid 100 Hz
    # regardless of GPS lock (we commanded "ontime 0.01").
    t0_wall: float | None = None
    pkt_count = 0
    nov_count = 0
    nov_t0: float | None = None
    nov_last_report = 0.0

    while not stop.is_set():
        raw = ser.readline()
        if not raw:
            continue
        parsed = _parse_rawimusxa(raw.decode('ascii', errors='ignore'))
        if parsed:
            if t0_wall is None:
                t0_wall = time.time()

            t = t0_wall + pkt_count * 0.01
            pkt_count += 1

            if nov_t0 is None:
                nov_t0 = t
                nov_last_report = t
            ts = f'{t % 1000:.3f}'
            _log(f'[NOV {ts}]  m/s² ax={parsed["accel_x"]:8.4f} ay={parsed["accel_y"]:8.4f} az={parsed["accel_z"]:8.4f} | °/s gx={parsed["gyro_x"]:8.4f} gy={parsed["gyro_y"]:8.4f} gz={parsed["gyro_z"]:8.4f}')
            rows.append({'ts': t, 'accel_x': parsed['accel_x'], 'accel_y': parsed['accel_y'], 'accel_z': parsed['accel_z'], 'gyro_x': parsed['gyro_x'], 'gyro_y': parsed['gyro_y'], 'gyro_z': parsed['gyro_z']})
            nov_count += 1
            if t - nov_last_report >= 1.0:
                elapsed = t - nov_t0 if nov_t0 else 1.0
                hz = nov_count / elapsed
                print(f'[NOV] {nov_count} pkts total @ {hz:.1f} Hz')
                nov_last_report = t

    ser.write(b'unlogall all\r\n')
    ser.close()
    print('[NovAtel] Stopped')


# ── Plot ───────────────────────────────────────────────────────────────────────

def plot(evk_rows: list, nov_rows: list, save_path=None) -> None:
    if not evk_rows and not nov_rows:
        print('No data recorded.')
        return

    def to_df(rows):
        df = pd.DataFrame(rows)
        df.index = pd.to_datetime(df['ts'], unit='s', utc=True)
        return df.drop(columns='ts')

    evk_df = to_df(evk_rows) if evk_rows else None
    nov_df = to_df(nov_rows) if nov_rows else None

    fig, axes = plt.subplots(len(PANELS), 1, figsize=(14, 2.5 * len(PANELS)), sharex=True)
    fig.suptitle('EVK-M9DR vs NovAtel — Raw IMU', fontsize=12)
    fmt = mdates.DateFormatter('%H:%M:%S')

    for ax, (title, col, unit) in zip(axes, PANELS):
        if evk_df is not None and col in evk_df.columns:
            s = evk_df[col].dropna()
            ax.plot(s.index, s, lw=0.7, alpha=0.9, color='steelblue', label='EVK')
        if nov_df is not None and col in nov_df.columns:
            s = nov_df[col].dropna()
            ax.plot(s.index, s, lw=0.7, alpha=0.9, color='tomato', label='NovAtel')
        ax.set_ylabel(unit, fontsize=8)
        ax.set_title(title, fontsize=9, loc='left', pad=2)
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, lw=0.3, alpha=0.5)
        ax.xaxis.set_major_formatter(fmt)

    axes[-1].set_xlabel('Time (UTC)')
    fig.autofmt_xdate(rotation=30, ha='right')
    plt.tight_layout()

    if save_path:
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        print(f'Saved → {save_path}')
    else:
        plt.show()


# ── Main ───────────────────────────────────────────────────────────────────────

def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument('--save', action='store_true', help='Save PNG instead of showing')
    args = ap.parse_args()

    evk_rows: list = []
    nov_rows: list = []
    stop = threading.Event()

    t_evk = threading.Thread(target=evk_thread, args=(stop, evk_rows), daemon=True)
    t_nov = threading.Thread(target=nov_thread, args=(stop, nov_rows), daemon=True)
    t_evk.start()
    t_nov.start()

    print('Printing raw IMU — Ctrl+C to stop and plot.')
    try:
        while True:
            time.sleep(60)
    except KeyboardInterrupt:
        print()

    stop.set()
    t_evk.join(timeout=3)
    t_nov.join(timeout=3)

    out = Path(__file__).parent / 'logs' / f'imu_raw_{int(time.time())}.txt'
    out.parent.mkdir(exist_ok=True)
    out.write_text('\n'.join(_log_lines) + '\n')
    print(f'Saved {len(_log_lines)} lines → {out}')

    save_path = Path(__file__).parent / 'logs' / 'comparison_live.png' if args.save else None
    plot(evk_rows, nov_rows, save_path)


if __name__ == '__main__':
    main()
