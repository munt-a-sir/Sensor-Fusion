#!/usr/bin/env python3
"""
EVK-M9DR vs NovAtel PwrPak7 comparison chart — Raw IMU.

Usage:
    python compare.py                                   # auto-selects latest logs
    python compare.py logs/EVK/foo_imu.csv logs/bar.csv
    python compare.py --save                            # save PNG instead of showing
"""
import argparse
import re
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import pandas as pd

HERE        = Path(__file__).parent
LOG_EVK     = HERE / 'logs' / 'EVK'
LOG_NOVATEL = HERE / 'logs'

# (title, EVK column, NovAtel column, y-label, EVK legend label, NovAtel legend label)
# EVK raw IMU comes from ESF-MEAS; NovAtel raw IMU comes from RAWIMU record.
# Heading: EVK uses ESF-ALG Yaw from the .txt log (NAV-ATT is not logged by the dashboard).
PANELS = [
    ('Raw Accel X', 'accel_x_ms2', 'raw_accel_x_ms2', 'm/s²', 'EVK (RAW IMU)', 'NovAtel (RAWIMU)'),
    ('Raw Accel Y', 'accel_y_ms2', 'raw_accel_y_ms2', 'm/s²', 'EVK (RAW IMU)', 'NovAtel (RAWIMU)'),
    ('Raw Accel Z', 'accel_z_ms2', 'raw_accel_z_ms2', 'm/s²', 'EVK (RAW IMU)', 'NovAtel (RAWIMU)'),
    ('Raw Gyro X',  'gyro_x_degs', 'raw_gyro_x_degs', '°/s',  'EVK (RAW IMU)', 'NovAtel (RAWIMU)'),
    ('Raw Gyro Y',  'gyro_y_degs', 'raw_gyro_y_degs', '°/s',  'EVK (RAW IMU)', 'NovAtel (RAWIMU)'),
    ('Raw Gyro Z',  'gyro_z_degs', 'raw_gyro_z_degs', '°/s',  'EVK (RAW IMU)', 'NovAtel (RAWIMU)'),
    ('Heading',     'nav_hdg',     'azimuth_deg',     '°',     'EVK (NAV-ATT)', 'NovAtel (SPAN)'),
]

NOV_RAW_COLS = [
    'raw_accel_x_ms2', 'raw_accel_y_ms2', 'raw_accel_z_ms2',
    'raw_gyro_x_degs', 'raw_gyro_y_degs', 'raw_gyro_z_degs',
]


def _latest_evk(directory: Path) -> Path | None:
    files = sorted(directory.glob('*_imu.csv'))
    return files[-1] if files else None


def _latest_novatel(directory: Path) -> Path | None:
    # Prefer the 100 Hz IMU file; fall back to the 10 Hz combined CSV.
    files_100hz = sorted(directory.glob('novatel_[0-9]*_imu100hz.csv'))
    if files_100hz:
        return files_100hz[-1]
    files_10hz = sorted(
        f for f in directory.glob('novatel_[0-9]*.csv')
        if '_imu100hz' not in f.name
    )
    return files_10hz[-1] if files_10hz else None


def _load(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df['t'] = pd.to_datetime(df['unix_timestamp'].astype(float), unit='s', utc=True)
    return df.set_index('t').sort_index()


def _load_evk_heading(txt_path: Path) -> pd.Series:
    """Parse ESF-ALG Yaw from the EVK .txt log as a UTC-indexed heading series (fallback)."""
    if not txt_path.exists():
        print(f'[compare] EVK heading → {txt_path.name} not found, heading panel will be empty')
        return pd.Series(dtype=float, name='nav_hdg')

    m = re.search(r'(\d{4}-\d{2}-\d{2})', txt_path.name)
    log_date = m.group(1) if m else '1970-01-01'

    pat = re.compile(r'^\[(\d{2}:\d{2}:\d{2})\].*?ESF-ALG.*?Yaw:([+-]?\d+\.?\d*)')
    records = []
    with open(txt_path) as f:
        for line in f:
            match = pat.match(line)
            if not match:
                continue
            dt = datetime.strptime(
                f'{log_date} {match.group(1)}', '%Y-%m-%d %H:%M:%S'
            ).replace(tzinfo=timezone.utc)
            records.append((pd.Timestamp(dt), float(match.group(2))))

    if not records:
        print(f'[compare] EVK heading → no ESF-ALG Yaw entries in {txt_path.name}')
        return pd.Series(dtype=float, name='nav_hdg')

    times, values = zip(*records)
    print(f'[compare] EVK heading → {txt_path.name} ({len(records)} ESF-ALG Yaw samples, fallback)')
    return pd.Series(list(values), index=pd.DatetimeIndex(list(times), tz='UTC'), name='nav_hdg')


def _load_screencast_heading(navatt_path: Path) -> pd.Series:
    """Load manually-extracted NAV-ATT headings from a screencast CSV (legacy fallback)."""
    if not navatt_path.exists():
        return pd.Series(dtype=float, name='nav_hdg')
    df = pd.read_csv(navatt_path)
    df['t'] = pd.to_datetime(df['unix_timestamp'].astype(float), unit='s', utc=True)
    col = next((c for c in ('nav_att_hdg', 'heading_deg') if c in df.columns), None)
    if col is None:
        return pd.Series(dtype=float, name='nav_hdg')
    s = df.set_index('t')[col].dropna().rename('nav_hdg')
    print(f'[compare] EVK heading → {navatt_path.name} ({len(s)} screencast NAV-ATT samples, legacy)')
    return s


def _inject_evk_heading(evk: pd.DataFrame, evk_path: Path) -> pd.DataFrame:
    """Attach nav_hdg to the EVK DataFrame.

    Priority:
      1. nav_hdg column already in the IMU CSV (new logs with NAV-ATT logging)
      2. Sibling *_navatt.csv  (screencast-extracted, legacy)
      3. ESF-ALG Yaw from the sibling .txt log (last resort)

    Sparse sources (1–10 Hz) are upsampled to the ~100 Hz IMU index via ffill.
    """
    if 'nav_hdg' in evk.columns:
        valid = evk['nav_hdg'].dropna()
        if not valid.empty:
            print(f'[compare] EVK heading → IMU CSV nav_hdg column ({len(valid)} samples)')
            return evk

    hdg = pd.Series(dtype=float, name='nav_hdg')

    nav_path    = Path(str(evk_path).replace('_imu.csv', '_nav.csv'))
    navatt_path = Path(str(evk_path).replace('_imu.csv', '_navatt.csv'))
    if hdg.empty:
        hdg = _load_screencast_heading(nav_path)
    if hdg.empty:
        hdg = _load_screencast_heading(navatt_path)

    if hdg.empty:
        txt_path = Path(str(evk_path).replace('_imu.csv', '.txt'))
        hdg = _load_evk_heading(txt_path)

    if hdg.empty:
        return evk

    combined_idx = evk.index.union(hdg.index).sort_values()
    hdg_up = hdg.reindex(combined_idx).ffill().reindex(evk.index)
    evk = evk.copy()
    evk['nav_hdg'] = hdg_up.values
    return evk


def _inject_novatel_heading(nov: pd.DataFrame, nov_path: Path) -> pd.DataFrame:
    """Ensure azimuth_deg is present; if using a 100 Hz file, pull from the 10 Hz sibling."""
    if 'azimuth_deg' in nov.columns:
        return nov
    sibling = Path(re.sub(r'_imu100hz.*\.csv$', '.csv', str(nov_path)))
    if not sibling.exists():
        return nov
    print(f'[compare] NovAtel heading → {sibling.name}')
    hz10 = _load(sibling)[['azimuth_deg']].dropna()
    return nov.join(hz10, how='left').ffill()


def _drop_inactive(df: pd.DataFrame, cols: list) -> pd.DataFrame:
    """Drop rows where all given columns are zero or NaN (sensor not yet active)."""
    existing = [c for c in cols if c in df.columns]
    if not existing:
        return df
    return df[df[existing].abs().sum(axis=1) > 0]


def _clip_overlap(a: pd.DataFrame, b: pd.DataFrame):
    start = max(a.index.min(), b.index.min())
    end   = min(a.index.max(), b.index.max())
    if start >= end:
        print('[compare] Warning: logs do not overlap in time — plotting full range.')
        return a, b
    return a.loc[start:end], b.loc[start:end]


def build_chart(evk: pd.DataFrame, nov: pd.DataFrame, save: bool = False) -> None:
    n   = len(PANELS)
    fig, axes = plt.subplots(n, 1, figsize=(14, 2.6 * n), sharex=True)
    fig.suptitle('EVK-M9DR vs NovAtel PwrPak7 — Raw IMU', fontsize=13)

    fmt = mdates.DateFormatter('%H:%M:%S')

    for ax, (title, evk_col, nov_col, ylabel, evk_label, nov_label) in zip(axes, PANELS):
        plotted = False
        if evk_col in evk.columns:
            s = evk[evk_col].dropna()
            if not s.empty:
                ax.plot(s.index, s, label=evk_label, color='steelblue',
                        linewidth=0.7, alpha=0.9)
                plotted = True
        if nov_col in nov.columns:
            s = nov[nov_col].dropna()
            if not s.empty:
                ax.plot(s.index, s, label=nov_label, color='tomato',
                        linewidth=0.7, alpha=0.9)
                plotted = True
        if not plotted:
            ax.text(0.5, 0.5, 'No data', transform=ax.transAxes,
                    ha='center', va='center', color='grey', fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_title(title, fontsize=9, loc='left', pad=2)
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, linewidth=0.3, alpha=0.5)
        ax.xaxis.set_major_formatter(fmt)

    axes[-1].set_xlabel('Time (UTC)')
    fig.autofmt_xdate(rotation=30, ha='right')
    plt.tight_layout()

    if save:
        out = HERE / 'logs' / 'comparison.png'
        fig.savefig(out, dpi=150, bbox_inches='tight')
        print(f'[compare] Saved → {out}')
    else:
        plt.show()


def main() -> None:
    parser = argparse.ArgumentParser(description='EVK vs NovAtel Raw IMU comparison chart')
    parser.add_argument('evk',     nargs='?', help='EVK IMU CSV')
    parser.add_argument('novatel', nargs='?', help='NovAtel 10 Hz CSV')
    parser.add_argument('--save',  action='store_true', help='Save PNG instead of showing')
    args = parser.parse_args()

    evk_path = Path(args.evk)     if args.evk     else _latest_evk(LOG_EVK)
    nov_path = Path(args.novatel) if args.novatel else _latest_novatel(LOG_NOVATEL)

    if not evk_path or not evk_path.exists():
        print(f'[compare] No EVK IMU CSV found in {LOG_EVK}'); return
    if not nov_path or not nov_path.exists():
        print(f'[compare] No NovAtel CSV found in {LOG_NOVATEL}'); return

    print(f'[compare] EVK     → {evk_path.name}')
    print(f'[compare] NovAtel → {nov_path.name}')

    evk = _load(evk_path)
    nov = _load(nov_path)
    evk = _inject_evk_heading(evk, evk_path)
    nov = _inject_novatel_heading(nov, nov_path)
    nov = _drop_inactive(nov, NOV_RAW_COLS)
    evk, nov = _clip_overlap(evk, nov)

    build_chart(evk, nov, save=args.save)


if __name__ == '__main__':
    main()
