#!/usr/bin/env python3
"""
EVK-M9DR vs NovAtel PwrPak7 comparison charts — Raw IMU and Vehicle Frame INS.

Usage:
    python compare.py                                   # auto-selects latest logs
    python compare.py logs/EVK/foo_imu.csv logs/bar.csv
    python compare.py --save                            # save PNGs instead of showing
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
LOG_NOVATEL = HERE / 'logs' / 'Novatel'

# Chart 1 — Raw IMU comparison (ESF-MEAS vs RAWIMUSXA)
RAW_PANELS = [
    ('Raw Accel X', 'accel_x_ms2',    'raw_accel_x_ms2', 'm/s²', 'EVK (ESF-MEAS)', 'NovAtel (RAWIMUSXA)'),
    ('Raw Accel Y', 'accel_y_ms2',    'raw_accel_y_ms2', 'm/s²', 'EVK (ESF-MEAS)', 'NovAtel (RAWIMUSXA)'),
    ('Raw Accel Z', 'accel_z_ms2',    'raw_accel_z_ms2', 'm/s²', 'EVK (ESF-MEAS)', 'NovAtel (RAWIMUSXA)'),
    ('Raw Gyro X',  'gyro_x_degs',    'raw_gyro_x_degs', '°/s',  'EVK (ESF-MEAS)', 'NovAtel (RAWIMUSXA)'),
    ('Raw Gyro Y',  'gyro_y_degs',    'raw_gyro_y_degs', '°/s',  'EVK (ESF-MEAS)', 'NovAtel (RAWIMUSXA)'),
    ('Raw Gyro Z',  'gyro_z_degs',    'raw_gyro_z_degs', '°/s',  'EVK (ESF-MEAS)', 'NovAtel (RAWIMUSXA)'),
    ('Heading',     'nav_hdg',         'azimuth_deg',     '°',    'EVK (NAV-ATT)',  'NovAtel (SPAN)'),
]

# Chart 2 — Vehicle-frame INS comparison (ESF-INS vs CORRIMUDATAA)
VEH_PANELS = [
    ('Veh Accel X (Lateral)',      'ins_accel_x_ms2', 'veh_lat_acc_ms2',    'm/s²', 'EVK (ESF-INS)', 'NovAtel (CORRIMUDATAA)'),
    ('Veh Accel Y (Longitudinal)', 'ins_accel_y_ms2', 'veh_long_acc_ms2',   'm/s²', 'EVK (ESF-INS)', 'NovAtel (CORRIMUDATAA)'),
    ('Veh Accel Z (Vertical)',     'ins_accel_z_ms2', 'veh_vert_acc_ms2',   'm/s²', 'EVK (ESF-INS)', 'NovAtel (CORRIMUDATAA)'),
    ('Veh Gyro X (Pitch rate)',    'ins_gyro_x_degs', 'veh_pitch_rate_degs','°/s',  'EVK (ESF-INS)', 'NovAtel (CORRIMUDATAA)'),
    ('Veh Gyro Y (Roll rate)',     'ins_gyro_y_degs', 'veh_roll_rate_degs', '°/s',  'EVK (ESF-INS)', 'NovAtel (CORRIMUDATAA)'),
    ('Veh Gyro Z (Yaw rate)',      'ins_gyro_z_degs', 'veh_yaw_rate_degs',  '°/s',  'EVK (ESF-INS)', 'NovAtel (CORRIMUDATAA)'),
]

# Chart 3 — Attitude comparison (NAV-ATT roll/pitch vs INSPVAXA roll/pitch)
ATT_PANELS = [
    ('Roll',  'roll_deg',  'roll_deg',  '°', 'EVK (NAV-ATT)', 'NovAtel (INSPVAXA)'),
    ('Pitch', 'pitch_deg', 'pitch_deg', '°', 'EVK (NAV-ATT)', 'NovAtel (INSPVAXA)'),
]

NOV_RAW_COLS = [
    'raw_accel_x_ms2', 'raw_accel_y_ms2', 'raw_accel_z_ms2',
    'raw_gyro_x_degs', 'raw_gyro_y_degs', 'raw_gyro_z_degs',
]

NOV_VEH_COLS = [
    'veh_lat_acc_ms2', 'veh_long_acc_ms2', 'veh_vert_acc_ms2',
    'veh_pitch_rate_degs', 'veh_roll_rate_degs', 'veh_yaw_rate_degs',
]

EVK_VEH_COLS = [
    'ins_accel_x_ms2', 'ins_accel_y_ms2', 'ins_accel_z_ms2',
    'ins_gyro_x_degs', 'ins_gyro_y_degs', 'ins_gyro_z_degs',
]


def _latest_evk(directory: Path) -> Path | None:
    files = sorted(directory.glob('*_imu.csv'))
    return files[-1] if files else None


def _latest_novatel(directory: Path) -> Path | None:
    files_100hz = sorted(directory.glob('novatel_[0-9]*_imu100hz.csv'))
    if files_100hz:
        return files_100hz[-1]
    files_10hz = sorted(
        f for f in directory.glob('novatel_[0-9]*.csv')
        if '_imu100hz' not in f.name
    )
    return files_10hz[-1] if files_10hz else None


_GPS_EPOCH_UNIX = 315964800
_LEAP_SECONDS   = 18


def _load_novatel(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    if 'gps_week' in df.columns and 'gps_seconds' in df.columns:
        mask = df['gps_week'].notna() & df['gps_seconds'].notna()
        unix = (
            _GPS_EPOCH_UNIX
            + df.loc[mask, 'gps_week'].astype(int) * 604800
            + df.loc[mask, 'gps_seconds'].astype(float)
            - _LEAP_SECONDS
        )
        df.loc[mask, '_gps_unix'] = unix
        df['t'] = pd.to_datetime(df['_gps_unix'].fillna(
            df['unix_timestamp'].astype(float)
        ), unit='s', utc=True)
    else:
        df['t'] = pd.to_datetime(df['unix_timestamp'].astype(float), unit='s', utc=True)
    return df.set_index('t').sort_index()


def _load_evk(path: Path) -> pd.DataFrame:
    df = pd.read_csv(path)
    df['t'] = pd.to_datetime(df['unix_timestamp'].astype(float), unit='s', utc=True)
    return df.set_index('t').sort_index()


def _load_evk_heading(txt_path: Path) -> pd.Series:
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
    if 'azimuth_deg' in nov.columns:
        return nov
    sibling = Path(re.sub(r'_imu100hz.*\.csv$', '.csv', str(nov_path)))
    if not sibling.exists():
        return nov
    print(f'[compare] NovAtel heading → {sibling.name}')
    hz10 = _load_novatel(sibling)[['azimuth_deg']].dropna()
    return nov.join(hz10, how='left').ffill()


def _inject_evk_attitude(evk: pd.DataFrame, evk_path: Path) -> pd.DataFrame:
    """Load roll_deg and pitch_deg from the sibling _nav.csv (NAV-ATT) into the EVK DataFrame."""
    nav_path = Path(str(evk_path).replace('_imu.csv', '_nav.csv'))
    if not nav_path.exists():
        print(f'[compare] EVK attitude → {nav_path.name} not found, attitude panel will be empty')
        return evk
    nav = pd.read_csv(nav_path)
    nav['t'] = pd.to_datetime(nav['unix_timestamp'].astype(float), unit='s', utc=True)
    nav = nav.set_index('t').sort_index()
    cols = [c for c in ('roll_deg', 'pitch_deg') if c in nav.columns]
    if not cols:
        return evk
    print(f'[compare] EVK attitude → {nav_path.name} ({len(nav)} NAV-ATT samples)')
    att = nav[cols].dropna(how='all')
    combined_idx = evk.index.union(att.index).sort_values()
    att_up = att.reindex(combined_idx).ffill().reindex(evk.index)
    evk = evk.copy()
    for col in cols:
        evk[col] = att_up[col].values
    return evk


def _inject_novatel_att(nov: pd.DataFrame, nov_path: Path) -> pd.DataFrame:
    """Ensure roll_deg and pitch_deg are present; pull from 10 Hz sibling if using 100 Hz file."""
    if 'roll_deg' in nov.columns and 'pitch_deg' in nov.columns:
        return nov
    sibling = Path(re.sub(r'_imu100hz.*\.csv$', '.csv', str(nov_path)))
    if not sibling.exists():
        return nov
    print(f'[compare] NovAtel attitude → {sibling.name}')
    hz10 = _load_novatel(sibling)[['roll_deg', 'pitch_deg']].dropna(how='all')
    return nov.join(hz10, how='left').ffill()


def _inject_novatel_veh(nov: pd.DataFrame, nov_path: Path) -> pd.DataFrame:
    """If using the 100 Hz IMU file, vehicle-frame cols may be sparse — ffill them.
    If cols are missing entirely, pull from the 10 Hz sibling CSV."""
    veh_present = [c for c in NOV_VEH_COLS if c in nov.columns]
    if veh_present:
        nov = nov.copy()
        nov[veh_present] = nov[veh_present].ffill()
        return nov
    sibling = Path(re.sub(r'_imu100hz.*\.csv$', '.csv', str(nov_path)))
    if not sibling.exists():
        return nov
    print(f'[compare] NovAtel vehicle-frame → {sibling.name}')
    hz10 = _load_novatel(sibling)[NOV_VEH_COLS].dropna(how='all')
    return nov.join(hz10, how='left').ffill()


def _drop_inactive(df: pd.DataFrame, cols: list) -> pd.DataFrame:
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


def _session_ts(evk_path: Path) -> str:
    """Extract session datetime string from the EVK filename for use in output filenames."""
    m = re.search(r'(\d{4}-\d{2}-\d{2}_\d{2}-\d{2}-\d{2})', evk_path.name)
    return m.group(1) if m else datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


def _build_panels(fig, axes, evk, nov, panels, title):
    fmt = mdates.DateFormatter('%H:%M:%S')
    fig.suptitle(title, fontsize=13)
    for ax, (panel_title, evk_col, nov_col, ylabel, evk_label, nov_label) in zip(axes, panels):
        plotted = False
        if evk_col in evk.columns:
            s = evk[evk_col].dropna()
            if not s.empty:
                ax.plot(s.index, s, label=evk_label, color='steelblue', linewidth=0.7, alpha=0.9)
                plotted = True
        if nov_col in nov.columns:
            s = nov[nov_col].dropna()
            if not s.empty:
                ax.plot(s.index, s, label=nov_label, color='tomato', linewidth=0.7, alpha=0.9)
                plotted = True
        if not plotted:
            ax.text(0.5, 0.5, 'No data', transform=ax.transAxes,
                    ha='center', va='center', color='grey', fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.set_title(panel_title, fontsize=9, loc='left', pad=2)
        ax.legend(fontsize=7, loc='upper right')
        ax.grid(True, linewidth=0.3, alpha=0.5)
        ax.xaxis.set_major_formatter(fmt)
    axes[-1].set_xlabel('Time (UTC)')
    fig.autofmt_xdate(rotation=30, ha='right')
    plt.tight_layout()


def main() -> None:
    parser = argparse.ArgumentParser(description='EVK vs NovAtel IMU comparison charts')
    parser.add_argument('evk',     nargs='?', help='EVK IMU CSV')
    parser.add_argument('novatel', nargs='?', help='NovAtel IMU CSV')
    parser.add_argument('--save',  action='store_true', help='Save PNGs instead of showing')
    args = parser.parse_args()

    evk_path = Path(args.evk)     if args.evk     else _latest_evk(LOG_EVK)
    nov_path = Path(args.novatel) if args.novatel else _latest_novatel(LOG_NOVATEL)

    if not evk_path or not evk_path.exists():
        print(f'[compare] No EVK IMU CSV found in {LOG_EVK}'); return
    if not nov_path or not nov_path.exists():
        print(f'[compare] No NovAtel CSV found in {LOG_NOVATEL}'); return

    print(f'[compare] EVK     → {evk_path.name}')
    print(f'[compare] NovAtel → {nov_path.name}')

    evk = _load_evk(evk_path)
    nov = _load_novatel(nov_path)
    evk = _inject_evk_heading(evk, evk_path)
    evk = _inject_evk_attitude(evk, evk_path)
    nov = _inject_novatel_heading(nov, nov_path)
    nov = _inject_novatel_veh(nov, nov_path)
    nov = _inject_novatel_att(nov, nov_path)
    nov = _drop_inactive(nov, NOV_RAW_COLS)
    evk, nov = _clip_overlap(evk, nov)

    session = _session_ts(evk_path)
    log_dir = HERE / 'logs'

    # ── Chart 1: Raw IMU ──────────────────────────────────────────────────────
    fig1, axes1 = plt.subplots(len(RAW_PANELS), 1, figsize=(14, 2.6 * len(RAW_PANELS)), sharex=True)
    _build_panels(fig1, axes1, evk, nov, RAW_PANELS,
                  f'EVK-M9DR vs NovAtel PwrPak7 — Raw IMU  [{session}]')

    # ── Chart 2: Vehicle-frame INS ────────────────────────────────────────────
    evk_veh = _drop_inactive(evk, EVK_VEH_COLS)
    nov_veh = _drop_inactive(nov, NOV_VEH_COLS)
    fig2, axes2 = plt.subplots(len(VEH_PANELS), 1, figsize=(14, 2.6 * len(VEH_PANELS)), sharex=True)
    _build_panels(fig2, axes2, evk_veh, nov_veh, VEH_PANELS,
                  f'EVK-M9DR (ESF-INS) vs NovAtel (CORRIMUDATAA) — Vehicle Frame  [{session}]')

    # ── Chart 3: Attitude (roll/pitch) ────────────────────────────────────────
    fig3, axes3 = plt.subplots(len(ATT_PANELS), 1, figsize=(14, 2.6 * len(ATT_PANELS)), sharex=True)
    _build_panels(fig3, axes3, evk, nov, ATT_PANELS,
                  f'EVK-M9DR (NAV-ATT) vs NovAtel (INSPVAXA) — Roll & Pitch  [{session}]')

    if args.save:
        out1 = log_dir / f'comparison_raw_{session}.png'
        out2 = log_dir / f'comparison_veh_{session}.png'
        out3 = log_dir / f'comparison_att_{session}.png'
        fig1.savefig(out1, dpi=150, bbox_inches='tight')
        fig2.savefig(out2, dpi=150, bbox_inches='tight')
        fig3.savefig(out3, dpi=150, bbox_inches='tight')
        print(f'[compare] Saved → {out1}')
        print(f'[compare] Saved → {out2}')
        print(f'[compare] Saved → {out3}')
    else:
        plt.show()


if __name__ == '__main__':
    main()
