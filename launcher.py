#!/usr/bin/env python3
"""
Launch NovAtel PwrPak7 and EVK-M9DR dashboards simultaneously.
Ctrl+C stops both servers and generates a vehicle-frame comparison chart.

  NovAtel  → http://localhost:5000
  EVK-M9DR → ws://localhost:5052  (open EVK-M9DR-dashboard.html)
"""

import re
import sys
import subprocess
from datetime import datetime, timezone
from pathlib import Path

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import pandas as pd

HERE    = Path(__file__).parent
NOVATEL = HERE / 'Novatel'
EVK     = HERE / 'EVK-M9DR'

NOVATEL_PY = NOVATEL / '.venv' / 'bin' / 'python'
EVK_PY     = sys.executable   # EVK uses system Python
VENV_PY = HERE / '.venv' / 'bin' / 'python'
PY  = str(VENV_PY) if VENV_PY.exists() else sys.executable

NOVATEL_PORT = 5000
EVK_PORT     = 5052
EVK_HTTP_PORT = 5050

# ── Log parsers ───────────────────────────────────────────────────────────────

def _find_novatel_csv(after_utc: datetime) -> Path | None:
    logs = sorted((NOVATEL / 'logs').glob('novatel_*.csv'))
    # Return the most recently modified CSV created after session start
    for p in reversed(logs):
        mtime = datetime.fromtimestamp(p.stat().st_mtime, tz=timezone.utc)
        if mtime >= after_utc:
            return p
    return logs[-1] if logs else None


def _parse_novatel(csv_path: Path) -> pd.DataFrame:
    df = pd.read_csv(csv_path)
    df['utc'] = pd.to_datetime(df['unix_timestamp'].astype(float), unit='s', utc=True)
    accel_cols = ['veh_long_acc_ms2', 'veh_lat_acc_ms2', 'veh_vert_acc_ms2']
    gyro_cols  = ['veh_pitch_rate_degs', 'veh_roll_rate_degs', 'veh_yaw_rate_degs']
    keep = ['utc'] + accel_cols + gyro_cols
    df = df[keep].dropna()
    # Drop rows where all vehicle-frame values are zero (INS inactive)
    mask = df[accel_cols + gyro_cols].abs().sum(axis=1) > 0
    return df[mask].reset_index(drop=True)


_EVK_LINE = re.compile(
    r'\[(\d{2}:\d{2}:\d{2}\.\d+)\]'
    r'.*?Veh\(m/s.\) X:([+-][\d.]+) Y:([+-][\d.]+) Z:([+-][\d.]+)'
    r'.*?VehGyro\(./s\) X:([+-][\d.]+) Y:([+-][\d.]+) Z:([+-][\d.]+)'
)
_EVK_SESSION = re.compile(r'^# Session started (\S+)')


def _parse_evk(log_path: Path, session_start: datetime) -> pd.DataFrame:
    # Find the last "# Session started" marker and read from there.
    # session_start is UTC; the marker uses local datetime.now() — compare loosely.
    text = log_path.read_text(errors='replace')
    lines = text.splitlines()

    last_marker_idx = 0
    for i, line in enumerate(lines):
        if _EVK_SESSION.match(line):
            last_marker_idx = i

    date = session_start.date()
    rows = []
    for line in lines[last_marker_idx:]:
        m = _EVK_LINE.search(line)
        if not m:
            continue
        t_str, vax, vay, vaz, vgx, vgy, vgz = m.groups()
        h, mn, s_str = t_str.split(':')
        s_f = float(s_str)
        ts = pd.Timestamp(
            year=date.year, month=date.month, day=date.day,
            hour=int(h), minute=int(mn),
            second=int(s_f), microsecond=int((s_f % 1) * 1e6),
            tz='UTC',
        )
        rows.append({
            'utc':             ts,
            'veh_forward_ms2': float(vax),
            'veh_lateral_ms2': float(vay),
            'veh_vertical_ms2':float(vaz),
            'veh_pitch_degs':  float(vgx),
            'veh_roll_degs':   float(vgy),
            'veh_yaw_degs':    float(vgz),
        })
    return pd.DataFrame(rows)


# ── Chart ─────────────────────────────────────────────────────────────────────
'''
def generate_chart(session_start: datetime) -> None:
    nov_csv = _find_novatel_csv(session_start)
    evk_log = EVK / 'EVK-M9DR-log.txt'

    print(f'[chart] NovAtel CSV : {nov_csv}')
    print(f'[chart] EVK log     : {evk_log}')

    nov = _parse_novatel(nov_csv) if nov_csv else pd.DataFrame()
    evk = _parse_evk(evk_log, session_start) if evk_log.exists() else pd.DataFrame()

    if nov.empty and evk.empty:
        print('[chart] No data found — nothing to plot.')
        return

    fig, axes = plt.subplots(3, 2, figsize=(16, 10))
    fig.suptitle(
        f'NovAtel PwrPak7 vs EVK-M9DR — Vehicle Frame\n'
        f'Session: {session_start.strftime("%Y-%m-%d %H:%M:%S UTC")}',
        fontsize=11,
    )

    fmt = mdates.DateFormatter('%H:%M:%S')
    NOV_COLOR = '#58a6ff'
    EVK_COLOR = '#f85149'

    panels = [
        # (row, col, title, ylabel, novatel_col, evk_col)
        (0, 0, 'Accel — Forward (X)',  'm/s²', 'veh_long_acc_ms2',    'veh_forward_ms2'),
        (1, 0, 'Accel — Lateral (Y)',  'm/s²', 'veh_lat_acc_ms2',     'veh_lateral_ms2'),
        (2, 0, 'Accel — Vertical (Z)', 'm/s²', 'veh_vert_acc_ms2',    'veh_vertical_ms2'),
        (0, 1, 'Gyro — Pitch rate',    '°/s',  'veh_pitch_rate_degs', 'veh_pitch_degs'),
        (1, 1, 'Gyro — Roll rate',     '°/s',  'veh_roll_rate_degs',  'veh_roll_degs'),
        (2, 1, 'Gyro — Yaw rate',      '°/s',  'veh_yaw_rate_degs',   'veh_yaw_degs'),
    ]

    for row, col, title, ylabel, ncol, ecol in panels:
        ax = axes[row][col]
        plotted = False
        if not nov.empty and ncol in nov.columns:
            ax.plot(nov['utc'], nov[ncol], label='NovAtel', color=NOV_COLOR, lw=0.9)
            plotted = True
        if not evk.empty and ecol in evk.columns:
            ax.plot(evk['utc'], evk[ecol], label='EVK-M9DR', color=EVK_COLOR, lw=0.9, alpha=0.85)
            plotted = True
        ax.set_title(title, fontsize=9)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.xaxis.set_major_formatter(fmt)
        ax.grid(True, alpha=0.25)
        if plotted:
            ax.legend(fontsize=7)
        plt.setp(ax.xaxis.get_majorticklabels(), rotation=30, ha='right', fontsize=7)

    plt.tight_layout()
    out = HERE / f'comparison_{session_start.strftime("%Y%m%d_%H%M%S")}.png'
    plt.savefig(out, dpi=150, bbox_inches='tight')
    print(f'[chart] Saved → {out}')
    plt.show()
'''

# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    session_start = datetime.now(tz=timezone.utc)

    print(f'[launcher] NovAtel  → http://localhost:{NOVATEL_PORT}')
    nov_proc = subprocess.Popen([PY, 'server.py'], cwd=HERE)

    print(f'[launcher] EVK-M9DR WS  → ws://localhost:{EVK_PORT}')
    evk_proc = subprocess.Popen([PY, 'EVK-M9DR-dashboard.py'], cwd=HERE)

    print(f'[launcher] EVK-M9DR HTTP → http://localhost:{EVK_HTTP_PORT}/EVK-M9DR-dashboard.html')
    evk_http = subprocess.Popen(
        [PY, '-m', 'http.server', str(EVK_HTTP_PORT), '--directory', str(HERE)],
    )

    print('[launcher] All servers running. Press Ctrl+C to stop and generate chart.\n')

    try:
        nov_proc.wait()
    except KeyboardInterrupt:
        print('\n[launcher] Stopping servers...')
        nov_proc.terminate()
        evk_proc.terminate()
        evk_http.terminate()
        try:
            nov_proc.wait(timeout=5)
            evk_proc.wait(timeout=5)
            evk_http.wait(timeout=5)
        except subprocess.TimeoutExpired:
            nov_proc.kill()
            evk_proc.kill()
            evk_http.kill()
        print('[launcher] Generating comparison chart...')
        subprocess.run([PY, str(HERE / 'compare.py'), '--save'], cwd=HERE)


if __name__ == '__main__':
    main()
