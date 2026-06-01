#!/usr/bin/env python3
"""
NovAtel PwrPak7 Real-Time Dashboard Server

Streams:
  RAWIMUSXA      100 Hz  raw IMU counts (sensor frame)
  CORRIMUDATAA   100 Hz  corrected IMU in vehicle body frame (m/s², deg/s)
  INSPVAXA        10 Hz  INS position + attitude
  BESTPOSA         1 Hz  GNSS-only best position  (ASCII — binary is BESTPOSB)
  ITDETECTSTATUSA  1 Hz  spectrum-analysis data (band power / noise per record)
"""

import csv
import datetime
import os
import threading
import time

import serial
from flask import Flask
from flask_socketio import SocketIO

SERIAL_PORT   = '/dev/ttyUSB0'
TARGET_BAUD   = 460800   # operating baud; auto-negotiated from 115200 on first connect
_FALLBACK_BAUD = 115200  # factory default; used only during baud-rate upgrade

app = Flask(__name__, static_folder=None)
socketio = SocketIO(app, cors_allowed_origins='*', async_mode='threading')

_latest = {}   # latest parsed result per event key; replayed to new clients on connect

@socketio.on('connect')
def _on_connect():
    for event, data in _latest.items():
        socketio.emit(event, data)

# ── Logging setup ─────────────────────────────────────────────────────────────

_HERE     = os.path.dirname(os.path.abspath(__file__))
_LOG_DIR  = os.path.join(_HERE, 'logs')
os.makedirs(_LOG_DIR, exist_ok=True)

_SESSION  = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')

# 10 Hz combined log (all sources merged on each INSPVAXA tick)
_CSV_PATH = os.path.join(_LOG_DIR, f'novatel_{_SESSION}.csv')
_LOG_PATH = os.path.join(_LOG_DIR, f'novatel_{_SESSION}.log')

# 100 Hz raw IMU log (RAWIMUSXA + CORRIMUDATAA, every packet)
_IMU_PATH = os.path.join(_LOG_DIR, f'novatel_{_SESSION}_imu100hz.csv')

# Raw serial capture — every line the receiver sends, timestamped
_RAW_PATH = os.path.join(_LOG_DIR, f'novatel_{_SESSION}_raw.log')

CSV_FIELDS = [
    # ── Timing ───────────────────────────────────────────────────────────────
    'sys_datetime', 'unix_timestamp',
    'gps_week', 'gps_seconds',
    # ── INS status ───────────────────────────────────────────────────────────
    'ins_status', 'pose_quality',
    # ── INS position (INSPVAXA) ───────────────────────────────────────────────
    'ins_lat', 'ins_lon', 'ins_height_m', 'ins_undulation_m',
    # ── INS attitude (INSPVAXA) ───────────────────────────────────────────────
    'roll_deg', 'pitch_deg', 'azimuth_deg',
    'roll_sig_deg', 'pitch_sig_deg', 'azimuth_sig_deg',
    # ── INS velocity (INSPVAXA) ───────────────────────────────────────────────
    'vel_n_ms', 'vel_e_ms', 'vel_u_ms', 'speed_2d_ms',
    'vel_n_sig_ms', 'vel_e_sig_ms', 'vel_u_sig_ms',
    # ── Vehicle-frame corrected IMU (CORRIMUDATAA) ────────────────────────────
    'veh_lat_acc_ms2', 'veh_long_acc_ms2', 'veh_vert_acc_ms2',
    'veh_pitch_rate_degs', 'veh_roll_rate_degs', 'veh_yaw_rate_degs',
    'veh_pitch_rate_rads', 'veh_roll_rate_rads', 'veh_yaw_rate_rads',
    # ── Raw sensor-frame IMU counts (RAWIMUSXA) ───────────────────────────────
    'raw_accel_x_lsb', 'raw_accel_y_lsb', 'raw_accel_z_lsb',
    'raw_gyro_x_lsb',  'raw_gyro_y_lsb',  'raw_gyro_z_lsb',
    'raw_accel_x_ms2', 'raw_accel_y_ms2', 'raw_accel_z_ms2',
    'raw_gyro_x_degs', 'raw_gyro_y_degs', 'raw_gyro_z_degs',
    'imu_type', 'imu_status_hex',
    # ── GNSS-only position (BESTPOSA) ─────────────────────────────────────────
    'gnss_lat', 'gnss_lon', 'gnss_height_m', 'gnss_undulation_m',
    'gnss_sol_status', 'gnss_pos_type', 'gnss_sol_quality',
    'gnss_lat_sig_m', 'gnss_lon_sig_m', 'gnss_height_sig_m',
    'gnss_num_svs', 'gnss_num_used',
    'gnss_diff_age_s', 'gnss_sol_age_s',
    # ── INS extended flags (INSPVAXA ext_status) ──────────────────────────────
    'ext_pos_update', 'ext_vel_update', 'ext_att_update',
    'ext_phase_update', 'ext_zero_vel', 'ext_wheel_sensor',
    'ext_heading_update', 'ext_ins_enabled', 'ext_heading_align',
    # ── RF spectrum (ITDETECTSTATUSA) ─────────────────────────────────────────
    'spec_analysis_type', 'spec_band_id',
    'spec_center_freq_hz', 'spec_bandwidth_hz',
    'spec_power_dbm', 'spec_noise_dbm',
]

IMU_FIELDS = [
    'sys_datetime', 'unix_timestamp',
    'raw_accel_x_lsb', 'raw_accel_y_lsb', 'raw_accel_z_lsb',
    'raw_gyro_x_lsb',  'raw_gyro_y_lsb',  'raw_gyro_z_lsb',
    'raw_accel_x_ms2', 'raw_accel_y_ms2', 'raw_accel_z_ms2',
    'raw_gyro_x_degs', 'raw_gyro_y_degs', 'raw_gyro_z_degs',
    'imu_type', 'imu_status_hex',
    'veh_lat_acc_ms2', 'veh_long_acc_ms2', 'veh_vert_acc_ms2',
    'veh_pitch_rate_degs', 'veh_roll_rate_degs', 'veh_yaw_rate_degs',
]

# File handles (opened once, kept for the session)
_csv_fh  = None
_csv_w   = None
_log_fh  = None
_imu_fh  = None
_imu_w   = None
_raw_fh  = None


def _open_logs():
    global _csv_fh, _csv_w, _log_fh, _imu_fh, _imu_w, _raw_fh
    _csv_fh = open(_CSV_PATH, 'a', newline='', buffering=1)
    _csv_w  = csv.DictWriter(_csv_fh, fieldnames=CSV_FIELDS, extrasaction='ignore')
    _csv_w.writeheader()
    _log_fh = open(_LOG_PATH, 'a', buffering=1)
    _imu_fh = open(_IMU_PATH, 'a', newline='', buffering=1)
    _imu_w  = csv.DictWriter(_imu_fh, fieldnames=IMU_FIELDS, extrasaction='ignore')
    _imu_w.writeheader()
    _raw_fh = open(_RAW_PATH, 'a', buffering=1)
    print(f'[log] 10Hz CSV  → {_CSV_PATH}')
    print(f'[log] 10Hz text → {_LOG_PATH}')
    print(f'[log] 100Hz IMU → {_IMU_PATH}')
    print(f'[log] Raw serial → {_RAW_PATH}')


def _raw(line: str):
    """Stamp and write every serial line to the raw capture."""
    if _raw_fh:
        _raw_fh.write(f'[{datetime.datetime.now().strftime("%H:%M:%S.%f")}] {line}')


def _fmt(v, spec='+.4f'):
    try:
        return format(float(v), spec)
    except (TypeError, ValueError):
        return str(v) if v not in (None, '') else '—'


def _write_imu_row(latest):
    """Write one row to the 100 Hz IMU CSV."""
    if _imu_w is None:
        return
    now  = datetime.datetime.now()
    imu  = latest.get('imu')  or {}
    corr = latest.get('corr') or {}
    _imu_w.writerow({
        'sys_datetime':       now.strftime('%Y-%m-%d %H:%M:%S.%f'),
        'unix_timestamp':     f'{now.timestamp():.6f}',
        'raw_accel_x_lsb':   imu.get('accel_x', ''),
        'raw_accel_y_lsb':   imu.get('accel_y', ''),
        'raw_accel_z_lsb':   imu.get('accel_z', ''),
        'raw_gyro_x_lsb':    imu.get('gyro_x', ''),
        'raw_gyro_y_lsb':    imu.get('gyro_y', ''),
        'raw_gyro_z_lsb':    imu.get('gyro_z', ''),
        'raw_accel_x_ms2':   imu.get('accel_x_ms2', ''),
        'raw_accel_y_ms2':   imu.get('accel_y_ms2', ''),
        'raw_accel_z_ms2':   imu.get('accel_z_ms2', ''),
        'raw_gyro_x_degs':   imu.get('gyro_x_degs', ''),
        'raw_gyro_y_degs':   imu.get('gyro_y_degs', ''),
        'raw_gyro_z_degs':   imu.get('gyro_z_degs', ''),
        'imu_type':           imu.get('imu_type', ''),
        'imu_status_hex':     imu.get('status', ''),
        'veh_lat_acc_ms2':    corr.get('lat_acc', ''),
        'veh_long_acc_ms2':   corr.get('long_acc', ''),
        'veh_vert_acc_ms2':   corr.get('vert_acc', ''),
        'veh_pitch_rate_degs':corr.get('pitch_rate_degs', ''),
        'veh_roll_rate_degs': corr.get('roll_rate_degs', ''),
        'veh_yaw_rate_degs':  corr.get('yaw_rate_degs', ''),
    })


def _write_main_row(latest):
    """Write one row to the 10 Hz combined CSV and text log (called on INS tick)."""
    if _csv_w is None:
        return
    now  = datetime.datetime.now()
    ins       = latest.get('ins')       or {}
    ins_state = latest.get('ins_state') or {}
    corr = latest.get('corr') or {}
    imu  = latest.get('imu')  or {}
    gnss = latest.get('gnss') or {}
    cal  = latest.get('cal')  or {}

    # Use full INSPVAXA status when available; fall back to INSSTATUSA label
    # so the column is never blank even when the INS hasn't started aligning.
    ins_status_str = ins.get('pose_label', '') or ins_state.get('pose_label', '')
    ins_quality    = ins.get('pose_quality', '') or (
        'warn' if ins_status_str in WARN_INS else
        'bad'  if ins_status_str else ''
    )

    try:
        spd = (float(ins.get('vel_n', 0))**2 + float(ins.get('vel_e', 0))**2) ** 0.5
    except (TypeError, ValueError):
        spd = ''

    row = {
        'sys_datetime':        now.strftime('%Y-%m-%d %H:%M:%S.%f'),
        'unix_timestamp':      f'{now.timestamp():.6f}',
        'gps_week':            ins.get('week', '') or gnss.get('week', ''),
        'gps_seconds':         ins.get('seconds', '') or gnss.get('seconds', ''),
        'ins_status':          ins_status_str,
        'pose_quality':        ins_quality,
        'ins_lat':             ins.get('lat', ''),
        'ins_lon':             ins.get('lon', ''),
        'ins_height_m':        ins.get('height', ''),
        'ins_undulation_m':    ins.get('undulation', ''),
        'roll_deg':            ins.get('roll', ''),
        'pitch_deg':           ins.get('pitch', ''),
        'azimuth_deg':         ins.get('azimuth', ''),
        'roll_sig_deg':        ins.get('roll_sig', ''),
        'pitch_sig_deg':       ins.get('pitch_sig', ''),
        'azimuth_sig_deg':     ins.get('azimuth_sig', ''),
        'vel_n_ms':            ins.get('vel_n', ''),
        'vel_e_ms':            ins.get('vel_e', ''),
        'vel_u_ms':            ins.get('vel_u', ''),
        'speed_2d_ms':         f'{spd:.4f}' if spd != '' else '',
        'vel_n_sig_ms':        ins.get('vel_n_sig', ''),
        'vel_e_sig_ms':        ins.get('vel_e_sig', ''),
        'vel_u_sig_ms':        ins.get('vel_u_sig', ''),
        'veh_lat_acc_ms2':     corr.get('lat_acc', ''),
        'veh_long_acc_ms2':    corr.get('long_acc', ''),
        'veh_vert_acc_ms2':    corr.get('vert_acc', ''),
        'veh_pitch_rate_degs': corr.get('pitch_rate_degs', ''),
        'veh_roll_rate_degs':  corr.get('roll_rate_degs', ''),
        'veh_yaw_rate_degs':   corr.get('yaw_rate_degs', ''),
        'veh_pitch_rate_rads': corr.get('pitch_rate_rads', ''),
        'veh_roll_rate_rads':  corr.get('roll_rate_rads', ''),
        'veh_yaw_rate_rads':   corr.get('yaw_rate_rads', ''),
        'raw_accel_x_lsb':    imu.get('accel_x', ''),
        'raw_accel_y_lsb':    imu.get('accel_y', ''),
        'raw_accel_z_lsb':    imu.get('accel_z', ''),
        'raw_gyro_x_lsb':     imu.get('gyro_x', ''),
        'raw_gyro_y_lsb':     imu.get('gyro_y', ''),
        'raw_gyro_z_lsb':     imu.get('gyro_z', ''),
        'raw_accel_x_ms2':    imu.get('accel_x_ms2', ''),
        'raw_accel_y_ms2':    imu.get('accel_y_ms2', ''),
        'raw_accel_z_ms2':    imu.get('accel_z_ms2', ''),
        'raw_gyro_x_degs':    imu.get('gyro_x_degs', ''),
        'raw_gyro_y_degs':    imu.get('gyro_y_degs', ''),
        'raw_gyro_z_degs':    imu.get('gyro_z_degs', ''),
        'imu_type':            imu.get('imu_type', ''),
        'imu_status_hex':      imu.get('status', ''),
        'gnss_lat':            gnss.get('lat', ''),
        'gnss_lon':            gnss.get('lon', ''),
        'gnss_height_m':       gnss.get('height', ''),
        'gnss_undulation_m':   gnss.get('undulation', ''),
        'gnss_sol_status':     gnss.get('sol_status', ''),
        'gnss_pos_type':       gnss.get('pos_type', ''),
        'gnss_sol_quality':    gnss.get('sol_quality', ''),
        'gnss_lat_sig_m':      gnss.get('lat_sig', ''),
        'gnss_lon_sig_m':      gnss.get('lon_sig', ''),
        'gnss_height_sig_m':   gnss.get('height_sig', ''),
        'gnss_num_svs':        gnss.get('num_svs', ''),
        'gnss_num_used':       gnss.get('num_used', ''),
        'gnss_diff_age_s':     gnss.get('diff_age', ''),
        'gnss_sol_age_s':      gnss.get('sol_age', ''),
        'ext_pos_update':      int(bool(ins.get('ext_pos_update'))),
        'ext_vel_update':      int(bool(ins.get('ext_vel_update'))),
        'ext_att_update':      int(bool(ins.get('ext_att_update'))),
        'ext_phase_update':    int(bool(ins.get('ext_phase_update'))),
        'ext_zero_vel':        int(bool(ins.get('ext_zero_vel'))),
        'ext_wheel_sensor':    int(bool(ins.get('ext_wheel_sensor'))),
        'ext_heading_update':  int(bool(ins.get('ext_heading_update'))),
        'ext_ins_enabled':     int(bool(ins.get('ext_ins_enabled'))),
        'ext_heading_align':   int(bool(ins.get('ext_heading_align'))),
        'spec_analysis_type':  cal.get('analysis_type', ''),
        'spec_band_id':        cal.get('band_id', ''),
        'spec_center_freq_hz': cal.get('center_freq', ''),
        'spec_bandwidth_hz':   cal.get('bandwidth', ''),
        'spec_power_dbm':      cal.get('power_dbm', ''),
        'spec_noise_dbm':      cal.get('noise_dbm', ''),
    }
    _csv_w.writerow(row)

    # ── Human-readable line ───────────────────────────────────────────────────
    status    = ins.get('pose_label', 'INACTIVE')
    spd_str   = f'{spd:.3f}' if isinstance(spd, float) else '—'
    log_line = (
        f"[{now.strftime('%H:%M:%S.%f')}] {status:<30} | "
        f"VehAccel(m/s²) X:{_fmt(corr.get('lat_acc'))} Y:{_fmt(corr.get('long_acc'))} Z:{_fmt(corr.get('vert_acc'))}  "
        f"VehGyro(°/s) X:{_fmt(corr.get('pitch_rate_degs'))} Y:{_fmt(corr.get('roll_rate_degs'))} Z:{_fmt(corr.get('yaw_rate_degs'))} | "
        f"RawIMU(m/s²) Ax:{_fmt(imu.get('accel_x_ms2'))} Ay:{_fmt(imu.get('accel_y_ms2'))} Az:{_fmt(imu.get('accel_z_ms2'))} "
        f"Gyro(°/s) Gx:{_fmt(imu.get('gyro_x_degs'))} Gy:{_fmt(imu.get('gyro_y_degs'))} Gz:{_fmt(imu.get('gyro_z_degs'))} | "
        f"Roll:{_fmt(ins.get('roll'),'+.2f')}°(σ{_fmt(ins.get('roll_sig'),'.3f')}) "
        f"Pitch:{_fmt(ins.get('pitch'),'+.2f')}°(σ{_fmt(ins.get('pitch_sig'),'.3f')}) "
        f"Hdg:{_fmt(ins.get('azimuth'),'+.2f')}°(σ{_fmt(ins.get('azimuth_sig'),'.3f')}) | "
        f"Vel N:{_fmt(ins['vel_n']*3.6 if 'vel_n' in ins else None,'+.3f')} "
        f"E:{_fmt(ins['vel_e']*3.6 if 'vel_e' in ins else None,'+.3f')} "
        f"U:{_fmt(ins['vel_u']*3.6 if 'vel_u' in ins else None,'+.3f')} "
        f"2D:{f'{spd*3.6:.3f}' if isinstance(spd, float) else '—'} km/h | "
        f"INS {_fmt(ins.get('lat'),'.7f')},{_fmt(ins.get('lon'),'.7f')} H:{_fmt(ins.get('height'),'.3f')}m | "
        f"GNSS {_fmt(gnss.get('lat'),'.7f')},{_fmt(gnss.get('lon'),'.7f')} "
        f"σLat:{_fmt(gnss.get('lat_sig'),'.4f')}m σLon:{_fmt(gnss.get('lon_sig'),'.4f')}m "
        f"SVs:{gnss.get('num_svs','—')}/{gnss.get('num_used','—')} {gnss.get('sol_status','—')} {gnss.get('pos_type','—')}"
    )
    print(log_line)
    if _log_fh:
        _log_fh.write(log_line + '\n')

# ── Column definitions ────────────────────────────────────────────────────────

RAWIMUSXA_COLS = [
    'head', 'week_num1', 'seconds1', 'imu_info', 'week_num2', 'seconds2',
    'status', 'accel_z', 'neg_accel_y', 'accel_x', 'gyro_z', 'neg_gyro_y', 'gyro_x', 'crc',
]

INSPVAXA_COLS = [
    'head', 'port', 'sequence', 'idle_perc', 'time_status', 'week', 'seconds',
    'receiver_status', 'reserved', 'sw_version', 'pose_type', 'pos_type', 'lat', 'lon',
    'height', 'undulation', 'vel_n', 'vel_e', 'vel_u', 'roll', 'pitch',
    'azimuth', 'lat_sig', 'lon_sig', 'height_sig', 'vel_n_sig', 'vel_e_sig',
    'vel_u_sig', 'roll_sig', 'pitch_sig', 'azimuth_sig', 'ext_status', 'crc',
]

BESTPOSA_COLS = [
    'head', 'port', 'sequence', 'idle_perc', 'time_status', 'week', 'seconds',
    'receiver_status', 'reserved', 'sw_version', 'sol_status', 'pos_type',
    'lat', 'lon', 'height', 'undulation', 'datum',
    'lat_sig', 'lon_sig', 'height_sig', 'stn_id',
    'diff_age', 'sol_age', 'num_svs', 'num_used',
    'num_above_mask', 'num_above_filter', 'reserved2',
    'ext_sol_status', 'galileo_multi', 'gps_multi', 'crc',
]

# CORRIMUDATAA: corrected IMU in vehicle body frame
# Fields after header semicolon: pitch_rate, roll_rate, yaw_rate,
#   lateral_acc, longitudinal_acc, vertical_acc
CORRIMUDATA_COLS = [
    'head', 'port', 'sequence', 'idle_perc', 'time_status', 'week', 'seconds',
    'receiver_status', 'reserved', 'sw_version',
    'data_week', 'data_seconds',
    'pitch_rate', 'roll_rate', 'yaw_rate',
    'lat_acc', 'long_acc', 'vert_acc', 'crc',
]

GOOD_INS  = {'INS_SOLUTION_GOOD', 'INS_ALIGNMENT_COMPLETE'}
WARN_INS  = {'INS_ALIGNING', 'INS_DETERMINING_ORIENTATION',
             'INS_HIGH_VARIANCE', 'INS_WAITING_INITIALPOS'}
GOOD_GNSS = {'SOL_COMPUTED'}
WARN_GNSS = {'PENDING', 'COLD_START', 'INTEGRITY_WARNING'}

RAD2DEG = 57.295779513

# IMU type → (accel_scale m/s²/LSB, gyro_scale deg/s/LSB)
# Type 61 = Epson G370N  (NovAtel OEM7 documentation)
IMU_SCALES = {
    61: ((0.400 / 65536) * (9.80665 / 1000), 0.0151515 / 65536),
}
_DEFAULT_SCALE = IMU_SCALES[61]


def _imu_scale(imu_type):
    try:
        return IMU_SCALES.get(int(imu_type), _DEFAULT_SCALE)
    except (TypeError, ValueError):
        return _DEFAULT_SCALE

# ── Helpers ───────────────────────────────────────────────────────────────────

def _find_msg(line, tag):
    for prefix in ('%', '#'):
        idx = line.find(f'{prefix}{tag}')
        if idx >= 0:
            return line[idx:]
    return None


def _split_header_body(fields, idx=9):
    """Split field[idx] on the NovAtel header/body semicolon."""
    if len(fields) > idx and ';' in fields[idx]:
        left, right = fields[idx].split(';', 1)
        fields[idx:idx + 1] = [left, right]
    return fields


def _strip_crc(fields, expected):
    """Expand the *CRC-bearing field so len(fields) == expected."""
    if len(fields) == expected - 1:
        last = fields[-1]
        if '*' in last:
            val, crc = last.split('*', 1)
            fields[-1:] = [val, crc]
    elif len(fields) >= expected:
        fields[expected - 1] = fields[expected - 1].split('*')[0]
    return fields


# ── Parsers ───────────────────────────────────────────────────────────────────

def parse_rawimusxa(line):
    msg = _find_msg(line, 'RAWIMUSXA')
    if not msg:
        return None
    fields = msg.strip().split(',')
    if len(fields) >= 13:
        fields[12:13] = fields[12].split('*')
    if len(fields) != len(RAWIMUSXA_COLS):
        return None
    d = dict(zip(RAWIMUSXA_COLS, fields))
    gps_sec_raw = d['seconds1'].split(';')[0]
    try:
        imu_type = d.get('imu_info', '?')
        ax = int(d['accel_x']);  ay = -int(d['neg_accel_y']); az = int(d['accel_z'])
        gx = int(d['gyro_x']);   gy = -int(d['neg_gyro_y']);  gz = int(d['gyro_z'])
        as_, gs = _imu_scale(imu_type)
        return {
            'week':       int(d.get('week_num2', d.get('week_num1', 0))),
            'seconds':    float(d.get('seconds2', gps_sec_raw)),
            'imu_type':   imu_type,
            'status':     d.get('status', '0'),
            'accel_x': ax, 'accel_y': ay, 'accel_z': az,
            'gyro_x':  gx, 'gyro_y':  gy, 'gyro_z':  gz,
            'accel_x_ms2': ax * as_, 'accel_y_ms2': ay * as_, 'accel_z_ms2': az * as_,
            'gyro_x_degs': gx * gs,  'gyro_y_degs': gy * gs,  'gyro_z_degs': gz * gs,
        }
    except (ValueError, KeyError):
        return None


def parse_corrimudata(line):
    """
    Corrected IMU in vehicle body frame.
    CORRIMUDATAA outputs delta angles (rad/sample) and delta velocities
    (m/s per sample), NOT instantaneous rates. Divide by IMU_DT to get
    true rates (rad/s, m/s²).
    """
    msg = _find_msg(line, 'CORRIMUDATAA')
    if not msg:
        return None
    fields = msg.strip().split(',')
    fields = _split_header_body(fields)
    fields = _strip_crc(fields, len(CORRIMUDATA_COLS))
    if len(fields) != len(CORRIMUDATA_COLS):
        return None
    d = dict(zip(CORRIMUDATA_COLS, fields))
    try:
        IMU_DT = 0.01  # 100 Hz → 0.01 s per sample
        pr = float(d['pitch_rate']) / IMU_DT
        rr = float(d['roll_rate'])  / IMU_DT
        yr = float(d['yaw_rate'])   / IMU_DT
        return {
            'week':      d.get('week', '0'),
            'seconds':   float(d.get('seconds', 0)),
            'pitch_rate_rads': pr,
            'roll_rate_rads':  rr,
            'yaw_rate_rads':   yr,
            'pitch_rate_degs': pr * RAD2DEG,
            'roll_rate_degs':  rr * RAD2DEG,
            'yaw_rate_degs':   yr * RAD2DEG,
            'lat_acc':    float(d['lat_acc'])  / IMU_DT,
            'long_acc':   float(d['long_acc']) / IMU_DT,
            'vert_acc':   float(d['vert_acc']) / IMU_DT,
        }
    except (ValueError, KeyError):
        return None


def parse_inspvaxa(line):
    msg = _find_msg(line, 'INSPVAXA')
    if not msg:
        return None
    fields = msg.strip().split(',')
    fields = _split_header_body(fields)
    fields = _strip_crc(fields, len(INSPVAXA_COLS))
    if len(fields) != len(INSPVAXA_COLS):
        return None
    d = dict(zip(INSPVAXA_COLS, fields))
    try:
        pose_label = d.get('pose_type', 'INS_INACTIVE').strip()
        ext = int(d.get('ext_status', '0'), 16)
        return {
            'week':          d.get('week', '0'),
            'seconds':       float(d.get('seconds', 0)),
            'time_status':   d.get('time_status', ''),
            'pose_label':    pose_label,
            'pose_quality':  ('good' if pose_label in GOOD_INS
                              else 'warn' if pose_label in WARN_INS else 'bad'),
            'lat':           float(d['lat']),
            'lon':           float(d['lon']),
            'height':        float(d['height']),
            'undulation':    float(d['undulation']),
            'vel_n':         float(d['vel_n']),
            'vel_e':         float(d['vel_e']),
            'vel_u':         float(d['vel_u']),
            'roll':          float(d['roll']),
            'pitch':         float(d['pitch']),
            'azimuth':       float(d['azimuth']),
            'lat_sig':       float(d['lat_sig']),
            'lon_sig':       float(d['lon_sig']),
            'height_sig':    float(d['height_sig']),
            'vel_n_sig':     float(d['vel_n_sig']),
            'vel_e_sig':     float(d['vel_e_sig']),
            'vel_u_sig':     float(d['vel_u_sig']),
            'roll_sig':      float(d['roll_sig']),
            'pitch_sig':     float(d['pitch_sig']),
            'azimuth_sig':   float(d['azimuth_sig']),
            'ext_pos_update':     bool(ext & (1 << 0)),
            'ext_vel_update':     bool(ext & (1 << 1)),
            'ext_att_update':     bool(ext & (1 << 2)),
            'ext_phase_update':   bool(ext & (1 << 3)),
            'ext_zero_vel':       bool(ext & (1 << 4)),
            'ext_wheel_sensor':   bool(ext & (1 << 5)),
            'ext_heading_update': bool(ext & (1 << 6)),
            'ext_ins_enabled':    bool(ext & (1 << 8)),
            'ext_heading_align':  bool(ext & (1 << 9)),
        }
    except (ValueError, KeyError):
        return None


def parse_bestposa(line):
    # NOTE: 'A' suffix = ASCII text. 'B' suffix = binary (used by App Suite).
    # Both carry identical data — we request A because we parse text.
    msg = _find_msg(line, 'BESTPOSA')
    if not msg:
        return None
    fields = msg.strip().split(',')
    fields = _split_header_body(fields)
    fields = _strip_crc(fields, len(BESTPOSA_COLS))
    if len(fields) != len(BESTPOSA_COLS):
        return None
    d = dict(zip(BESTPOSA_COLS, fields))
    try:
        sol  = d.get('sol_status', '').strip()
        ptype = d.get('pos_type', '').strip()
        return {
            'week':        d.get('week', '0'),
            'seconds':     float(d.get('seconds', 0)),
            'sol_status':  sol,
            'pos_type':    ptype,
            'sol_quality': ('good' if sol in GOOD_GNSS else
                            'warn' if sol in WARN_GNSS else 'bad'),
            'lat':         float(d['lat']),
            'lon':         float(d['lon']),
            'height':      float(d['height']),
            'undulation':  float(d['undulation']),
            'lat_sig':     float(d['lat_sig']),
            'lon_sig':     float(d['lon_sig']),
            'height_sig':  float(d['height_sig']),
            'num_svs':     int(d['num_svs']),
            'num_used':    int(d['num_used']),
            'diff_age':    float(d['diff_age']),
            'sol_age':     float(d['sol_age']),
        }
    except (ValueError, KeyError):
        return None


def parse_itdetect(line):
    """
    ITDETECTSTATUSA — actual format is spectrum-analysis data per axis.
    Body fields (after semicolon): num_records, band_id, analysis_type,
      center_freq_hz, bandwidth_hz, power_dbm, noise_dbm, status_hex, ...
    We just surface the first record's detect_status flag word.
    """
    msg = _find_msg(line, 'ITDETECTSTATUSA')
    if not msg:
        return None
    fields = msg.strip().split(',')
    fields = _split_header_body(fields)
    for i, f in enumerate(fields):
        if '*' in f:
            fields[i] = f.split('*')[0]
            break
    # field[10] = num_records, field[11] = band_id, field[12] = analysis_type
    # field[17] = status hex (8 chars)
    def _s(i): return fields[i].strip() if len(fields) > i else '—'
    def _f(i):
        try:
            return float(fields[i]) if len(fields) > i else 0.0
        except ValueError:
            return 0.0
    try:
        return {
            'num_records':    _s(10),
            'band_id':        _s(11),
            'analysis_type':  _s(12),
            'center_freq':    _f(13),
            'bandwidth':      _f(14),
            'power_dbm':      _f(15),
            'noise_dbm':      _f(16),
            'status_hex':     _s(17),
        }
    except (ValueError, IndexError):
        return None


def parse_insconfiga(line):
    """
    INSCONFIGA — current SPAN configuration: lever arm (ANT1) and body rotation (RBV).
    Body format: imu_str, imu_id, ..., profile, status, align_mode, ..., num_translations,
      <type>, <frame>, x, y, z, xunc, yunc, zunc, FROM_NVM, ... num_rotations,
      <type>, <frame>, rx, ry, rz, rxunc, ryunc, rzunc, FROM_NVM
    ANT1/RBV records are located by name, not fixed offset.
    """
    msg = _find_msg(line, 'INSCONFIGA')
    if not msg:
        return None
    try:
        body   = msg.split(';', 1)[1].split('*')[0]
        fields = [f.strip() for f in body.split(',')]

        result = {
            'imu_type':   fields[0],
            'profile':    '',
            'align_mode': '',
            'ant1_x': 0.0, 'ant1_y': 0.0, 'ant1_z': 0.0,
            'ant1_x_unc': 0.0, 'ant1_y_unc': 0.0, 'ant1_z_unc': 0.0,
            'rbv_x':  0.0, 'rbv_y':  0.0, 'rbv_z':  0.0,
            'rbv_x_unc':  0.0, 'rbv_y_unc':  0.0, 'rbv_z_unc':  0.0,
        }

        # Fixed header fields: imu_str, imu_id, ?, align_vel, profile, status, align_mode
        if len(fields) > 4:
            result['profile']    = fields[4]
        if len(fields) > 6:
            result['align_mode'] = fields[6]

        # Search for ANT1 and RBV records by label anywhere in the field list.
        # Each record: label, frame, x, y, z, xunc, yunc, zunc, source (9 fields)
        for i, f in enumerate(fields):
            if f == 'ANT1' and i + 8 < len(fields):
                result['ant1_x']     = float(fields[i + 2])
                result['ant1_y']     = float(fields[i + 3])
                result['ant1_z']     = float(fields[i + 4])
                result['ant1_x_unc'] = float(fields[i + 5])
                result['ant1_y_unc'] = float(fields[i + 6])
                result['ant1_z_unc'] = float(fields[i + 7])
            elif f == 'RBV' and i + 8 < len(fields):
                result['rbv_x']     = float(fields[i + 2])
                result['rbv_y']     = float(fields[i + 3])
                result['rbv_z']     = float(fields[i + 4])
                result['rbv_x_unc'] = float(fields[i + 5])
                result['rbv_y_unc'] = float(fields[i + 6])
                result['rbv_z_unc'] = float(fields[i + 7])

        return result
    except (ValueError, IndexError):
        return None


def parse_psrdopa(line):
    msg = _find_msg(line, 'PSRDOPA')
    if not msg:
        return None
    try:
        body = msg.split(';', 1)[1]
        if '*' in body:
            body = body[:body.rfind('*')]
        f = [x.strip() for x in body.split(',')]
        return {
            'gdop': float(f[0]),
            'pdop': float(f[1]),
            'hdop': float(f[2]),
            'vdop': float(f[3]),
        }
    except (ValueError, IndexError):
        return None


def parse_insstatusa(line):
    """
    INSSTATUSA — outputs INS state machine status even when INS is fully inactive.
    Used to populate ins_status in GPS-only log rows when INSPVAXA isn't flowing.
    Body after semicolon: ins_status, pos_type, ext_status
    """
    msg = _find_msg(line, 'INSSTATUSA')
    if not msg:
        return None
    try:
        body = msg.split(';', 1)[1]
        fields = body.split(',')
        status = fields[0].strip()
        return {'pose_label': status}
    except (IndexError, ValueError):
        return None


# ── Dispatch table ────────────────────────────────────────────────────────────
# Ordered by frequency (high → low) so the fast path skips rarely-matched tags
PARSERS = [
    ('RAWIMUSXA',       parse_rawimusxa,    'imu'),
    ('CORRIMUDATAA',    parse_corrimudata,  'corr'),
    ('INSPVAXA',        parse_inspvaxa,     'ins'),
    ('INSSTATUSA',      parse_insstatusa,   'ins_state'),
    ('INSCONFIGA',      parse_insconfiga,   'inscfg'),
    ('BESTPOSA',        parse_bestposa,     'gnss'),
    ('ITDETECTSTATUSA', parse_itdetect,     'cal'),
    ('PSRDOPA',         parse_psrdopa,      'dop'),
]

# One-shot diagnostic queries sent immediately on connect.
# Responses land in the raw log so we can see firmware, INS config, and
# receiver status without needing the unit to be aligned or moving.
DIAG_CMDS = [
    'log versiona once\r\n',          # firmware version + model
    'log rxstatusa once\r\n',         # receiver error/status word
    'log insconfiga once\r\n',        # current SPAN config: lever arm, rotation, mode
    'log insstatusa once\r\n',        # current INS state machine status
    'log bestposa once\r\n',          # immediate GPS fix snapshot
]

LOG_CMDS = [
    'log rawimusxa ontime 0.01\r\n',       # 100 Hz  raw sensor-frame counts
    'log corrimudataa ontime 0.01\r\n',    # 100 Hz  corrected vehicle-frame
    'log inspvaxa ontime 1\r\n',           #   1 Hz  INS attitude + velocity
    'log insstatusa ontime 1\r\n',         #   1 Hz  INS state machine status (even when inactive)
    'log insconfiga onchanged\r\n',        # on change  installation lever arm + body rotation
    'log bestposa ontime 1\r\n',           #   1 Hz  GNSS-only position (ASCII)
    'log itdetectstatusa ontime 1\r\n',    #   1 Hz  RF spectrum
    'log psrdopa ontime 1\r\n',            #   1 Hz  DOP values (HDOP, PDOP, VDOP, GDOP)
]


# ── Serial reader thread ──────────────────────────────────────────────────────

def _probe_serial(port, baud, probe_cmd=b'log versiona once\r\n', timeout=2.0):
    """Open port at baud, send probe_cmd, return True if any response arrives within timeout."""
    try:
        with serial.Serial(port, baud, timeout=0.1) as ser:
            ser.write(probe_cmd)
            deadline = time.time() + timeout
            while time.time() < deadline:
                chunk = ser.read(ser.in_waiting or 1)
                if chunk and chunk.strip():
                    return True
    except serial.SerialException:
        pass
    return False


def serial_reader():
    global _latest

    connect_baud = TARGET_BAUD

    while True:
        try:
            with serial.Serial(SERIAL_PORT, connect_baud, timeout=1) as ser:
                # ── Baud-rate negotiation ──────────────────────────────────────
                # After a power cycle the PwrPak7 resets to 115200. We try
                # TARGET_BAUD first; if nothing arrives within 2 s we reconnect
                # at 115200 and send the COM upgrade command, then loop back to
                # open at TARGET_BAUD for normal operation.
                if connect_baud == TARGET_BAUD:
                    ser.write(b'log versiona once\r\n')
                    t0 = time.time()
                    got_data = False
                    while time.time() - t0 < 2.0:
                        chunk = ser.read(ser.in_waiting or 1)
                        if chunk and chunk.strip():
                            got_data = True
                            break
                    if not got_data:
                        print(f'[serial] No response at {TARGET_BAUD}; upgrading from {_FALLBACK_BAUD}')
                        connect_baud = _FALLBACK_BAUD
                        continue   # reopen at _FALLBACK_BAUD

                if connect_baud == _FALLBACK_BAUD:
                    # Issue baud-rate change — device switches immediately.
                    # Do NOT send saveconfig so device resets cleanly on power cycle.
                    cmd = f'com usb1 {TARGET_BAUD} n 8 1 n off on\r\n'
                    ser.write(cmd.encode())
                    print(f'[serial] → {cmd.strip()}')
                    time.sleep(0.5)
                    connect_baud = TARGET_BAUD
                    print(f'[serial] Reconnecting at {TARGET_BAUD} baud')
                    continue   # reopen at TARGET_BAUD

                # ── Normal init at TARGET_BAUD ─────────────────────────────────
                print(f'[serial] Connected to {SERIAL_PORT} at {connect_baud} baud')
                socketio.emit('conn', {'ok': True, 'port': SERIAL_PORT})

                # Stop all existing logs (including any persisted binary logs
                # from previous sessions that would corrupt our ASCII stream).
                ser.write(b'unlogall\r\n')
                print('[serial] → unlogall')
                time.sleep(0.5)          # let the binary stream drain
                ser.reset_input_buffer() # discard buffered binary garbage

                # Send diagnostic one-shots first so their responses land at
                # the top of the raw log before the streaming data begins.
                for cmd in DIAG_CMDS:
                    ser.write(cmd.encode())
                    print(f'[serial] → {cmd.strip()}')
                    time.sleep(0.15)   # longer gap — let each response arrive

                for cmd in LOG_CMDS:
                    ser.write(cmd.encode())
                    print(f'[serial] → {cmd.strip()}')
                    time.sleep(0.05)

                while True:
                    raw = ser.readline()
                    if not raw:
                        continue
                    line = raw.decode('ascii', errors='ignore')
                    _raw(line)   # capture every line regardless of parse result
                    for tag, parser, event in PARSERS:
                        if tag not in line:
                            continue
                        result = parser(line)
                        if result:
                            _latest[event] = result
                            socketio.emit(event, result)
                            # 100 Hz IMU log: write on every raw + corrected IMU packet
                            if event in ('imu', 'corr'):
                                _write_imu_row(_latest)
                            # 10 Hz combined log: write on every INS packet
                            elif event == 'ins':
                                _write_main_row(_latest)
                            # 1 Hz fallback: write GPS row when INS is not running
                            elif event == 'gnss' and not _latest.get('ins'):
                                _write_main_row(_latest)
                            # Log installation config to text log each time it arrives
                            elif event == 'inscfg' and _log_fh:
                                cfg = result
                                _log_fh.write(
                                    f"INSCONFIG  "
                                    f"Profile:{cfg.get('profile','')}  "
                                    f"AlignMode:{cfg.get('align_mode','')}  "
                                    f"ANT1(m) X:{cfg.get('ant1_x',0):.4f} Y:{cfg.get('ant1_y',0):.4f} Z:{cfg.get('ant1_z',0):.4f} "
                                    f"(±{cfg.get('ant1_x_unc',0):.4f} ±{cfg.get('ant1_y_unc',0):.4f} ±{cfg.get('ant1_z_unc',0):.4f})  "
                                    f"RBV(°) X:{cfg.get('rbv_x',0):.4f} Y:{cfg.get('rbv_y',0):.4f} Z:{cfg.get('rbv_z',0):.4f} "
                                    f"(±{cfg.get('rbv_x_unc',0):.4f} ±{cfg.get('rbv_y_unc',0):.4f} ±{cfg.get('rbv_z_unc',0):.4f})\n"
                                )
                        break

        except serial.SerialException as exc:
            print(f'[serial] Error: {exc}')
            socketio.emit('conn', {'ok': False, 'error': str(exc)})
            connect_baud = TARGET_BAUD   # reset to target on reconnect
            time.sleep(3)


# ── HTTP ──────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    here = os.path.dirname(os.path.abspath(__file__))
    with open(os.path.join(here, 'dashboard.html'), encoding='utf-8') as f:
        return f.read()


if __name__ == '__main__':
    _open_logs()
    t = threading.Thread(target=serial_reader, daemon=True)
    t.start()
    print('Dashboard → http://localhost:5000')
    socketio.run(app, host='0.0.0.0', port=5000, debug=False,
                 allow_unsafe_werkzeug=True)
