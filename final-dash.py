#!/usr/bin/env python3
"""
final-dash.py — EVK-M9DR dashboard + NovAtel PwrPak7 IMU recorder.

Runs:
  • WebSocket server (ws://localhost:5052) — streams all parsed messages to the browser
  • EVK-M9DR serial reader (async, /dev/ttyACM0) — ESF-MEAS, NAV-PVT, NAV-ATT, etc.
  • NovAtel PwrPak7 serial reader (thread, /dev/ttyUSB0) — RAWIMUSXA at 100 Hz

Usage:
    python3 final-dash.py
    Open EVK-M9DR-dashboard.html in your browser.
"""
import asyncio
import csv
import json
import os
import threading
import time
import serial
import websockets
from datetime import datetime, timedelta, timezone
from pyubx2 import UBXReader, UBXMessage, UBX_PROTOCOL, NMEA_PROTOCOL, SET_LAYER_RAM, TXN_NONE

# ── Ports & WebSocket ─────────────────────────────────────────────────────────

EVK_PORT = '/dev/ttyACM0'
EVK_BAUD = 115200
NOV_PORT = '/dev/ttyUSB0'
NOV_BAUD = 460800
WS_HOST  = 'localhost'
WS_PORT  = 5052

# ── Log paths ─────────────────────────────────────────────────────────────────

_HERE     = os.path.dirname(os.path.abspath(__file__))
_SESSION  = datetime.now().strftime('%Y%m%d_%H%M%S')
_EVK_DIR  = os.path.join(_HERE, 'logs', 'EVK')
_NOV_DIR  = os.path.join(_HERE, 'logs', 'Novatel')
os.makedirs(_EVK_DIR, exist_ok=True)
os.makedirs(_NOV_DIR, exist_ok=True)

LOG_FILE  = os.path.join(_EVK_DIR, f'evk_{_SESSION}.txt')
_IMU_PATH = os.path.join(_EVK_DIR, f'evk_{_SESSION}_imu.csv')
_NAV_PATH = os.path.join(_EVK_DIR, f'evk_{_SESSION}_nav.csv')
_NOV_PATH = os.path.join(_NOV_DIR, f'novatel_{_SESSION}_imu100hz.csv')

# ── Scale factors ─────────────────────────────────────────────────────────────

EVK_ACCEL_SCALE = 1 / 1024.0
EVK_GYRO_SCALE  = 1 / 4096.0
NOV_ACCEL_SCALE = (0.400 / 65536) * (9.80665 / 1000)
NOV_GYRO_SCALE  = 0.0151515 / 65536
EMA_ALPHA       = 0.2

# ── Shared state ──────────────────────────────────────────────────────────────

FUSION_NAMES = {0: 'Initializing', 1: 'Fusion', 2: 'Suspended', 3: 'Disabled'}
FIX_NAMES    = {0: 'No fix', 1: 'DR only', 2: '2D', 3: '3D', 4: 'GNSS+DR', 5: 'Time only'}

log_state = {
    'utc':    '—',
    'fusion': '—',
    'ax': 0.0, 'ay': 0.0, 'az': 0.0,
    'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
    'vax': 0.0, 'vay': 0.0, 'vaz': 0.0,
    'hdg': 0.0,
    'lat': None, 'lon': None,
    'hdop': None, 'vdop': None, 'pdop': None,
    'fix_type': 0,
    'speed': 0.0,
    'vel_n': 0.0, 'vel_e': 0.0, 'vel_d': 0.0,
    'h_acc': None, 'v_acc': None,
    'head_mot': 0.0, 'head_veh': 0.0,
    'nav_roll': 0.0, 'nav_pitch': 0.0, 'nav_hdg': 0.0,
    'alg_roll': 0.0, 'alg_pitch': 0.0,
    'ths_hdg': 0.0, 'ths_mi': '—',
    'nov_ax': 0.0, 'nov_ay': 0.0, 'nov_az': 0.0,
    'nov_gx': 0.0, 'nov_gy': 0.0, 'nov_gz': 0.0,
}

esf_meas_latest   = {16: 0,   17: 0,   18: 0,   14: 0,   13: 0,   5: 0  }
esf_meas_smoothed = {16: 0.0, 17: 0.0, 18: 0.0, 14: 0.0, 13: 0.0, 5: 0.0}

# ── Helpers ───────────────────────────────────────────────────────────────────

def signed24(val):
    return val - (1 << 24) if val & (1 << 23) else val

# ── WebSocket ─────────────────────────────────────────────────────────────────

clients = set()

async def broadcast(msg: dict):
    if clients:
        data = json.dumps(msg)
        await asyncio.gather(*[c.send(data) for c in clients], return_exceptions=True)

def log(level: str, text: str, loop: asyncio.AbstractEventLoop):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    print(f"[{level.upper()}] {text}")
    asyncio.run_coroutine_threadsafe(
        broadcast({"type": "LOG", "level": level, "msg": text, "ts": ts}), loop
    )

async def ws_handler(websocket):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)

# ── 1 Hz text logger ─────────────────────────────────────────────────────────

async def data_logger():
    with open(LOG_FILE, 'w') as f:
        f.write(
            "# [UTC] Fusion Fix | "
            "EVK Accel(m/s²) X Y Z  Gyro(°/s) X Y Z | "
            "NovAtel Accel(m/s²) X Y Z  Gyro(°/s) X Y Z | "
            "Veh(m/s²) X Y Z | Lat Lon | HDOP VDOP PDOP | "
            "ESF-ALG Roll Pitch Yaw\n"
        )
        while True:
            await asyncio.sleep(1.0)
            s = log_state
            pos = f"Lat:{s['lat']:.6f} Lon:{s['lon']:.6f}" if s['lat'] is not None else "Lat:— Lon:—"
            line = (
                f"[{s['utc']}] {s['fusion']} Fix:{FIX_NAMES.get(s['fix_type'], s['fix_type'])} | "
                f"EVK Accel X:{s['ax']:+.4f} Y:{s['ay']:+.4f} Z:{s['az']:+.4f}  "
                f"Gyro X:{s['gx']:+.4f} Y:{s['gy']:+.4f} Z:{s['gz']:+.4f} | "
                f"NovAtel Accel X:{s['nov_ax']:+.4f} Y:{s['nov_ay']:+.4f} Z:{s['nov_az']:+.4f}  "
                f"Gyro X:{s['nov_gx']:+.4f} Y:{s['nov_gy']:+.4f} Z:{s['nov_gz']:+.4f} | "
                f"Veh X:{s['vax']:+.4f} Y:{s['vay']:+.4f} Z:{s['vaz']:+.4f} | "
                f"{pos} | "
                f"HDOP:{s['hdop'] if s['hdop'] is not None else '—'} "
                f"VDOP:{s['vdop'] if s['vdop'] is not None else '—'} "
                f"PDOP:{s['pdop'] if s['pdop'] is not None else '—'} | "
                f"ESF-ALG Roll:{s['alg_roll']:+.2f}° Pitch:{s['alg_pitch']:+.2f}° Yaw:{s['hdg']:+.2f}°"
            )
            f.write(line + '\n')
            f.flush()
            await broadcast({"type": "DATA_LOG", "line": line})

# ── NovAtel PwrPak7 thread ────────────────────────────────────────────────────

_RAWIMUSXA_COLS = [
    'head', 'week_num1', 'seconds1', 'imu_info', 'week_num2', 'seconds2',
    'status', 'accel_z', 'neg_accel_y', 'accel_x', 'gyro_z', 'neg_gyro_y', 'gyro_x', 'crc',
]

_NOV_IMU_FIELDS = [
    'sys_datetime', 'unix_timestamp', 'gps_seconds',
    'raw_accel_x_ms2', 'raw_accel_y_ms2', 'raw_accel_z_ms2',
    'raw_gyro_x_degs', 'raw_gyro_y_degs', 'raw_gyro_z_degs',
]


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
    if len(fields) != len(_RAWIMUSXA_COLS):
        return None

    d = dict(zip(_RAWIMUSXA_COLS, fields))
    try:
        ax = int(d['accel_x']);  ay = -int(d['neg_accel_y']); az = int(d['accel_z'])
        gx = int(d['gyro_x']);   gy = -int(d['neg_gyro_y']);  gz = int(d['gyro_z'])
        gps_sec = float(d['seconds2'])
    except (ValueError, KeyError):
        return None

    return {
        'gps_seconds':     gps_sec,
        'raw_accel_x_ms2': ax * NOV_ACCEL_SCALE,
        'raw_accel_y_ms2': ay * NOV_ACCEL_SCALE,
        'raw_accel_z_ms2': az * NOV_ACCEL_SCALE,
        'raw_gyro_x_degs': gx * NOV_GYRO_SCALE,
        'raw_gyro_y_degs': gy * NOV_GYRO_SCALE,
        'raw_gyro_z_degs': gz * NOV_GYRO_SCALE,
    }


def nov_thread(stop: threading.Event, loop: asyncio.AbstractEventLoop) -> None:
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

    t0_wall: float | None = None
    pkt_count = 0

    nov_fh = open(_NOV_PATH, 'w', newline='', buffering=1)
    nov_w  = csv.DictWriter(nov_fh, fieldnames=_NOV_IMU_FIELDS)
    nov_w.writeheader()

    try:
        while not stop.is_set():
            raw = ser.readline()
            if not raw:
                continue
            parsed = _parse_rawimusxa(raw.decode('ascii', errors='ignore'))
            if not parsed:
                continue

            if t0_wall is None:
                t0_wall = time.time()
            t = t0_wall + pkt_count * 0.01
            pkt_count += 1

            ax = parsed['raw_accel_x_ms2']
            ay = parsed['raw_accel_y_ms2']
            az = parsed['raw_accel_z_ms2']
            gx = parsed['raw_gyro_x_degs']
            gy = parsed['raw_gyro_y_degs']
            gz = parsed['raw_gyro_z_degs']

            log_state.update({'nov_ax': ax, 'nov_ay': ay, 'nov_az': az,
                              'nov_gx': gx, 'nov_gy': gy, 'nov_gz': gz})

            dt = datetime.fromtimestamp(t, tz=timezone.utc)
            nov_w.writerow({
                'sys_datetime':    dt.strftime('%Y-%m-%d %H:%M:%S.%f'),
                'unix_timestamp':  f'{t:.6f}',
                'gps_seconds':     parsed['gps_seconds'],
                'raw_accel_x_ms2': round(ax, 6),
                'raw_accel_y_ms2': round(ay, 6),
                'raw_accel_z_ms2': round(az, 6),
                'raw_gyro_x_degs': round(gx, 6),
                'raw_gyro_y_degs': round(gy, 6),
                'raw_gyro_z_degs': round(gz, 6),
            })

            asyncio.run_coroutine_threadsafe(
                broadcast({
                    "type":  "NOV-MEAS",
                    "accel": {"x": round(ax, 4), "y": round(ay, 4), "z": round(az, 4)},
                    "gyro":  {"x": round(gx, 4), "y": round(gy, 4), "z": round(gz, 4)},
                }),
                loop,
            )
    finally:
        ser.write(b'unlogall all\r\n')
        ser.close()
        nov_fh.close()
        print('[NovAtel] Stopped')

# ── EVK-M9DR async reader ─────────────────────────────────────────────────────

async def serial_reader():
    stream = serial.Serial(EVK_PORT, EVK_BAUD, timeout=1)

    cfg_keys = [
        ("CFG-SFCORE-USE_SF",             1),
        ("CFG-SFIMU-IMU_EN",              1),
        ("CFG-SFIMU-AUTO_MNTALG_ENA",     1),
        ("CFG-MSGOUT-UBX_ESF_MEAS_USB",   1),
        ("CFG-MSGOUT-UBX_ESF_ALG_USB",    1),
        ("CFG-MSGOUT-UBX_ESF_INS_USB",    1),
        ("CFG-MSGOUT-UBX_ESF_STATUS_USB", 1),
        ("CFG-MSGOUT-UBX_NAV_PVT_USB",    1),
        ("CFG-MSGOUT-UBX_NAV_ATT_USB",    1),
        ("CFG-MSGOUT-UBX_NAV_STATUS_USB", 1),
        ("CFG-MSGOUT-NMEA_ID_GGA_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_GSA_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_RMC_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_THS_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_ZDA_USB",    1),
        ("CFG-RATE-MEAS",    10),
        ("CFG-RATE-NAV",      1),
        ("CFG-RATE-TIMEREF",  0),
    ]
    cfg_msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfg_keys)
    stream.write(cfg_msg.serialize())

    ubr  = UBXReader(stream, protfilter=UBX_PROTOCOL | NMEA_PROTOCOL)
    loop = asyncio.get_event_loop()

    prev_fusion_mode = None
    prev_calib       = {}
    _nav_utc_anchor  = None
    _nav_itow_anchor = None

    def read_loop():
        nonlocal prev_fusion_mode, prev_calib, _nav_utc_anchor, _nav_itow_anchor

        _IMU_FIELDS = [
            'sys_datetime', 'unix_timestamp',
            'accel_x_ms2', 'accel_y_ms2', 'accel_z_ms2',
            'gyro_x_degs', 'gyro_y_degs', 'gyro_z_degs',
            'ins_accel_x_ms2', 'ins_accel_y_ms2', 'ins_accel_z_ms2',
            'ins_gyro_x_degs', 'ins_gyro_y_degs', 'ins_gyro_z_degs',
            'heading_deg', 'nav_hdg',
        ]
        imu_fh = open(_IMU_PATH, 'w', newline='', buffering=1)
        imu_w  = csv.DictWriter(imu_fh, fieldnames=_IMU_FIELDS)
        imu_w.writeheader()

        _NAV_FIELDS = [
            'sys_datetime', 'unix_timestamp',
            'fix_type', 'num_sv',
            'lat_deg', 'lon_deg', 'height_m',
            'h_acc_m', 'v_acc_m',
            'speed_ms', 'vel_n_ms', 'vel_e_ms', 'vel_d_ms',
            'head_mot_deg', 'head_veh_deg',
            'roll_deg', 'pitch_deg', 'heading_deg',
            'roll_acc_deg', 'pitch_acc_deg', 'heading_acc_deg',
        ]
        nav_fh = open(_NAV_PATH, 'w', newline='', buffering=1)
        nav_w  = csv.DictWriter(nav_fh, fieldnames=_NAV_FIELDS, extrasaction='ignore')
        nav_w.writeheader()

        _DISPLAY_HZ      = 1 / 30
        _last_meas_bcast = 0.0
        log('info', f'Serial open: {EVK_PORT} @ {EVK_BAUD} baud', loop)
        log('info', 'Config sent — messages enabled', loop)

        while True:
            try:
                (_, parsed) = ubr.read()
            except Exception as e:
                log('error', f'Parse error: {e}', loop)
                continue
            if not parsed:
                continue

            identity = parsed.identity
            msg      = None

            # ── ESF-MEAS: raw accel/gyro ──────────────────────────────────────
            if identity == 'ESF-MEAS':
                samples = {}
                for i in range(1, 9):
                    dtype  = getattr(parsed, f"dataType_{i:02d}", None)
                    dfield = getattr(parsed, f"dataField_{i:02d}", None)
                    if dtype is None or dfield is None:
                        break
                    samples[dtype] = signed24(dfield)

                if not any(k in samples for k in (5, 13, 14, 16, 17, 18)):
                    msg = None
                else:
                    for k, v in samples.items():
                        if k in esf_meas_latest:
                            esf_meas_latest[k] = v
                            esf_meas_smoothed[k] = (
                                EMA_ALPHA * v + (1 - EMA_ALPHA) * esf_meas_smoothed[k]
                            )
                    ax = round(esf_meas_smoothed[16] * EVK_ACCEL_SCALE, 4)
                    ay = round(esf_meas_smoothed[17] * EVK_ACCEL_SCALE, 4)
                    az = round(esf_meas_smoothed[18] * EVK_ACCEL_SCALE, 4)
                    gx = round(esf_meas_smoothed[14] * EVK_GYRO_SCALE,  4)
                    gy = round(esf_meas_smoothed[13] * EVK_GYRO_SCALE,  4)
                    gz = round(esf_meas_smoothed[5]  * EVK_GYRO_SCALE,  4)
                    log_state.update({'ax': ax, 'ay': ay, 'az': az,
                                      'gx': gx, 'gy': gy, 'gz': gz})

                    now_t = time.monotonic()
                    if now_t - _last_meas_bcast >= _DISPLAY_HZ:
                        _last_meas_bcast = now_t
                        msg = {
                            "type":  "ESF-MEAS",
                            "accel": {"x": ax, "y": ay, "z": az},
                            "gyro":  {"x": gx, "y": gy, "z": gz},
                        }

                    if _nav_utc_anchor is not None:
                        delta_ms = int(parsed.timeTag) - _nav_itow_anchor
                        if delta_ms < 0:
                            delta_ms += 604800000
                        esf_dt = _nav_utc_anchor + timedelta(milliseconds=delta_ms)
                    else:
                        esf_dt = datetime.now(tz=timezone.utc)

                    imu_w.writerow({
                        'sys_datetime':    esf_dt.strftime('%Y-%m-%d %H:%M:%S.%f'),
                        'unix_timestamp':  f'{esf_dt.timestamp():.6f}',
                        'accel_x_ms2':     round(esf_meas_latest[16] * EVK_ACCEL_SCALE, 6),
                        'accel_y_ms2':     round(esf_meas_latest[17] * EVK_ACCEL_SCALE, 6),
                        'accel_z_ms2':     round(esf_meas_latest[18] * EVK_ACCEL_SCALE, 6),
                        'gyro_x_degs':     round(esf_meas_latest[14] * EVK_GYRO_SCALE,  6),
                        'gyro_y_degs':     round(esf_meas_latest[13] * EVK_GYRO_SCALE,  6),
                        'gyro_z_degs':     round(esf_meas_latest[5]  * EVK_GYRO_SCALE,  6),
                        'ins_accel_x_ms2': log_state.get('vax', ''),
                        'ins_accel_y_ms2': log_state.get('vay', ''),
                        'ins_accel_z_ms2': log_state.get('vaz', ''),
                        'ins_gyro_x_degs': log_state.get('vgx', ''),
                        'ins_gyro_y_degs': log_state.get('vgy', ''),
                        'ins_gyro_z_degs': log_state.get('vgz', ''),
                        'heading_deg':     log_state.get('ths_hdg', ''),
                        'nav_hdg':         log_state.get('nav_hdg', ''),
                    })

            # ── ESF-STATUS: fusion / calibration status ───────────────────────
            elif identity == 'ESF-STATUS':
                CALIB_NAMES  = {0: 'NOT CALIBRATED', 1: 'CALIBRATING',
                                2: 'CALIBRATED',     3: 'CALIBRATED'}
                SENSOR_NAMES = {5: 'Gyro Z', 10: 'Temperature', 13: 'Gyro Y',
                                14: 'Gyro X', 16: 'Accel X', 17: 'Accel Y', 18: 'Accel Z'}
                log_state['fusion'] = FUSION_NAMES.get(parsed.fusionMode, str(parsed.fusionMode))
                if parsed.fusionMode != prev_fusion_mode:
                    log('info', f'Fusion mode → {log_state["fusion"]}', loop)
                    prev_fusion_mode = parsed.fusionMode
                sensors = []
                for i in range(1, parsed.numSens + 1):
                    sensor_type = getattr(parsed, f"type_{i:02d}", None)
                    calib       = getattr(parsed, f"calibStatus_{i:02d}", None)
                    if sensor_type is None:
                        break
                    if prev_calib.get(sensor_type) != calib:
                        name = SENSOR_NAMES.get(sensor_type, f'type={sensor_type}')
                        log('info', f'{name} calib → {CALIB_NAMES.get(calib, calib)}', loop)
                        prev_calib[sensor_type] = calib
                    sensors.append({"type": sensor_type, "calibStatus": calib})
                msg = {
                    "type":          "ESF-STATUS",
                    "fusionMode":    parsed.fusionMode,
                    "imuInitStatus": parsed.imuInitStatus,
                    "insInitStatus": parsed.insInitStatus,
                    "mntAlgStatus":  parsed.mntAlgStatus,
                    "sensors":       sensors,
                }

            # ── ESF-INS: compensated vehicle-frame accel/gyro ─────────────────
            elif identity == 'ESF-INS':
                vax = round(parsed.xAccel,   4)
                vay = round(parsed.yAccel,   4)
                vaz = round(parsed.zAccel,   4)
                vgx = round(parsed.xAngRate, 4)
                vgy = round(parsed.yAngRate, 4)
                vgz = round(parsed.zAngRate, 4)
                log_state.update({'vax': vax, 'vay': vay, 'vaz': vaz,
                                  'vgx': vgx, 'vgy': vgy, 'vgz': vgz})
                msg = {
                    "type":  "ESF-INS",
                    "accel": {"x": vax, "y": vay, "z": vaz},
                    "gyro":  {"x": vgx, "y": vgy, "z": vgz},
                }

            # ── ESF-ALG: installation angle ───────────────────────────────────
            elif identity == 'ESF-ALG':
                log_state['hdg']       = round(parsed.yaw,   2)
                log_state['alg_roll']  = round(parsed.roll,  2)
                log_state['alg_pitch'] = round(parsed.pitch, 2)
                msg = {
                    "type":   "ESF-ALG",
                    "roll":   log_state['alg_roll'],
                    "pitch":  log_state['alg_pitch'],
                    "yaw":    log_state['hdg'],
                    "status": parsed.status,
                }

            # ── THS: true heading ─────────────────────────────────────────────
            elif identity in ('GNTHS', 'GPTHS', 'GLTHS', 'GATHS'):
                ths_mi = str(parsed.mi)
                if parsed.headt != '':
                    ths_hdg = round(float(parsed.headt), 2)
                    log_state.update({'ths_hdg': ths_hdg, 'ths_mi': ths_mi})
                    msg = {"type": "THS", "heading": ths_hdg, "mi": ths_mi}

            # ── GGA: position ─────────────────────────────────────────────────
            elif identity in ('GNGGA', 'GPGGA'):
                utc = str(parsed.time)
                log_state['utc'] = utc
                if parsed.lat and parsed.lon:
                    log_state['lat'] = float(parsed.lat)
                    log_state['lon'] = float(parsed.lon)
                    msg = {
                        "type":  "GGA",
                        "lat":   log_state['lat'],
                        "lon":   log_state['lon'],
                        "alt":   float(parsed.alt),
                        "hdop":  float(parsed.HDOP),
                        "numSV": int(parsed.numSV),
                        "time":  utc,
                    }

            # ── GSA: DOP ──────────────────────────────────────────────────────
            elif identity in ('GNGSA', 'GPGSA'):
                hdop = float(parsed.HDOP) if parsed.HDOP else None
                vdop = float(parsed.VDOP) if parsed.VDOP else None
                pdop = float(parsed.PDOP) if parsed.PDOP else None
                log_state.update({'hdop': hdop, 'vdop': vdop, 'pdop': pdop})
                msg = {"type": "GSA", "hdop": hdop, "vdop": vdop, "pdop": pdop}

            # ── RMC: UTC time ─────────────────────────────────────────────────
            elif identity in ('GNRMC', 'GPRMC'):
                utc = str(parsed.time)
                log_state['utc'] = utc
                msg = {"type": "RMC", "time": utc, "date": str(parsed.date)}

            # ── NAV-PVT: full position/velocity/time ──────────────────────────
            elif identity == 'NAV-PVT':
                fix_type = int(parsed.fixType)
                lat      = float(parsed.lat)
                lon      = float(parsed.lon)
                height   = round(parsed.height / 1000.0, 3)
                h_acc    = round(parsed.hAcc   / 1000.0, 3)
                v_acc    = round(parsed.vAcc   / 1000.0, 3)
                speed    = round(parsed.gSpeed / 1000.0, 4)
                vel_n    = round(parsed.velN   / 1000.0, 4)
                vel_e    = round(parsed.velE   / 1000.0, 4)
                vel_d    = round(parsed.velD   / 1000.0, 4)
                head_mot = round(float(parsed.headMot), 2)
                head_veh = round(float(parsed.headVeh), 2)
                num_sv   = int(parsed.numSV)
                updates  = {
                    'fix_type': fix_type, 'speed': speed,
                    'vel_n': vel_n, 'vel_e': vel_e, 'vel_d': vel_d,
                    'h_acc': h_acc, 'v_acc': v_acc,
                    'head_mot': head_mot, 'head_veh': head_veh,
                }
                if fix_type >= 2:
                    updates.update({'lat': lat, 'lon': lon})
                log_state.update(updates)
                try:
                    pvt_dt = datetime(
                        parsed.year, parsed.month, parsed.day,
                        parsed.hour, parsed.min, parsed.sec,
                        max(0, int(parsed.nano)) // 1000,
                        tzinfo=timezone.utc,
                    )
                    _nav_utc_anchor  = pvt_dt
                    _nav_itow_anchor = int(parsed.iTOW)
                except Exception:
                    pvt_dt = datetime.now(tz=timezone.utc)
                nav_w.writerow({
                    'sys_datetime':  pvt_dt.strftime('%Y-%m-%d %H:%M:%S.%f'),
                    'unix_timestamp': f'{pvt_dt.timestamp():.6f}',
                    'fix_type':      fix_type,
                    'num_sv':        num_sv,
                    'lat_deg':       lat,
                    'lon_deg':       lon,
                    'height_m':      height,
                    'h_acc_m':       h_acc,
                    'v_acc_m':       v_acc,
                    'speed_ms':      speed,
                    'vel_n_ms':      vel_n,
                    'vel_e_ms':      vel_e,
                    'vel_d_ms':      vel_d,
                    'head_mot_deg':  head_mot,
                    'head_veh_deg':  head_veh,
                    'roll_deg':      log_state['nav_roll'],
                    'pitch_deg':     log_state['nav_pitch'],
                    'heading_deg':   log_state['nav_hdg'],
                })
                msg = {
                    "type":    "NAV-PVT",
                    "fixType": fix_type, "numSV": num_sv,
                    "lat": lat, "lon": lon, "alt": height,
                    "speed": speed,
                    "velN": vel_n, "velE": vel_e, "velD": vel_d,
                    "hAcc": h_acc, "vAcc": v_acc,
                    "headMot": head_mot, "headVeh": head_veh,
                }

            # ── NAV-ATT: orientation ──────────────────────────────────────────
            elif identity == 'NAV-ATT':
                nav_roll  = round(float(parsed.roll),       2)
                nav_pitch = round(float(parsed.pitch),      2)
                nav_hdg   = round(float(parsed.heading),    2)
                roll_acc  = round(float(parsed.accRoll),    2)
                pitch_acc = round(float(parsed.accPitch),   2)
                hdg_acc   = round(float(parsed.accHeading), 2)
                log_state.update({'nav_roll': nav_roll, 'nav_pitch': nav_pitch,
                                  'nav_hdg': nav_hdg})
                if _nav_utc_anchor is not None:
                    delta_ms = int(parsed.iTOW) - _nav_itow_anchor
                    if delta_ms < 0:
                        delta_ms += 604800000
                    att_dt = _nav_utc_anchor + timedelta(milliseconds=delta_ms)
                else:
                    att_dt = datetime.now(tz=timezone.utc)
                nav_w.writerow({
                    'sys_datetime':    att_dt.strftime('%Y-%m-%d %H:%M:%S.%f'),
                    'unix_timestamp':  f'{att_dt.timestamp():.6f}',
                    'roll_deg':        nav_roll,
                    'pitch_deg':       nav_pitch,
                    'heading_deg':     nav_hdg,
                    'roll_acc_deg':    roll_acc,
                    'pitch_acc_deg':   pitch_acc,
                    'heading_acc_deg': hdg_acc,
                })
                msg = {
                    "type":    "NAV-ATT",
                    "roll":    nav_roll,
                    "pitch":   nav_pitch,
                    "heading": nav_hdg,
                }

            # ── NAV-STATUS: fix status ────────────────────────────────────────
            elif identity == 'NAV-STATUS':
                fix_type = int(parsed.gpsFix)
                log_state['fix_type'] = fix_type
                msg = {
                    "type":     "NAV-STATUS",
                    "gpsFix":   fix_type,
                    "gpsFixOk": bool(getattr(parsed, 'gpsFixOk',
                                             int(getattr(parsed, 'flags', 0)) & 1)),
                }

            if msg:
                asyncio.run_coroutine_threadsafe(broadcast(msg), loop)

    await asyncio.to_thread(read_loop)

# ── Main ──────────────────────────────────────────────────────────────────────

async def main():
    loop = asyncio.get_event_loop()
    stop = threading.Event()

    t_nov = threading.Thread(target=nov_thread, args=(stop, loop),
                             daemon=True, name='NovAtel')
    t_nov.start()

    print(f"WebSocket  → ws://{WS_HOST}:{WS_PORT}")
    print(f"EVK log    → {LOG_FILE}")
    print(f"EVK IMU    → {_IMU_PATH}")
    print(f"EVK NAV    → {_NAV_PATH}")
    print(f"NovAtel    → {_NOV_PATH}")
    print("Open EVK-M9DR-dashboard.html in your browser.")

    try:
        async with websockets.serve(ws_handler, WS_HOST, WS_PORT, reuse_address=True):
            await asyncio.gather(serial_reader(), data_logger())
    except asyncio.CancelledError:
        pass
    finally:
        stop.set()
        t_nov.join(timeout=3)
        print("Stopped.")

asyncio.run(main())
