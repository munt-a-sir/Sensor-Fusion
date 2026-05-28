import asyncio
import json
import os
import serial
import websockets
from datetime import datetime
from pyubx2 import UBXReader, UBXMessage, UBX_PROTOCOL, NMEA_PROTOCOL, SET_LAYER_RAM, TXN_NONE

SERIAL_PORT = '/dev/ttyACM0'
BAUDRATE    = 115200
WS_HOST     = 'localhost'
WS_PORT     = 5052
LOG_FILE    = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'EVK-M9DR-log.txt')

GYRO_SCALE  = 1 / 4096.0
ACCEL_SCALE = 1 / 1024.0
EMA_ALPHA   = 0.2   # lower = smoother but more lag; raise toward 1.0 for less smoothing

FUSION_NAMES = {0: 'Initializing', 1: 'Fusion', 2: 'Suspended', 3: 'Disabled'}
FIX_NAMES    = {0: 'No fix', 1: 'DR only', 2: '2D', 3: '3D', 4: 'GNSS+DR', 5: 'Time only'}

log_state = {
    'utc':    '—',
    'fusion': '—',
    'ax': 0.0, 'ay': 0.0, 'az': 0.0,
    'gx': 0.0, 'gy': 0.0, 'gz': 0.0,
    'vax': 0.0, 'vay': 0.0, 'vaz': 0.0,
    'vgx': 0.0, 'vgy': 0.0, 'vgz': 0.0,
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
}

def signed24(val):
    return val - (1 << 24) if val & (1 << 23) else val

clients = set()
esf_meas_latest   = {16: 0,   17: 0,   18: 0,   14: 0,   13: 0,   5: 0  }
esf_meas_smoothed = {16: 0.0, 17: 0.0, 18: 0.0, 14: 0.0, 13: 0.0, 5: 0.0}

async def broadcast(msg: dict):
    if clients:
        data = json.dumps(msg)
        await asyncio.gather(*[c.send(data) for c in clients], return_exceptions=True)

def log(level: str, text: str, loop):
    ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
    print(f"[{level.upper()}] {text}")
    asyncio.run_coroutine_threadsafe(
        broadcast({"type": "LOG", "level": level, "msg": text, "ts": ts}), loop
    )

async def data_logger():
    with open(LOG_FILE, 'a') as f:
        f.write(f"# Session started {datetime.now().isoformat()}\n")
        f.write("# [UTC] Fusion Fix | Accel(m/s²) X Y Z  Gyro(°/s) X Y Z | Veh(m/s²) X Y Z  VehGyro(°/s) X Y Z  THS Hdg(°) | Spd(m/s) VelN VelE VelD | Lat Lon | hAcc vAcc | HDOP VDOP PDOP | NAV-ATT Roll Pitch Hdg | ESF-ALG Roll Pitch Yaw | THS Hdg MI\n")
        while True:
            await asyncio.sleep(1.0)
            s = log_state
            pos = f"Lat:{s['lat']:.6f} Lon:{s['lon']:.6f}" if s['lat'] is not None else "Lat:— Lon:—"
            line = (
                f"[{s['utc']}] {s['fusion']} Fix:{FIX_NAMES.get(s['fix_type'], s['fix_type'])} | "
                f"Accel(m/s²) X:{s['ax']:+.4f} Y:{s['ay']:+.4f} Z:{s['az']:+.4f}  "
                f"Gyro(°/s) X:{s['gx']:+.4f} Y:{s['gy']:+.4f} Z:{s['gz']:+.4f} | "
                f"Veh(m/s²) X:{s['vax']:+.4f} Y:{s['vay']:+.4f} Z:{s['vaz']:+.4f}  "
                f"VehGyro(°/s) X:{s['vgx']:+.4f} Y:{s['vgy']:+.4f} Z:{s['vgz']:+.4f}  "
                f"THS Hdg:{s['ths_hdg']:+.2f}° | "
                f"Spd:{s['speed']:+.3f}m/s VN:{s['vel_n']:+.3f} VE:{s['vel_e']:+.3f} VD:{s['vel_d']:+.3f} | "
                f"{pos} | "
                f"hAcc:{s['h_acc'] if s['h_acc'] is not None else '—'}m vAcc:{s['v_acc'] if s['v_acc'] is not None else '—'}m | "
                f"HDOP:{s['hdop'] if s['hdop'] is not None else '—'} "
                f"VDOP:{s['vdop'] if s['vdop'] is not None else '—'} "
                f"PDOP:{s['pdop'] if s['pdop'] is not None else '—'} | "
                f"NAV-ATT Roll:{s['nav_roll']:+.2f}° Pitch:{s['nav_pitch']:+.2f}° Hdg:{s['nav_hdg']:+.2f}° | "
                f"ESF-ALG Roll:{s['alg_roll']:+.2f}° Pitch:{s['alg_pitch']:+.2f}° Yaw:{s['hdg']:+.2f}° | "
                f"THS Hdg:{s['ths_hdg']:+.2f}° MI:{s['ths_mi']}"
            )
            f.write(line + '\n')
            f.flush()
            await broadcast({"type": "DATA_LOG", "line": line})

async def ws_handler(websocket):
    clients.add(websocket)
    try:
        await websocket.wait_closed()
    finally:
        clients.discard(websocket)

async def serial_reader():
    stream = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

    cfg_keys = [
        ("CFG-SFCORE-USE_SF",             1),
        ("CFG-SFIMU-IMU_EN",              1),
        ("CFG-SFIMU-AUTO_MNTALG_ENA",     1),
        ("CFG-MSGOUT-UBX_ESF_MEAS_USB",   1),
        ("CFG-MSGOUT-UBX_ESF_ALG_USB",    1),
        ("CFG-MSGOUT-UBX_ESF_INS_USB",    1),
        ("CFG-MSGOUT-UBX_ESF_STATUS_USB", 1),
        ("CFG-MSGOUT-UBX_NAV_PVT_USB",     1),
        ("CFG-MSGOUT-UBX_NAV_ATT_USB",     1),
        ("CFG-MSGOUT-UBX_NAV_STATUS_USB",  1),
        ("CFG-MSGOUT-NMEA_ID_GGA_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_GSA_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_RMC_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_THS_USB",    1),
        ("CFG-MSGOUT-NMEA_ID_ZDA_USB",    1),
        # 30 Hz: measurement period = 1000/30 ≈ 33 ms
        ("CFG-RATE-MEAS",    33),
        ("CFG-RATE-NAV",      1),
        ("CFG-RATE-TIMEREF",  0),
    ]
    cfg_msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfg_keys)
    stream.write(cfg_msg.serialize())
    print("Config sent.")

    ubr = UBXReader(stream, protfilter=UBX_PROTOCOL | NMEA_PROTOCOL)
    loop = asyncio.get_event_loop()

    prev_fusion_mode = None
    prev_calib = {}

    def read_loop():
        nonlocal prev_fusion_mode, prev_calib
        log('info', f'Serial open: {SERIAL_PORT} @ {BAUDRATE} baud', loop)
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
            msg = None

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
                    # Persist latest raw value per data type, then smooth
                    for k, v in samples.items():
                        if k in esf_meas_latest:
                            esf_meas_latest[k] = v
                            esf_meas_smoothed[k] = (
                                EMA_ALPHA * v + (1 - EMA_ALPHA) * esf_meas_smoothed[k]
                            )
                    ax = round(esf_meas_smoothed[16] * ACCEL_SCALE, 4)
                    ay = round(esf_meas_smoothed[17] * ACCEL_SCALE, 4)
                    az = round(esf_meas_smoothed[18] * ACCEL_SCALE, 4)
                    gx = round(esf_meas_smoothed[14] * GYRO_SCALE, 4)
                    gy = round(esf_meas_smoothed[13] * GYRO_SCALE, 4)
                    gz = round(esf_meas_smoothed[5]  * GYRO_SCALE, 4)
                    log_state.update({'ax': ax, 'ay': ay, 'az': az, 'gx': gx, 'gy': gy, 'gz': gz})
                    msg = {
                        "type": "ESF-MEAS",
                        "accel": {"x": ax, "y": ay, "z": az},
                        "gyro":  {"x": gx, "y": gy, "z": gz},
                    }
            
            elif identity == "ESF-STATUS":
                FUSION_NAMES = {0: 'Initializing', 1: 'Fusion', 2: 'Suspended', 3: 'Disabled'}
                CALIB_NAMES  = {0: 'NOT CALIBRATED', 1: 'CALIBRATING', 2: 'CALIBRATED', 3: 'CALIBRATED'}
                SENSOR_NAMES = {5: 'Gyro Z', 10: 'Temperature', 13: 'Gyro Y', 14: 'Gyro X', 16: 'Accel X', 17: 'Accel Y', 18: 'Accel Z'}
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
                    "type": "ESF-STATUS",
                    "fusionMode":    parsed.fusionMode,
                    "imuInitStatus": parsed.imuInitStatus,
                    "insInitStatus": parsed.insInitStatus,
                    "mntAlgStatus":  parsed.mntAlgStatus,
                    "sensors": sensors,
                }

            elif identity == 'ESF-INS':
                vax = round(parsed.xAccel,    4)
                vay = round(parsed.yAccel,    4)
                vaz = round(parsed.zAccel,    4)
                vgx = round(parsed.xAngRate,  4)
                vgy = round(parsed.yAngRate,  4)
                vgz = round(parsed.zAngRate,  4)
                log_state.update({'vax': vax, 'vay': vay, 'vaz': vaz,
                                  'vgx': vgx, 'vgy': vgy, 'vgz': vgz})
                msg = {
                    "type": "ESF-INS",
                    "accel": {"x": vax, "y": vay, "z": vaz},
                    "gyro":  {
                        "x": round(parsed.xAngRate, 4),
                        "y": round(parsed.yAngRate, 4),
                        "z": round(parsed.zAngRate, 4),
                    },
                }

            elif identity == 'ESF-ALG':
                log_state['hdg']       = round(parsed.yaw,   2)
                log_state['alg_roll']  = round(parsed.roll,  2)
                log_state['alg_pitch'] = round(parsed.pitch, 2)
                msg = {
                    "type": "ESF-ALG",
                    "roll":  log_state['alg_roll'],
                    "pitch": log_state['alg_pitch'],
                    "yaw":   log_state['hdg'],
                }

            elif identity in ('GNTHS', 'GPTHS', 'GLTHS', 'GATHS'):
                ths_mi = str(parsed.mi)
                if parsed.headt != '':
                    ths_hdg = round(float(parsed.headt), 2)
                    log_state.update({'ths_hdg': ths_hdg, 'ths_mi': ths_mi})
                    msg = {
                        "type":    "THS",
                        "heading": ths_hdg,
                        "mi":      ths_mi,
                    }

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

            elif identity in ('GNGSA', 'GPGSA'):
                hdop = float(parsed.HDOP) if parsed.HDOP else None
                vdop = float(parsed.VDOP) if parsed.VDOP else None
                pdop = float(parsed.PDOP) if parsed.PDOP else None
                log_state.update({'hdop': hdop, 'vdop': vdop, 'pdop': pdop})
                msg = {
                    "type": "GSA",
                    "hdop": hdop,
                    "vdop": vdop,
                    "pdop": pdop,
                }

            elif identity in ('GNRMC', 'GPRMC'):
                utc = str(parsed.time)
                log_state['utc'] = utc
                msg = {
                    "type": "RMC",
                    "time": utc,
                    "date": str(parsed.date),
                }

            elif identity == 'NAV-PVT':
                fix_type = int(parsed.fixType)
                lat      = float(parsed.lat)
                lon      = float(parsed.lon)
                height   = round(parsed.height / 1000.0, 3)   # mm → m
                h_acc    = round(parsed.hAcc   / 1000.0, 3)   # mm → m
                v_acc    = round(parsed.vAcc   / 1000.0, 3)   # mm → m
                speed    = round(parsed.gSpeed / 1000.0, 4)   # mm/s → m/s
                vel_n    = round(parsed.velN   / 1000.0, 4)
                vel_e    = round(parsed.velE   / 1000.0, 4)
                vel_d    = round(parsed.velD   / 1000.0, 4)
                head_mot = round(float(parsed.headMot), 2)    # auto-scaled to deg
                head_veh = round(float(parsed.headVeh), 2)
                num_sv   = int(parsed.numSV)
                updates = {
                    'fix_type': fix_type, 'speed': speed,
                    'vel_n': vel_n, 'vel_e': vel_e, 'vel_d': vel_d,
                    'h_acc': h_acc, 'v_acc': v_acc,
                    'head_mot': head_mot, 'head_veh': head_veh,
                }
                if fix_type >= 2:
                    updates.update({'lat': lat, 'lon': lon})
                log_state.update(updates)
                msg = {
                    "type": "NAV-PVT",
                    "fixType": fix_type, "numSV": num_sv,
                    "lat": lat, "lon": lon, "alt": height,
                    "speed": speed,
                    "velN": vel_n, "velE": vel_e, "velD": vel_d,
                    "hAcc": h_acc, "vAcc": v_acc,
                    "headMot": head_mot, "headVeh": head_veh,
                }

            elif identity == 'NAV-ATT':
                nav_roll  = round(float(parsed.roll),    2)   # auto-scaled to deg
                nav_pitch = round(float(parsed.pitch),   2)
                nav_hdg   = round(float(parsed.heading), 2)
                log_state.update({'nav_roll': nav_roll, 'nav_pitch': nav_pitch, 'nav_hdg': nav_hdg})
                msg = {
                    "type": "NAV-ATT",
                    "roll": nav_roll, "pitch": nav_pitch, "heading": nav_hdg,
                }

            elif identity == 'NAV-STATUS':
                fix_type = int(parsed.gpsFix)
                log_state['fix_type'] = fix_type
                msg = {
                    "type": "NAV-STATUS",
                    "gpsFix": fix_type,
                    "gpsFixOk": bool(getattr(parsed, 'gpsFixOk', int(getattr(parsed, 'flags', 0)) & 1)),
                }

            if msg:
                asyncio.run_coroutine_threadsafe(broadcast(msg), loop)

    await asyncio.to_thread(read_loop)

async def main():
    print(f"Dashboard WebSocket server at ws://{WS_HOST}:{WS_PORT}")
    print(f"Logging to {LOG_FILE}")
    print("Open EVK-M9DR-dashboard.html in your browser.")
    async with websockets.serve(ws_handler, WS_HOST, WS_PORT, reuse_address=True):
        await asyncio.gather(serial_reader(), data_logger())

asyncio.run(main())
