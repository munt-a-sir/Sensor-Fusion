import serial
from pyubx2 import UBXReader, UBXMessage, UBX_PROTOCOL, NMEA_PROTOCOL, SET_LAYER_RAM, TXN_NONE

serial_port = '/dev/ttyACM0'
baudrate = 115200

stream = serial.Serial(serial_port, baudrate, timeout=1)

# Have enable messages


# Enable sensor fusion core + IMU, then enable message outputs on UART1
cfg_keys = [
    ("CFG-SFCORE-USE_SF", 1),                 # Enable sensor fusion subsystem
    ("CFG-SFIMU-IMU_EN", 1),                  # Enable internal IMU
    ("CFG-SFIMU-AUTO_MNTALG_ENA", 1),         # Auto-detect installation angle
    ("CFG-MSGOUT-UBX_ESF_MEAS_USB", 1),    # Raw accel/gyro (sensor frame)
    ("CFG-MSGOUT-UBX_ESF_ALG_USB", 1),     # Installation angle
    ("CFG-MSGOUT-UBX_ESF_INS_USB", 1),     # Compensated accel/gyro (vehicle frame)
    ("CFG-MSGOUT-UBX_ESF_STATUS_USB", 1),  # Sensor fusion status
    ("CFG-MSGOUT-UBX_NAV_ATT_USB", 1),     # Orientation (Euler)
    ("CFG-MSGOUT-NMEA_ID_ZDA_USB", 1),     # UTC timestamp
]

cfg_msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, cfg_keys)
stream.write(cfg_msg.serialize())
print("Config sent — messages enabled.")

ubr = UBXReader(stream, protfilter=UBX_PROTOCOL | NMEA_PROTOCOL)

pause = False

IMU_INIT_LABELS = {
    0: 'Not Init',
    1: 'Initializing',
    2: 'Initialized',
    3: 'Initialized',
}

CALIB_STATUS_LABELS = {
    0: 'Not calibrated',
    1: 'Calibrating',
    2: 'Calibrated (coarse)',
    3: 'Calibrated (fine)',
}

SENSOR_TYPE_LABELS = {
    5:  'Gyro Z',
    10: 'Temperature',
    13: 'Gyro Y',
    14: 'Gyro X',
    16: 'Accel X',
    17: 'Accel Y',
    18: 'Accel Z',
}

flag = {
    "ESF-MEAS": False,
    "ESF-INS": False,
    "ESF-STATUS": False,
    "ESF-ALG": True,
    "NAV-ATT": False,
    "GNGGA": False,
    "GNGLL": False,
    "GNRMC": False,
    "GNVTG": False,
    "GNGSA": False,
    "GSV": False,
    "GNZDA": False,
}

try:
    print(f"Starting to parse data from {serial_port}...")
    while True:
        (raw_data, parsed_data) = ubr.read()

        if not parsed_data:
            continue

        identity = parsed_data.identity

        # --- IMU: raw accel and gyro (sensor frame) ---
        if flag["ESF-MEAS"] and identity == 'ESF-MEAS':
            # dataType codes: 14=gyroX, 13=gyroY, 5=gyroZ, 16=accelX, 17=accelY, 18=accelZ
            # dataField is a signed 24-bit two's complement integer
            GYRO_SCALE  = 1 / 4096.0   # deg/s per LSB
            ACCEL_SCALE = 1 / 1024.0   # m/s² per LSB

            def signed24(val):
                return val - (1 << 24) if val & (1 << 23) else val

            sensors = {}
            for i in range(1, 9):
                dtype = getattr(parsed_data, f"dataType_{i:02d}", None)
                dfield = getattr(parsed_data, f"dataField_{i:02d}", None)
                if dtype is None or dfield is None:
                    break
                sensors[dtype] = signed24(dfield)

            gx = sensors.get(14, 0) * GYRO_SCALE
            gy = sensors.get(13, 0) * GYRO_SCALE
            gz = sensors.get(5,  0) * GYRO_SCALE
            ax = sensors.get(16, 0) * ACCEL_SCALE
            ay = sensors.get(17, 0) * ACCEL_SCALE
            az = sensors.get(18, 0) * ACCEL_SCALE

            print(
                f"[ESF-MEAS] "
                f"Accel (m/s²)  X: {ax:+8.4f}  Y: {ay:+8.4f}  Z: {az:+8.4f}  |  "
                f"Gyro (°/s)   X: {gx:+8.4f}  Y: {gy:+8.4f}  Z: {gz:+8.4f}"
            )

        # --- IMU: compensated accel/gyro (vehicle frame) ---
        elif flag["ESF-INS"] and identity == 'ESF-INS':
            print(f"[ESF-INS] Vehicle Frame — xAccel: {parsed_data.xAccel}, yAccel: {parsed_data.yAccel}, zAccel: {parsed_data.zAccel} | xGyro: {parsed_data.xAngRate}, yGyro: {parsed_data.yAngRate}, zGyro: {parsed_data.zAngRate}")

        # --- Sensor fusion status ---
        if flag["ESF-STATUS"] and identity == 'ESF-STATUS':
            print(parsed_data)
            print(f"[ESF-STATUS] Fusion Mode: {parsed_data.fusionMode} | "
                  f"IMU Init: {IMU_INIT_LABELS.get(parsed_data.imuInitStatus)} | "
                  f"INS Init: {IMU_INIT_LABELS.get(parsed_data.insInitStatus)} | "
                  f"Num Sensors: {parsed_data.numSens}")
            for i in range(1, parsed_data.numSens + 1):
                sensor_type = getattr(parsed_data, f"type_{i:02d}", None)
                calib       = getattr(parsed_data, f"calibStatus_{i:02d}", None)
                if sensor_type is None:
                    break
                print(f"  {SENSOR_TYPE_LABELS.get(sensor_type, f'type={sensor_type}')}: {CALIB_STATUS_LABELS.get(calib, calib)}")

        # --- IMU: installation angle ---
        elif flag["ESF-ALG"] and identity == 'ESF-ALG':
            print(parsed_data)
            print(f"[ESF-ALG] Installation Angle — Roll: {parsed_data.roll}, Pitch: {parsed_data.pitch}, Yaw: {parsed_data.yaw}")

        elif flag["NAV-ATT"] and identity == 'NAV-ATT':
            print(f"[NAV-ATT] Orientation — Roll: {parsed_data.roll}, Pitch: {parsed_data.pitch}, Heading: {parsed_data.heading}")

        # --- Position (lat/lon) ---
        elif  flag["GNGGA"] and identity in ('GNGGA', 'GPGGA'):
            print(f"[GGA] Position: {parsed_data.lat}, {parsed_data.lon} | Altitude: {parsed_data.alt}m | HDOP: {parsed_data.HDOP} | Satellites: {parsed_data.numSV} | Time: {parsed_data.time}")

        elif flag["GNGLL"] and identity in ('GNGLL', 'GPGLL'):
            print(f"[GLL] Position: {parsed_data.lat}, {parsed_data.lon} | Time: {parsed_data.time}")

        elif flag["GNRMC"] and identity in ('GNRMC', 'GPRMC'):
            print(f"UTC: {parsed_data.time} [RMC] Position: {parsed_data.lat}, {parsed_data.lon} | Speed: {parsed_data.spd} knots | Time: {parsed_data.date} {parsed_data.time}")
        # --- Speed and course ---
        elif flag["GNVTG"] and identity in ('GNVTG', 'GPVTG'):
            print(f"[VTG] Speed: {parsed_data.sogk} km/h | Course: {parsed_data.cogt} deg true north")

        # --- DOP and active satellites ---
        elif flag["GNGSA"] and identity in ('GNGSA', 'GPGSA'):
            print(f"[GSA] HDOP: {parsed_data.HDOP} | VDOP: {parsed_data.VDOP} | PDOP: {parsed_data.PDOP}")

        # --- Satellites in view ---
        elif flag["GSV"] and identity[-3:] == 'GSV':
            print(f"[GSV] Satellites Visible: {parsed_data.numSV}")
            for i in range(1, 5):
                elv = getattr(parsed_data, f"elv_0{i}", None)
                az = getattr(parsed_data, f"az_0{i}", None)
                if elv is not None and az is not None:
                    print(f"  Sat {i} — Elevation: {elv}, Azimuth: {az}")

        # --- Timestamp ---
        elif flag["GNZDA"] and identity in ('GNZDA', 'GPZDA'):
            print(f"[ZDA] Time: {parsed_data.time} | Date: {parsed_data.day}/{parsed_data.month}/{parsed_data.year}")

except KeyboardInterrupt:
    print("Streaming stopped by user.")
finally:
    stream.close()
