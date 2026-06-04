import serial
from pyubx2 import UBXReader, UBXMessage, UBX_PROTOCOL, NMEA_PROTOCOL, SET_LAYER_RAM, TXN_NONE, POLL
import time

stream = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

msgout_keys = [
    # ("CFG-SFCORE-USE_SF", 1),
    # ("CFG-SFIMU-IMU_EN", 1),
    # ("CFG-SFIMU-AUTO_MNTALG_ENA", 1),
    ("CFG-MSGOUT-UBX_ESF_RAW_USB",    1),
    ("CFG-MSGOUT-UBX_ESF_ALG_USB",    1),
    ("CFG-MSGOUT-UBX_ESF_INS_USB",    1),
    ("CFG-MSGOUT-UBX_ESF_STATUS_USB", 1),
    ("CFG-MSGOUT-UBX_NAV2_PVT_USB", 1),
    ("CFG-MSGOUT-UBX_NAV_SAT_USB", 1),
    ("CFG-MSGOUT-UBX_NAV_ATT_USB", 1),
    # ("CFG-MSGOUT-NMEA_ID_ZDA_USB", 1),
    # ("CFG-MSGOUT-NMEA_ID_RMC_USB", 1),
]

rate_keys = [
    ("CFG-RATE-MEAS", 25),
    ("CFG-RATE-NAV",   3),
]

def send_cfg(stream, keys, label):
    msg = UBXMessage.config_set(SET_LAYER_RAM, TXN_NONE, keys)
    stream.write(msg.serialize())
    stream.flush()
    time.sleep(0.1)

send_cfg(stream, msgout_keys, "MSGOUT config")
send_cfg(stream, rate_keys,   "RATE config")
stream.reset_input_buffer()

ubr = UBXReader(stream, protfilter=UBX_PROTOCOL | NMEA_PROTOCOL)

poll_msg = UBXMessage("NAV", "NAV-ORB", POLL)
stream.write(poll_msg.serialize())
stream.flush()

for _ in range(200):
    _, parsed = ubr.read()
    if parsed is None:
        continue
    if parsed.identity == "NAV-ORB":
        print("NAV-ORB supported:", parsed)
        break
    if parsed.identity == "ACK-NAK":
        print("ACK-NAK: NAV-ORB not supported on this firmware")
        break


start_time = time.time()
counter = {}
esf_raw = {16: 0.0, 17: 0.0, 18: 0.0, 14: 0.0, 13: 0.0, 5: 0.0}
GYRO_SCALE  = 1 / 4096.0
ACCEL_SCALE = 1 / 1024.0

def signed24(val):
    return val - (1 << 24) if val & (1 << 23) else val

while time.time() < start_time + 5:
    (_, parsed_data) = ubr.read()

    if parsed_data is None:
        continue

    if parsed_data.identity == "ESF-MEAS":
        counter["ESF-MEAS"] = counter.get("ESF-MEAS", 0) + 1
        samples = {}
        for i in range(1, 9):
            dtype  = getattr(parsed_data, f"dataType_{i:02d}", None)
            dfield = getattr(parsed_data, f"dataField_{i:02d}", None)
            if dtype is None or dfield is None:
                break
            samples[dtype] = signed24(dfield)
            if not any(k in samples for k in (5, 13, 14, 16, 17, 18)):
                msg = None
            else:
                for k, v in samples.items():
                    if k in esf_raw:
                        esf_raw[k] = v
                ax = esf_raw[16] * ACCEL_SCALE
                ay = esf_raw[17] * ACCEL_SCALE
                az = esf_raw[18] * ACCEL_SCALE
                gx = esf_raw[14] * GYRO_SCALE
                gy = esf_raw[13] * GYRO_SCALE
                gz = esf_raw[5]  * GYRO_SCALE
    
    if parsed_data.identity == "NAV-SAT":
        counter["NAV-SAT"] = counter.get("NAV-SAT", 0) + 1

    if parsed_data.identity == "NAV-ATT":
        print(parsed_data)
        counter["NAV-ATT"] = counter.get("NAV-ATT", 0) + 1

print(counter)


