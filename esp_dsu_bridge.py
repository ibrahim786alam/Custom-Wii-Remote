
import socket
import struct
import time
import zlib
import random
import select
import math

# ============================================================
# CONFIGURATION
# ============================================================

ESP_PORT = 4210
DSU_PORT = 26760

DSU_HOST = "0.0.0.0"
ESP_HOST = "0.0.0.0"

STREAM_HZ = 50
DIAGNOSTIC_PRINT_INTERVAL = 0.25  # seconds
ESP_TIMEOUT = 1.0
DSU_CLIENT_TIMEOUT = 5.0

# MPU-6500 scaling used by your existing ESP firmware.
# Assumes accelerometer +/-2g and gyro +/-250 degrees/second.
ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0

# Optional gyro zero-rate offsets, in degrees/second.
# Leave at 0.0 to pass through the original gyro values unchanged.
# These correct a constant gyro bias; they do NOT set a 45-degree heading.
GYRO_OFFSET_X_DPS = 0.0
GYRO_OFFSET_Y_DPS = 0.0
GYRO_OFFSET_Z_DPS = 0.0

# Rotate the IMU's coordinate frame to match how the sensor is mounted.
# If the controller's physical direction is 90 degrees out of alignment,
# try +90.0 first; if it turns the wrong way, try -90.0.
# This rotates BOTH accelerometer and gyro axes; it is not a gyro bias.
SENSOR_ROTATION_Z_DEG = 0.0

# DSU protocol constants
DSU_VERSION = 1001
MSG_VERSION = 0x100000
MSG_CONTROLLER_INFO = 0x100001
MSG_CONTROLLER_DATA = 0x100002

SERVER_ID = random.randint(1, 0xFFFFFFFF)

# Stable controller identifier. Keep unchanged between launches.
CONTROLLER_MAC = bytes([0x02, 0x12, 0x34, 0x56, 0x78, 0x9A])


# ============================================================
# LIVE CONTROLLER STATE
# ============================================================

state = {
    "A": 0,
    "B": 0,
    "AX": 0,
    "AY": 0,
    "AZ": 0,
    "GX": 0,
    "GY": 0,
    "GZ": 0,
}

last_esp_packet = 0.0
packet_number = 0
last_diagnostic_print = 0.0
last_printed_values = None

# DSU clients that have requested controller data.
dsu_clients = {}


# ============================================================
# DSU PACKET BUILDER
# ============================================================

def make_dsu_packet(message_type, payload=b""):
    """
    Build a DSU/Cemuhook UDP packet with a valid header and CRC32.
    """
    packet = bytearray(20 + len(payload))

    packet[0:4] = b"DSUS"
    struct.pack_into("<H", packet, 4, DSU_VERSION)
    struct.pack_into("<H", packet, 6, len(packet) - 16)
    struct.pack_into("<I", packet, 8, 0)
    struct.pack_into("<I", packet, 12, SERVER_ID)
    struct.pack_into("<I", packet, 16, message_type)

    packet[20:] = payload

    checksum = zlib.crc32(packet) & 0xFFFFFFFF
    struct.pack_into("<I", packet, 8, checksum)

    return bytes(packet)


def controller_connected():
    return (
        last_esp_packet != 0.0
        and time.monotonic() - last_esp_packet < ESP_TIMEOUT
    )


def make_controller_info(slot):
    """
    DSU controller-information response for one requested slot.
    """
    connected = controller_connected()
    p = bytearray(12)

    p[0] = slot

    if connected:
        p[1] = 2                  # Connected
        p[2] = 2                  # Full gyro
        p[3] = 1                  # USB-style connection
        p[4:10] = CONTROLLER_MAC
        p[10] = 5                  # Battery: full/not applicable
    else:
        p[1] = 0

    p[11] = 0

    return make_dsu_packet(MSG_CONTROLLER_INFO, bytes(p))


# ============================================================
# ESP8266 UDP PACKET PARSER
# ============================================================

def process_esp_packet(data):
    """
    Expected ESP packet:
    A=0,B=0,AX=...,AY=...,AZ=...,GX=...,GY=...,GZ=...
    """
    global last_esp_packet

    try:
        message = data.decode("utf-8", errors="ignore").strip()
        values = {}

        for item in message.split(","):
            if "=" not in item:
                continue

            key, value = item.split("=", 1)
            values[key.strip().upper()] = int(float(value.strip()))

        required = ("A", "B", "AX", "AY", "AZ", "GX", "GY", "GZ")

        if not all(key in values for key in required):
            return

        for key in required:
            state[key] = values[key]

        last_esp_packet = time.monotonic()
        print_imu_diagnostics()

    except (ValueError, UnicodeError):
        # Ignore malformed or incomplete packets.
        return


# ============================================================
# LIVE RAW-IMU DIAGNOSTICS
# ============================================================

def print_imu_diagnostics():
    """Print latest raw ESP readings for axis diagnosis over Wi-Fi."""
    global last_diagnostic_print, last_printed_values

    now = time.monotonic()
    if now - last_diagnostic_print < DIAGNOSTIC_PRINT_INTERVAL:
        return

    values = (
        state["AX"], state["AY"], state["AZ"],
        state["GX"], state["GY"], state["GZ"],
    )
    if values == last_printed_values:
        return

    last_diagnostic_print = now
    last_printed_values = values
    ax, ay, az, gx, gy, gz = values
    print(
        f"RAW  A[counts]: X={ax:7d} Y={ay:7d} Z={az:7d}  |  "
        f"G[counts]: X={gx:7d} Y={gy:7d} Z={gz:7d}",
        flush=True,
    )


# ============================================================
# DSU CONTROLLER DATA PACKET
# ============================================================

def rotate_sensor_xy(vector):
    """Rotate a 3D sensor vector around its Z axis by the configured angle."""
    x, y, z = vector
    angle = math.radians(SENSOR_ROTATION_Z_DEG)
    c = math.cos(angle)
    sn = math.sin(angle)
    return [x * c - y * sn, x * sn + y * c, z]


def make_controller_data():
    global packet_number

    packet_number = (packet_number + 1) & 0xFFFFFFFF

    connected = controller_connected()
    p = bytearray(80)

    # Preserve the previously working controller-data layout.
    p[0] = 0
    p[1] = 2 if connected else 0
    p[2] = 2
    p[3] = 1
    p[4:10] = CONTROLLER_MAC
    p[10] = 5 if connected else 0
    p[11] = 1 if connected else 0

    struct.pack_into("<I", p, 12, packet_number)

    if connected:
        # Physical A and B are passed through normally. No combo gesture.
        a_pressed = bool(state["A"])
        b_pressed = bool(state["B"])

        # DSU Buttons 2 byte (p[17]):
        # Y=0x80, B=0x40, A=0x20, X=0x10, R1=0x08,
        # L1=0x04, R2=0x02, L2=0x01.
        buttons = 0

        if b_pressed:
            buttons |= 0x40

        if a_pressed:
            buttons |= 0x20

        p[17] = buttons

        # Neutral analog sticks.
        p[20:24] = bytes([128, 128, 128, 128])

        # Analog button pressure fields for physical B and A.
        p[29] = 255 if b_pressed else 0
        p[30] = 255 if a_pressed else 0

        # Keep the original acceleration scale and axis order.
        accel_raw = [
            state["AX"] / ACCEL_SCALE,
            state["AY"] / ACCEL_SCALE,
            state["AZ"] / ACCEL_SCALE,
        ]
        accel = rotate_sensor_xy(accel_raw)

        # Convert gyro readings to degrees/second, correct optional zero-rate
        # bias, then rotate the sensor coordinate frame to match the controller.
        gx = state["GX"] / GYRO_SCALE - GYRO_OFFSET_X_DPS
        gy = state["GY"] / GYRO_SCALE - GYRO_OFFSET_Y_DPS
        gz = state["GZ"] / GYRO_SCALE - GYRO_OFFSET_Z_DPS

        # DSU gyro order: Pitch, Yaw, Roll
        gyro = [gy, gz, gx]

        # Timestamp in microseconds.
        struct.pack_into(
            "<Q", p, 48, time.monotonic_ns() // 1000
        )

        struct.pack_into("<fff", p, 56, *accel)
        struct.pack_into("<fff", p, 68, *gyro)

    return make_dsu_packet(MSG_CONTROLLER_DATA, bytes(p))


# ============================================================
# DSU REQUEST HANDLER
# ============================================================

def process_dsu_request(data, address, dsu_socket):
    """
    Handle DSU requests from Dolphin or another compatible client.
    """
    if len(data) < 20 or data[0:4] != b"DSUC":
        return

    try:
        message_type = struct.unpack_from("<I", data, 16)[0]
        payload = data[20:]

        if message_type == MSG_VERSION:
            response = struct.pack("<H", DSU_VERSION)
            dsu_socket.sendto(
                make_dsu_packet(MSG_VERSION, response), address
            )

        elif message_type == MSG_CONTROLLER_INFO:
            if len(payload) < 4:
                return

            count = struct.unpack_from("<i", payload, 0)[0]
            count = max(0, min(count, 4))

            slots = payload[4:4 + count]

            for slot in slots:
                if slot < 4:
                    dsu_socket.sendto(
                        make_controller_info(slot), address
                    )

        elif message_type == MSG_CONTROLLER_DATA:
            # Register this client for continuous controller updates.
            # A zero action mask requests all controllers.
            if len(payload) >= 1:
                flags = payload[0]

                if flags & 0x01:
                    if len(payload) < 2 or payload[1] != 0:
                        return

                dsu_clients[address] = time.monotonic()

    except (struct.error, OSError):
        return


# ============================================================
# MAIN LOOP
# ============================================================

def main():
    esp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    esp_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    esp_socket.bind((ESP_HOST, ESP_PORT))
    esp_socket.setblocking(False)

    dsu_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dsu_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    dsu_socket.bind((DSU_HOST, DSU_PORT))
    dsu_socket.setblocking(False)

    print("ESP-to-Dolphin DSU bridge started.")
    print("RAW IMU diagnostics enabled: move the controller and watch AX/AY/AZ and GX/GY/GZ.")
    print("Keep still, then roll right, roll left, pitch up/down, and yaw.")
    print(f"Listening for ESP packets on UDP {ESP_PORT}")
    print(f"DSU server listening on UDP {DSU_PORT}")
    print("A+B calibration/virtual-button gesture is disabled; physical A and B pass through normally.")
    print("Edit GYRO_OFFSET_X_DPS / Y_DPS / Z_DPS near the top to adjust gyro zero-rate bias (default 0.0).")
    print(f"IMU coordinate-frame Z rotation: {SENSOR_ROTATION_Z_DEG:+.1f} degrees (edit SENSOR_ROTATION_Z_DEG near the top).")
    print("If directions are worse, stop the bridge and try -90.0 or 0.0; this rotates accel and gyro axes together.")
    print("Press Ctrl+C to stop.\n")

    interval = 1.0 / STREAM_HZ
    next_stream = time.monotonic()

    try:
        while True:
            now = time.monotonic()

            readable, _, _ = select.select(
                [esp_socket, dsu_socket], [], [], 0.002
            )

            for sock in readable:
                try:
                    data, address = sock.recvfrom(2048)

                    if sock is esp_socket:
                        process_esp_packet(data)
                    else:
                        process_dsu_request(data, address, dsu_socket)

                except OSError:
                    pass

            now = time.monotonic()

            # Expire clients that stopped requesting controller data.
            expired = [
                address
                for address, last_seen in dsu_clients.items()
                if now - last_seen > DSU_CLIENT_TIMEOUT
            ]

            for address in expired:
                del dsu_clients[address]

            if now >= next_stream:
                if dsu_clients:
                    packet = make_controller_data()

                    for address in list(dsu_clients):
                        try:
                            dsu_socket.sendto(packet, address)
                        except OSError:
                            pass

                next_stream = now + interval

    except KeyboardInterrupt:
        print("\nBridge stopped.")

    finally:
        esp_socket.close()
        dsu_socket.close()


if __name__ == "__main__":
    main()
