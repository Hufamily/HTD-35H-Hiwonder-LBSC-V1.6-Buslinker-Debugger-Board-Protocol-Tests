import serial, time

PORT      = "COM5"
BAUD      = 9600
SERVO_IDS = [1]          # change or extend, e.g. [1, 2, 3]
WAIT      = 1.2          # seconds to collect relay output


def make_pos_read_packet(servo_ids: list) -> bytes:
    """CMD_MULT_SERVO_POS_READ (0x15)"""
    params = bytes([len(servo_ids)] + servo_ids)
    length = len(params) + 2
    return bytes([0x55, 0x55, length, 0x15]) + params


def parse_pos_response(data: bytes) -> dict:
    """Parse raw response bytes into {servo_id: position}."""
    idx = data.find(b"\x55\x55")
    if idx < 0 or data[idx + 3] != 0x15:
        return {}

    num_servos = data[idx + 4]
    positions  = {}
    offset     = idx + 5
    for _ in range(num_servos):
        sid = data[offset]
        pos = data[offset + 1] | (data[offset + 2] << 8)
        positions[sid] = pos
        offset += 3
    return positions


# ── Send & collect relay output ───────────────────────────────────────────

ser    = serial.Serial(PORT, baudrate=BAUD, timeout=WAIT)
packet = make_pos_read_packet(SERVO_IDS)

print(f"Sending : {packet.hex(' ').upper()}\n")
ser.reset_input_buffer()
ser.write(packet)

deadline = time.time() + WAIT
raw_all  = b""
while time.time() < deadline:
    if ser.in_waiting:
        raw_all += ser.read(ser.in_waiting)
    time.sleep(0.02)
ser.close()

# ── Parse hex from relay "< XX XX ..." lines ─────────────────────────────

response_bytes = b""
for line in raw_all.decode("ascii", errors="replace").splitlines():
    line = line.strip()
    if line.startswith("<"):
        response_bytes = bytes.fromhex(line[1:].strip().replace(" ", ""))
        print(f"Response: {line}")

# ── Display positions ─────────────────────────────────────────────────────

if not response_bytes:
    print("No response received — check wiring / relay sketch")
else:
    positions = parse_pos_response(response_bytes)
    if positions:
        print()
        for sid, pos in positions.items():
            degrees = pos * 240 / 1000
            print(f"  Servo {sid:2d}:  raw={pos:4d}   angle={degrees:.1f}°")
    else:
        print("Could not parse response — raw bytes:", response_bytes.hex(" ").upper())