"""
test.py  —  Hiwonder Probe, Relay-Aware Edition
================================================
Written specifically for relay.ino:

    void setup() {
        Serial.begin(9600);    // USB  ↔  PC
        Serial1.begin(9600);   // UART ↔  Servo / Controller board
    }
    void loop() {
        if (Serial.available())  Serial1.write(Serial.read());
        if (Serial1.available()) Serial.write(Serial1.read());
    }

Why the previous script got 159 silent probes
─────────────────────────────────────────────
  The previous script changed baud rates (9600 / 19200 / 38400 / 57600 /
  115200) but the Arduino's Serial port is FIXED at 9600. Sending at any
  other rate produces framing errors — the Arduino receives garbage and
  forwards garbage (or nothing) on Serial1. Every non-9600 probe was
  dead on arrival.

  Additionally the relay is byte-by-byte with a tiny loop delay, so the
  Arduino introduces ~1 ms of latency per byte. A 6-byte packet at 9600
  takes ~6 ms to transmit plus ~6 ms relay forwarding — the script's
  read window must account for this.

Key constraints this script respects
─────────────────────────────────────
  1. PC ↔ Arduino  : always 9600 baud  (Serial is hardcoded 9600)
  2. Arduino ↔ Servo: always 9600 baud (Serial1 is hardcoded 9600)
  3. HTD-35H direct servo bus runs at 115200 → will NOT work through
     this relay. You would need to change Serial1.begin(9600) to
     Serial1.begin(115200) in the sketch for direct-servo testing.
  4. Packet TX echo: if the servo bus is wired half-duplex (TX1 + RX1
     merged with a resistor), the Arduino's own TX bytes come back on
     RX1 → relay forwards them to PC before the servo reply arrives.
     This script detects and strips that echo automatically.
  5. Relay latency: read window = packet_tx_time + relay_latency +
     servo_response_time. At 9600 baud a 6-byte packet takes ~6 ms.
     We use a generous 800 ms window and drain in multiple reads.

What this script tests
───────────────────────
  Phase 0  Confirm the Arduino relay is alive and forwarding bytes
             • Send a known byte pattern, look for relay echo
             • Drain any startup garbage
             • Byte-timing loopback test

  Phase 1  Controller board protocol  (Protocol A / B)
             Both use 55 55 [Len] [Cmd] [Params], 9600 baud — perfect
             for relay.ino as-is.
             • CMD_GET_BATTERY_VOLTAGE  0x0F   (best ping, no side-effects)
             • CMD_MULT_SERVO_POS_READ  0x15   (servo 1 position)
             • CMD_ACTION_GROUP_STOP    0x07   (safe write, echoed back)
             • CMD_MULT_SERVO_UNLOAD    0x14   (disable torque — safe)

  Phase 2  Direct bus servo at 9600  (Protocol C, WRONG baud)
             Will almost certainly fail — included to confirm failure
             and tell you exactly what to change in relay.ino to fix it.
             • SERVO_POS_READ  cmd=28   IDs 1, 254 (broadcast)
             • SERVO_VIN_READ  cmd=27   ID 1

  Phase 3  Echo / wiring diagnostics
             • Short loopback: TX1–RX1 shorted → check relay path
             • Half-duplex echo detection
             • Timing sweep: read at 50 / 200 / 500 / 800 ms after TX
             • Flush test: send garbage, look for any byte back

  Phase 4  Upgraded relay.ino suggestions
             Prints a patched relay.ino that supports baud-switching
             so the PC can use both 9600 (controller board) and
             115200 (direct servo) through the same Arduino.

Usage
─────
  python test.py              # COM5 (Windows default)
  python test.py COM3
  python test.py /dev/ttyUSB0
  python test.py COM5 --phase 1       # run only phase 1
  python test.py COM5 --verbose       # print every byte timing detail
"""

import argparse
import math
import sys
import time
import serial
import serial.tools.list_ports
from datetime import datetime

# ─────────────────────────────────────────────────────────────────────────────
#  Constants
# ─────────────────────────────────────────────────────────────────────────────

# The Arduino relay is FIXED at 9600 on both ports. NEVER change this.
ARDUINO_BAUD = 9600

# At 9600 baud: 1 byte = 10 bits = 1.042 ms
# Relay loop() overhead: ~0.1 ms per iteration (conservative)
BYTE_MS = 1.042  # ms per byte at 9600

def relay_latency_ms(n_bytes: int) -> float:
    """Estimated ms for n bytes to travel PC→Arduino→Serial1 and back."""
    tx_ms  = n_bytes * BYTE_MS       # time to clock out TX bytes
    fwd_ms = n_bytes * BYTE_MS       # relay forwarding (byte by byte)
    rx_ms  = 8 * BYTE_MS             # typical response (8 bytes)
    back_ms = 8 * BYTE_MS            # relay back to PC
    return tx_ms + fwd_ms + rx_ms + back_ms + 20  # +20 ms safety margin

READ_WINDOW = 0.8   # seconds — generous window covering worst case

DEFAULT_PORT = "COM5"


# ─────────────────────────────────────────────────────────────────────────────
#  Packet builders
# ─────────────────────────────────────────────────────────────────────────────

def ctrl_pkt(cmd: int, params: bytes = b"") -> bytes:
    """
    Protocol A/B — controller board  (works through relay.ino as-is).
    55 55  [Length=N+2]  [Cmd]  [Params...]
    No servo ID, no checksum.
    """
    return bytes([0x55, 0x55, len(params) + 2, cmd]) + params


def bus_pkt(servo_id: int, cmd: int, params: bytes = b"") -> bytes:
    """
    Protocol C — direct bus servo  (requires Serial1 at 115200, NOT 9600).
    55 55  [ID]  [Length=N+3]  [Cmd]  [Params...]  [Checksum]
    Checksum = ~(ID+Length+Cmd+ΣParams) & 0xFF
    """
    length = len(params) + 3
    chk = (~(servo_id + length + cmd + sum(params))) & 0xFF
    return bytes([0x55, 0x55, servo_id, length, cmd]) + params + bytes([chk])


# ─────────────────────────────────────────────────────────────────────────────
#  Serial helpers
# ─────────────────────────────────────────────────────────────────────────────

def drain(ser: serial.Serial, wait: float = 0.15) -> bytes:
    """Read everything waiting in the RX buffer, with a brief wait."""
    time.sleep(wait)
    data = b""
    while ser.in_waiting:
        data += ser.read(ser.in_waiting)
        time.sleep(0.02)
    return data


def send_recv(ser: serial.Serial, pkt: bytes,
              window: float = READ_WINDOW, verbose: bool = False) -> bytes:
    """
    Flush RX, send pkt, wait for the relay to forward everything and the
    device to respond, then read all available bytes.
    Also drains in a loop to catch slow relay dribble.
    """
    ser.reset_input_buffer()
    ser.write(pkt)

    # Calculate when the last byte of our packet finishes transmitting
    tx_done_at = time.time() + len(pkt) * BYTE_MS / 1000.0

    deadline = time.time() + window
    data = b""
    while time.time() < deadline:
        if ser.in_waiting:
            chunk = ser.read(ser.in_waiting)
            data += chunk
            if verbose:
                print(f"          +{(time.time()-tx_done_at)*1000:.1f} ms  chunk: {chunk.hex(' ').upper()}")
            # If we have more than 2× the TX packet, likely got echo + response
            if len(data) >= len(pkt) * 2 + 4:
                break
        time.sleep(0.01)

    return data


def strip_echo(raw: bytes, sent: bytes) -> tuple[bytes, bool]:
    """
    If the first N bytes of raw match sent exactly, strip them.
    Returns (stripped_data, echo_was_present).
    Half-duplex wired correctly: TX bytes bounce back on RX1 → relay → PC.
    """
    if raw[:len(sent)] == sent:
        return raw[len(sent):], True
    # Partial echo check
    for n in range(len(sent) - 1, 0, -1):
        if raw[:n] == sent[:n]:
            return raw[n:], True
    return raw, False


# ─────────────────────────────────────────────────────────────────────────────
#  Response parsers
# ─────────────────────────────────────────────────────────────────────────────

def find_55(data: bytes) -> int:
    idx = data.find(b"\x55\x55")
    return idx


def parse_ctrl(data: bytes) -> dict | None:
    idx = find_55(data)
    if idx < 0 or idx + 3 >= len(data):
        return None
    length = data[idx + 2]
    cmd    = data[idx + 3]
    if length < 2:
        return None
    n   = length - 2
    end = idx + 4 + n
    if end > len(data):
        return None
    return {"cmd": cmd, "length": length,
            "params": data[idx + 4:end], "raw": data[idx:end]}


def parse_bus(data: bytes) -> dict | None:
    idx = find_55(data)
    if idx < 0 or idx + 5 >= len(data):
        return None
    sid    = data[idx + 2]
    length = data[idx + 3]
    cmd    = data[idx + 4]
    if length < 3:
        return None
    n  = length - 3
    pe = idx + 5 + n
    if pe >= len(data):
        return None
    params = data[idx + 5:pe]
    chk    = data[pe]
    if ((~(sid + length + cmd + sum(params))) & 0xFF) != chk:
        return None
    return {"servo_id": sid, "cmd": cmd, "params": params,
            "raw": data[idx:pe + 1]}


def u16(lo: int, hi: int) -> int:
    return (hi << 8) | lo

def s16(lo: int, hi: int) -> int:
    v = u16(lo, hi)
    return v if v < 32768 else v - 65536


# ─────────────────────────────────────────────────────────────────────────────
#  Result tracker
# ─────────────────────────────────────────────────────────────────────────────

class Results:
    def __init__(self):
        self._rows = []

    def add(self, key: str, hit: bool, note: str = "", raw: bytes = b""):
        self._rows.append((key, hit, note, raw))

    def hits(self, prefix: str = "") -> list:
        return [(k, n, r) for k, h, n, r in self._rows
                if h and k.startswith(prefix)]

    def any_hit(self, prefix: str = "") -> bool:
        return any(h for k, h, _, _ in self._rows if k.startswith(prefix))

    def all_rows(self):
        return self._rows


def pr(label: str, pkt: bytes, raw: bytes, ok: bool, detail: str = ""):
    tag = "✓" if ok else "·"
    rx  = raw.hex(" ").upper() if raw else "(silent)"
    print(f"\n  {tag} {label}")
    print(f"      TX : {pkt.hex(' ').upper()}")
    print(f"      RX : {rx}")
    if detail:
        print(f"      >>> {detail}")


# ─────────────────────────────────────────────────────────────────────────────
#  Phase 0 — Confirm the Arduino relay path is alive
# ─────────────────────────────────────────────────────────────────────────────

def phase0(ser: serial.Serial, res: Results, verbose: bool):
    print()
    print("━" * 70)
    print("  PHASE 0  ·  Arduino Relay Path Diagnostics")
    print(f"  relay.ino: Serial=9600, Serial1=9600, byte-by-byte passthrough")
    print("━" * 70)

    # 0a — Drain startup noise ─────────────────────────────────────────────
    print("\n  [0a] Draining startup noise (1 s)...")
    noise = drain(ser, 1.0)
    if noise:
        print(f"       RX: {noise.hex(' ').upper()}")
        try:
            print(f"       TEXT: {repr(noise.decode('utf-8', errors='replace'))}")
        except Exception:
            pass
        res.add("0a_noise", True, f"startup bytes={noise.hex()}", noise)
        print("       → Arduino sent startup data — serial path is live")
    else:
        print("       (silent — relay.ino has no startup print, this is normal)")
        res.add("0a_noise", False, "no startup data", b"")

    # 0b — List COM ports ──────────────────────────────────────────────────
    print("\n  [0b] Available COM ports:")
    for p in serial.tools.list_ports.comports():
        mark = "  ◄ IN USE" if p.device == ser.port else ""
        print(f"       {p.device}: {p.description}{mark}")

    # 0c — Single-byte round-trip (TX1/RX1 shorted = loopback test) ───────
    print("\n  [0c] Single-byte round-trip test (TX1↔RX1 shorted loopback):")
    print("       (Only passes if TX1 and RX1 are shorted together on Arduino)")
    print("       This confirms the PC→Arduino→Serial1→Arduino→PC path works.")
    results_loop = []
    for byte_val in [0xAA, 0x55, 0xAB, 0xCD]:
        ser.reset_input_buffer()
        ser.write(bytes([byte_val]))
        time.sleep(0.08)   # 1 byte at 9600 = 1 ms; relay adds ~1 ms; 80 ms generous
        rx = bytes(ser.read(ser.in_waiting or 4))
        came_back = byte_val in rx
        results_loop.append(came_back)
        tag = "✓ echoed back" if came_back else "· not echoed"
        print(f"       TX: {byte_val:02X}   RX: {rx.hex(' ').upper() if rx else '(empty)'}   [{tag}]")
    loopback_ok = all(results_loop)
    if loopback_ok:
        print("       ✓  All bytes echoed — TX1/RX1 loopback confirmed")
        print("          Remove the short before connecting servo!")
        res.add("0c_loopback", True, "TX1/RX1 shorted confirmed", b"")
    else:
        n = sum(results_loop)
        if n > 0:
            print(f"       ~ Partial echo ({n}/4 bytes returned) — possible wiring issue")
            res.add("0c_loopback", False, f"partial {n}/4", b"")
        else:
            print("       · No loopback — either TX1/RX1 not shorted (expected in normal use),")
            print("         or Serial1 is not sending/receiving at all")
            res.add("0c_loopback", False, "no loopback", b"")

    # 0d — Multi-byte relay timing test ───────────────────────────────────
    print("\n  [0d] Multi-byte relay timing test  (requires TX1/RX1 shorted):")
    test_pkt = bytes([0x55, 0x55, 0x02, 0x07])   # ctrl stop packet
    ser.reset_input_buffer()
    ser.write(test_pkt)
    # Sample at 50 ms, 150 ms, 300 ms, 600 ms
    snapshots = []
    t0 = time.time()
    accumulated = b""
    for wait_ms in [50, 150, 300, 600]:
        target = t0 + wait_ms / 1000.0
        time.sleep(max(0, target - time.time()))
        chunk = bytes(ser.read(ser.in_waiting or 32))
        accumulated += chunk
        if chunk and verbose:
            elapsed = (time.time() - t0) * 1000
            print(f"       +{elapsed:5.1f} ms  got: {chunk.hex(' ').upper()}")
        snapshots.append((wait_ms, len(accumulated)))

    if verbose:
        print(f"       Final accumulated: {accumulated.hex(' ').upper()}")
    if accumulated == test_pkt:
        print(f"       ✓  Exact packet echo at ~{snapshots[0][0]}-{snapshots[-1][0]} ms")
        print(f"          (loopback wiring confirmed, latency normal for 9600 baud relay)")
        res.add("0d_timing", True, "timing ok", accumulated)
    elif accumulated:
        # Calculate expected relay latency
        expected_echo_ms = math.ceil(len(test_pkt) * BYTE_MS * 2 + 5)
        print(f"       Got {len(accumulated)} byte(s): {accumulated.hex(' ').upper()}")
        print(f"       Expected relay echo time: ~{expected_echo_ms} ms at 9600 baud")
        res.add("0d_timing", False, f"partial={accumulated.hex()}", accumulated)
    else:
        print(f"       · Nothing received (TX1/RX1 may not be shorted — normal in actual use)")
        res.add("0d_timing", False, "silent", b"")


# ─────────────────────────────────────────────────────────────────────────────
#  Phase 1 — Controller board protocol through relay  (9600, works as-is)
# ─────────────────────────────────────────────────────────────────────────────

def phase1(ser: serial.Serial, res: Results, verbose: bool):
    print()
    print("━" * 70)
    print("  PHASE 1  ·  Controller Board Protocol  (A/B)")
    print("  Format: 55 55 [Len] [Cmd] [Params]   — 9600 baud — works through relay")
    print("  Boards: LBSC1.6 / Bus Servo Controller / LSC-6/16/24/32")
    print("━" * 70)

    def do(label: str, pkt: bytes, key: str,
           checker=None, detail_fn=None):
        raw = send_recv(ser, pkt, verbose=verbose)
        stripped, had_echo = strip_echo(raw, pkt)
        echo_note = "  [TX echo stripped]" if had_echo else ""
        parsed = parse_ctrl(stripped)
        ok = checker(parsed) if (checker and parsed) else bool(parsed)
        detail = (detail_fn(parsed) if (ok and detail_fn and parsed) else "") + echo_note
        pr(label, pkt, raw, ok, detail)
        res.add(key, ok, detail, raw)
        return ok, parsed, stripped

    # 1a — Battery voltage — best non-destructive ping ─────────────────────
    do("CMD_GET_BATTERY_VOLTAGE  cmd=0x0F  (read battery mV)",
       ctrl_pkt(0x0F), "1a_batt",
       checker=lambda p: p["cmd"] == 0x0F and len(p["params"]) >= 2,
       detail_fn=lambda p: f"battery={u16(p['params'][0], p['params'][1])} mV  "
                           f"({u16(p['params'][0], p['params'][1])/1000:.2f} V)")

    # 1b — Read servo 1 position ────────────────────────────────────────────
    ok, parsed, stripped = do(
        "CMD_MULT_SERVO_POS_READ  cmd=0x15  servo_id=1",
        ctrl_pkt(0x15, bytes([0x01, 0x01])), "1b_pos",
        checker=lambda p: p["cmd"] == 0x15 and len(p["params"]) >= 4)
    if ok and parsed:
        pos = u16(parsed["params"][2], parsed["params"][3])
        variant = ("Protocol-A (bus servo, 0-1000)"
                   if pos <= 1000
                   else "Protocol-B (PWM servo, 500-2500)")
        print(f"      >>> pos={pos}  angle≈{pos*240/1000:.1f}°  → {variant}")

    # 1c — Action group stop — safe write, board echoes it back ────────────
    do("CMD_ACTION_GROUP_STOP  cmd=0x07  (safe ping, board echoes)",
       ctrl_pkt(0x07), "1c_stop",
       checker=lambda p: p["cmd"] == 0x07,
       detail_fn=lambda p: "board confirmed stop")

    # 1d — Unload servo 1 (disable torque — servo goes limp, safe) ─────────
    do("CMD_MULT_SERVO_UNLOAD  cmd=0x14  servo_id=1  (limp mode)",
       ctrl_pkt(0x14, bytes([0x01, 0x01])), "1d_unload")

    # 1e — Run action group 0 once (only has effect if group 0 is stored) ──
    do("CMD_ACTION_GROUP_RUN  cmd=0x06  group=0  times=1  (may be no-op)",
       ctrl_pkt(0x06, bytes([0x00, 0x01, 0x00])), "1e_run",
       checker=lambda p: p["cmd"] == 0x06,
       detail_fn=lambda p: "board confirmed action run")

    # 1f — Multiple servo IDs ───────────────────────────────────────────────
    print("\n  ── Sweep servo IDs 1..6 for position ─────────────────────────────")
    for sid in range(1, 7):
        pkt = ctrl_pkt(0x15, bytes([0x01, sid]))
        raw = send_recv(ser, pkt, verbose=verbose)
        stripped, _ = strip_echo(raw, pkt)
        parsed = parse_ctrl(stripped)
        ok = bool(parsed and parsed["cmd"] == 0x15 and len(parsed["params"]) >= 4)
        detail = ""
        if ok:
            pos = u16(parsed["params"][2], parsed["params"][3])
            detail = f"pos={pos}  angle≈{pos*240/1000:.1f}°"
        pr(f"POS_READ  servo_id={sid}", pkt, raw, ok, detail)
        res.add(f"1f_pos_id{sid}", ok, detail, raw)


# ─────────────────────────────────────────────────────────────────────────────
#  Phase 2 — Direct bus servo at 9600  (will fail — explains why)
# ─────────────────────────────────────────────────────────────────────────────

def phase2(ser: serial.Serial, res: Results, verbose: bool):
    print()
    print("━" * 70)
    print("  PHASE 2  ·  Direct Bus Servo at 9600 baud  (HTD-35H / HTS-35H)")
    print("  NOTE: These servos require 115200 baud.")
    print("  relay.ino has Serial1.begin(9600) → will fail.")
    print("  Included to confirm the failure and show the fix.")
    print("━" * 70)
    print()
    print("  To make this work, change relay.ino line 3 to:")
    print("    Serial1.begin(115200);")
    print("  Then the PC must ALSO talk at 9600 (Serial is unchanged).")
    print("  The relay bridges 9600 ↔ 115200 transparently.")
    print()

    for sid in [1, 254]:
        for cmd, label in [(28, "POS_READ"), (27, "VIN_READ"), (14, "ID_READ")]:
            pkt = bus_pkt(sid, cmd)
            raw = send_recv(ser, pkt, verbose=verbose)
            stripped, had_echo = strip_echo(raw, pkt)
            parsed = parse_bus(stripped)
            ok = bool(parsed and parsed["cmd"] == cmd)
            detail = ""
            if ok:
                if cmd == 28 and len(parsed["params"]) >= 2:
                    detail = f"pos={s16(parsed['params'][0], parsed['params'][1])}"
                elif cmd == 27 and len(parsed["params"]) >= 2:
                    detail = f"vin={u16(parsed['params'][0], parsed['params'][1])} mV"
                elif cmd == 14 and parsed["params"]:
                    detail = f"servo_id={parsed['params'][0]}"
            echo_note = "  [TX echo stripped]" if had_echo else ""
            pr(f"BUS {label}  cmd={cmd}  id={sid}", pkt, raw, ok, detail + echo_note)
            res.add(f"2_{label.lower()}_id{sid}", ok, detail, raw)


# ─────────────────────────────────────────────────────────────────────────────
#  Phase 3 — Echo & wiring diagnostics
# ─────────────────────────────────────────────────────────────────────────────

def phase3(ser: serial.Serial, res: Results, verbose: bool):
    print()
    print("━" * 70)
    print("  PHASE 3  ·  Echo & Wiring Diagnostics")
    print("━" * 70)

    # 3a — TX echo presence / half-duplex detection ────────────────────────
    print("\n  [3a] Half-duplex TX echo detection:")
    print("       Sending a ctrl packet, checking if TX bytes come back immediately.")
    print("       If TX1+RX1 are merged with 470Ω resistor → echo expected.")
    print("       If TX1 and RX1 are separate (no merge) → no echo.")
    pkt = ctrl_pkt(0x0F)
    ser.reset_input_buffer()
    ser.write(pkt)
    # Read very quickly — before servo could possibly respond
    time.sleep(len(pkt) * BYTE_MS / 1000.0 * 3)   # 3× TX time
    early = bytes(ser.read(ser.in_waiting or 32))
    time.sleep(READ_WINDOW)
    late = bytes(ser.read(ser.in_waiting or 64))
    print(f"       TX      : {pkt.hex(' ').upper()}")
    print(f"       Early RX: {early.hex(' ').upper() if early else '(empty)'}")
    print(f"       Late RX : {late.hex(' ').upper() if late else '(empty)'}")
    if early == pkt:
        print("       ✓  TX echo present — half-duplex wiring detected (TX1+RX1 merged)")
        res.add("3a_hdx_echo", True, "echo=tx_pkt", early)
    elif early:
        print("       ~  Partial early data — possible partial echo or device response")
        res.add("3a_hdx_echo", False, f"partial={early.hex()}", early)
    else:
        print("       ·  No early echo — TX1 and RX1 appear to be separate (or no device)")
        res.add("3a_hdx_echo", False, "no echo", b"")

    # 3b — Timing sweep (read at multiple delays) ──────────────────────────
    print("\n  [3b] Response timing sweep  (battery cmd, read at 50/200/500/1000 ms):")
    pkt = ctrl_pkt(0x0F)
    ser.reset_input_buffer()
    ser.write(pkt)
    t0 = time.time()
    all_rx = b""
    for wait_ms in [50, 200, 500, 1000]:
        target = t0 + wait_ms / 1000.0
        time.sleep(max(0, target - time.time()))
        chunk = bytes(ser.read(ser.in_waiting or 64))
        all_rx += chunk
        label = f"+{wait_ms:4d} ms"
        if chunk:
            print(f"       {label}: {chunk.hex(' ').upper()}")
        else:
            print(f"       {label}: (nothing new)")
    if all_rx:
        stripped, _ = strip_echo(all_rx, pkt)
        parsed = parse_ctrl(stripped)
        if parsed and parsed["cmd"] == 0x0F:
            mv = u16(parsed["params"][0], parsed["params"][1]) if len(parsed["params"]) >= 2 else "?"
            print(f"       ✓  Valid response at some point  → battery={mv} mV")
            res.add("3b_timing", True, f"battery={mv}mV", all_rx)
        else:
            print(f"       ·  Got bytes but no valid ctrl response  ({len(all_rx)} bytes total)")
            res.add("3b_timing", False, f"raw={all_rx.hex()}", all_rx)
    else:
        print("       ·  Complete silence")
        res.add("3b_timing", False, "silent", b"")

    # 3c — Garbage flush (check if any byte triggers a response) ──────────
    print("\n  [3c] Garbage byte flush test  (check if anything triggers a reply):")
    garbage = bytes([0xFF, 0x00, 0xFF, 0x00, 0xAA, 0x55, 0x01, 0x02])
    pkt = garbage
    raw = send_recv(ser, pkt, verbose=verbose)
    stripped, _ = strip_echo(raw, pkt)
    if stripped:
        print(f"       TX : {pkt.hex(' ').upper()}")
        print(f"       RX : {raw.hex(' ').upper()}")
        print("       ~  Got a response to garbage — device may be in an error state")
        res.add("3c_garbage", True, f"rx={stripped.hex()}", stripped)
    else:
        print(f"       TX : {pkt.hex(' ').upper()}")
        print("       ·  No response to garbage (expected)")
        res.add("3c_garbage", False, "no response", b"")

    # 3d — Vary inter-byte gap (send packet 1 byte at a time) ──────────────
    print("\n  [3d] Slow byte-by-byte send  (50 ms gap between bytes):")
    print("       relay.ino relays byte-by-byte; the device gets bytes with gaps.")
    print("       Some devices accept this; others need a full unbroken packet.")
    pkt = ctrl_pkt(0x0F)
    ser.reset_input_buffer()
    for byte in pkt:
        ser.write(bytes([byte]))
        time.sleep(0.05)   # 50 ms inter-byte gap
    time.sleep(READ_WINDOW)
    raw = bytes(ser.read(ser.in_waiting or 64))
    stripped, _ = strip_echo(raw, pkt)
    parsed = parse_ctrl(stripped)
    ok = bool(parsed and parsed["cmd"] == 0x0F and len(parsed["params"]) >= 2)
    if ok:
        mv = u16(parsed["params"][0], parsed["params"][1])
        print(f"       ✓  Valid response even with 50 ms byte gaps → battery={mv} mV")
        print("          Device accepts fragmented packets (relay timing is fine)")
        res.add("3d_slowbytes", True, f"battery={mv}mV", raw)
    else:
        if stripped:
            print(f"       ~  Got {len(stripped)} bytes but no valid parse: {stripped.hex(' ').upper()}")
        else:
            print("       ·  Silent — device may reject packets with gaps (or not connected)")
        res.add("3d_slowbytes", False, stripped.hex() if stripped else "silent", raw)

    # 3e — Repeat battery read 5× (check consistency) ─────────────────────
    print("\n  [3e] Repeat battery read x5  (check consistency):")
    readings = []
    for i in range(5):
        pkt = ctrl_pkt(0x0F)
        raw = send_recv(ser, pkt, window=0.6, verbose=False)
        stripped, _ = strip_echo(raw, pkt)
        parsed = parse_ctrl(stripped)
        if parsed and parsed["cmd"] == 0x0F and len(parsed["params"]) >= 2:
            mv = u16(parsed["params"][0], parsed["params"][1])
            readings.append(mv)
            print(f"       [{i+1}] ✓  {mv} mV")
        else:
            print(f"       [{i+1}] · (no valid response)  raw={raw.hex(' ').upper() if raw else 'silent'}")
            readings.append(None)
        time.sleep(0.1)
    valid = [v for v in readings if v is not None]
    if valid:
        spread = max(valid) - min(valid)
        print(f"       ✓  {len(valid)}/5 valid  range={min(valid)}-{max(valid)} mV  spread={spread} mV")
        res.add("3e_repeat", True, f"readings={valid}", b"")
    else:
        print("       ·  No valid readings")
        res.add("3e_repeat", False, "no readings", b"")


# ─────────────────────────────────────────────────────────────────────────────
#  Phase 4 — Upgraded relay.ino suggestions
# ─────────────────────────────────────────────────────────────────────────────

RELAY_V2 = r"""
// relay_v2.ino  ─  Baud-aware passthrough for Hiwonder servo testing
//
// Changes from relay.ino:
//   • Serial1 at 115200 instead of 9600
//     → allows direct HTD-35H/HTS-35H bus servo commands
//   • LED blink on TX/RX for visual confirmation
//   • Startup message so PC knows Arduino is alive

#define LED_PIN 13   // built-in LED on most Arduino boards

void setup() {
    Serial.begin(9600);       // USB <-> PC  (always 9600)
    Serial1.begin(115200);    // UART <-> Servo bus  (115200 for direct servo)
    pinMode(LED_PIN, OUTPUT);
    
    // Blink 3x on startup so we know the sketch loaded
    for (int i = 0; i < 3; i++) {
        digitalWrite(LED_PIN, HIGH); delay(100);
        digitalWrite(LED_PIN, LOW);  delay(100);
    }
    
    // Send a banner so the PC knows we're alive
    Serial.println("RELAY_V2_READY");
}

void loop() {
    if (Serial.available()) {
        byte b = Serial.read();
        Serial1.write(b);
        // Blink LED briefly on each byte sent to servo
        digitalWrite(LED_PIN, HIGH);
        delayMicroseconds(200);
        digitalWrite(LED_PIN, LOW);
    }
    if (Serial1.available()) {
        Serial.write(Serial1.read());
    }
}
"""

RELAY_V3 = r"""
// relay_v3.ino  ─  Buffered passthrough with packet framing detection
//
// Improvements over relay_v2:
//   • Buffers incoming bytes until 55 55 header is complete before forwarding
//   • This prevents timing issues if servo needs a complete packet at once
//   • Status LED indicates RX from servo (data coming back)

#define LED_RX  13   // blinks when servo sends data back to PC

byte buf[64];
int  buf_pos = 0;
bool in_pkt  = false;

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);
    pinMode(LED_RX, OUTPUT);
    Serial.println("RELAY_V3_READY");
}

void loop() {
    // PC -> Servo (buffered)
    while (Serial.available()) {
        byte b = Serial.read();
        if (buf_pos < 64) buf[buf_pos++] = b;
        
        // Flush buffer to Serial1 when we see the header + length
        if (buf_pos >= 4 && buf[0] == 0x55 && buf[1] == 0x55) {
            int expected = buf[2] + 2;   // Length field + 2 header bytes
            if (buf_pos >= expected) {
                Serial1.write(buf, expected);
                buf_pos = 0;
                memset(buf, 0, sizeof(buf));
            }
        } else if (buf_pos >= 2 && !(buf[0] == 0x55 && buf[1] == 0x55)) {
            // Not a valid header, flush and reset
            buf_pos = 0;
        }
    }
    
    // Servo -> PC (immediate passthrough)
    while (Serial1.available()) {
        byte b = Serial1.read();
        Serial.write(b);
        digitalWrite(LED_RX, HIGH);
        delayMicroseconds(500);
        digitalWrite(LED_RX, LOW);
    }
}
"""


def phase4(verbose: bool):
    print()
    print("━" * 70)
    print("  PHASE 4  ·  Suggested relay.ino Improvements")
    print("━" * 70)
    print()
    print("  CURRENT relay.ino LIMITATIONS:")
    print("  ┌──────────────────────────────────────────────────────────────┐")
    print("  │ 1. Serial1 at 9600 → HTD-35H direct servo WILL NOT work     │")
    print("  │    HTD-35H requires 115200. Only LBSC/LSC boards work as-is. │")
    print("  │                                                              │")
    print("  │ 2. Byte-by-byte relay introduces ~1 ms/byte latency.        │")
    print("  │    At 9600 baud a 6-byte packet takes ~6 ms to relay.       │")
    print("  │    Most devices tolerate this; fragile ones may time out.   │")
    print("  │                                                              │")
    print("  │ 3. No startup message → PC cannot confirm Arduino is live.   │")
    print("  │    The only confirmation is the loopback test (Phase 0c).   │")
    print("  │                                                              │")
    print("  │ 4. No LED feedback → cannot tell if bytes are moving.       │")
    print("  └──────────────────────────────────────────────────────────────┘")
    print()
    print("  ── relay_v2.ino  (quickest fix: change baud + add LED + banner) ──")
    print(RELAY_V2)
    print()
    print("  ── relay_v3.ino  (buffered: fixes packet fragmentation issues) ───")
    print(RELAY_V3)
    print()
    print("  Half-duplex wiring reminder (required for HTD-35H bus servo):")
    print()
    print("    Arduino TX1 ──┬──[470 Ω]──┬── Servo Signal  (white/yellow)")
    print("    Arduino RX1 ──┘           │")
    print("    Arduino GND ──────────────┴── Servo GND      (black/brown)")
    print("    External 12 V ──────────── Servo VCC      (red)")
    print()
    print("  Without the resistor merge, TX1 and RX1 are isolated:")
    print("    TX1 drives the servo but RX1 cannot hear the servo's reply")
    print("    because TX1 is still holding the bus during the response window.")


# ─────────────────────────────────────────────────────────────────────────────
#  Summary
# ─────────────────────────────────────────────────────────────────────────────

def summary(res: Results, port: str):
    print()
    print("═" * 70)
    print("  FINAL SUMMARY")
    print("═" * 70)

    p1_hits  = res.hits("1")
    p2_hits  = res.hits("2")
    p3_hits  = res.hits("3")
    loopback = res.any_hit("0c")

    ctrl_ok = bool(p1_hits)
    bus_ok  = bool(p2_hits)

    print()
    if loopback:
        print("  ⚠  TX1/RX1 LOOPBACK DETECTED (0c)")
        print("     This means TX1 and RX1 are shorted together on the Arduino.")
        print("     Remove the short before connecting a servo or controller board.")
        print()

    if ctrl_ok:
        batt_hit = next(((k,n,r) for k,n,r in p1_hits if "batt" in k), None)
        pos_hit  = next(((k,n,r) for k,n,r in p1_hits if "pos"  in k), None)

        print("  ✅  RESULT: CONTROLLER BOARD PROTOCOL CONFIRMED  (Protocol A or B)")
        print()
        if batt_hit:
            print(f"     Battery: {batt_hit[1]}")
        if pos_hit and pos_hit[1]:
            print(f"     Position: {pos_hit[1]}")
            if "0-1000" in pos_hit[1] or "Protocol-A" in pos_hit[1]:
                variant = "A  (bus servo board, position range 0-1000)"
            elif "500-2500" in pos_hit[1] or "Protocol-B" in pos_hit[1]:
                variant = "B  (PWM servo board, position range 500-2500)"
            else:
                variant = "unknown subtype"
            print(f"     Variant : Protocol {variant}")
        print()
        print("     Packet  : 55 55  [Length]  [Cmd]  [Params...]")
        print("     Baud    : 9600  (relay.ino Serial1 9600 is correct)")
        print("     Length  : N_params + 2  (no servo ID, no checksum)")
        print()
        print("     Key commands:")
        print("       0x03  CMD_SERVO_MOVE           move servo(s) to position")
        print("       0x0F  CMD_GET_BATTERY_VOLTAGE  read battery mV")
        print("       0x15  CMD_MULT_SERVO_POS_READ  read servo position(s)")
        print("       0x14  CMD_MULT_SERVO_UNLOAD    limp mode (no torque)")
        print("       0x06  CMD_ACTION_GROUP_RUN     run stored action group")
        print("       0x07  CMD_ACTION_GROUP_STOP    stop action group")

    elif bus_ok:
        print("  ✅  RESULT: DIRECT BUS SERVO CONFIRMED  (Protocol C)")
        print("     (Surprising — Serial1 is 9600 but servo responded. Check wiring.)")

    else:
        # Diagnose based on what happened
        has_echo   = res.any_hit("3a_hdx")
        has_timing = res.any_hit("3b")
        has_noise  = res.any_hit("0a")

        print("  ❌  NO VALID RESPONSE DETECTED")
        print()
        print("  Most likely cause based on results:")
        print()

        if not loopback:
            print("  1. CANNOT CONFIRM Arduino relay is forwarding bytes")
            print("     → Run Phase 0c loopback test with TX1–RX1 shorted to verify")
            print("       PC→Arduino→Serial1→Arduino→PC path before connecting servo")
            print()

        print("  2. CONTROLLER BOARD (LBSC / LSC) — if that is your device:")
        print("     relay.ino Serial1=9600 is correct for these boards")
        print("     Check:")
        print("       a) Board has external 12V power (USB alone is not enough)")
        print("       b) Arduino TX1 → board RX pin  (not TX pin)")
        print("       c) Arduino RX1 ← board TX pin")
        print("       d) Common GND between Arduino and board")
        print("       e) Board is in UART/secondary-dev mode (check any jumpers)")
        print()

        print("  3. DIRECT BUS SERVO (HTD-35H) — if that is your device:")
        print("     ❌ relay.ino Serial1=9600 WILL NOT WORK")
        print("     HTD-35H requires 115200 baud on Serial1")
        print("     Fix: change relay.ino line 3:")
        print("       Serial1.begin(9600);   →   Serial1.begin(115200);")
        print("     AND use the resistor merge wiring:")
        print("       TX1 ──[470Ω]──┬── Servo Signal")
        print("       RX1 ──────────┘")
        print()

    # Hit table
    all_rows = res.all_rows()
    hits   = [(k,n,r) for k,h,n,r in all_rows if h]
    misses = [(k,n,r) for k,h,n,r in all_rows if not h and r]
    silent = [(k,n,r) for k,h,n,r in all_rows if not h and not r]
    print()
    print(f"  Hits   : {len(hits)}")
    for k, n, r in hits:
        print(f"    ✓ {k:<32}  {n[:45]}")
    print(f"  RX but invalid parse : {len(misses)}")
    for k, n, r in misses[:8]:
        print(f"    ? {k:<32}  rx={r.hex(' ')[:40]}")
    print(f"  Silent : {len(silent)}")


# ─────────────────────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="Hiwonder relay-aware probe (relay.ino)")
    parser.add_argument("port", nargs="?", default=DEFAULT_PORT)
    parser.add_argument("--phase", type=int, default=0,
                        help="Run only one phase (1-4). Default: run all.")
    parser.add_argument("--verbose", action="store_true",
                        help="Print byte-level timing details")
    args = parser.parse_args()

    print("=" * 70)
    print("  test.py  —  Hiwonder Probe  (relay.ino edition)")
    print(f"  Port : {args.port}   Baud : {ARDUINO_BAUD} (fixed by relay.ino)")
    print(f"  Time : {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 70)

    try:
        ser = serial.Serial(args.port, baudrate=ARDUINO_BAUD,
                            bytesize=8, parity="N", stopbits=1,
                            timeout=READ_WINDOW)
    except serial.SerialException as e:
        print(f"\n  ERROR: Cannot open {args.port}: {e}")
        print("  Available ports:")
        for p in serial.tools.list_ports.comports():
            print(f"    {p.device}: {p.description}")
        return

    print(f"  Port {args.port} opened at {ARDUINO_BAUD} baud.\n")

    res = Results()
    only = args.phase
    v    = args.verbose

    if only in (0, 0):   # always run phase 0
        phase0(ser, res, v)
    if only in (0, 1):
        phase1(ser, res, v)
    if only in (0, 2):
        phase2(ser, res, v)
    if only in (0, 3):
        phase3(ser, res, v)
    if only in (0, 4):
        phase4(v)         # phase4 is suggestions only, no serial needed

    if only != 4:
        summary(res, args.port)

    ser.close()
    print(f"\n  Done.  {datetime.now().strftime('%H:%M:%S')}\n")


if __name__ == "__main__":
    main()