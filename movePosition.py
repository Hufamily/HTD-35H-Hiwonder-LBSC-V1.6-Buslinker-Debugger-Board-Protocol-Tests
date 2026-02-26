import serial, time

ser = serial.Serial('COM5', baudrate=9600, timeout=5)

command  = 0x03              # CMD_SERVO_MOVE
servo_id = 1
position = 500               # range 0–1000 (~0° to 240°)
duration = 1000              # milliseconds

params = bytes([
    0x01,                    # number of servos
    duration & 0xFF,         # time low byte
    (duration >> 8) & 0xFF,  # time high byte
    servo_id,
    position & 0xFF,         # position low byte
    (position >> 8) & 0xFF,  # position high byte
])
length = len(params) + 2
packet = bytes([0x55, 0x55, length, command]) + params

print(f"Sending: {packet.hex()}")

ser.write(packet)
ser.close()