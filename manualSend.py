import serial, time

ser = serial.Serial('COM5', baudrate=9600, timeout=5)

servo_id = 1
command = 28  # POS_READ
length = 3
checksum = (~(servo_id + length + command)) & 0xFF
packet = bytes([0x55, 0x55, servo_id, length, command, checksum])

print(f"Sending: {packet.hex()}")

ser.write(packet)



# Simple blocking read for response
response = ser.read(20)  # Read up to 20 bytes or until timeout
print(f"Received {len(response)} bytes: {response.hex()}")

ser.close()