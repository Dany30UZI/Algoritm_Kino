import serial
import time

# Adjust 'COM3' to your Arduino's serial port (e.g., '/dev/ttyUSB0' on Linux)
arduino = serial.Serial('COM3', 9600, timeout=1)
time.sleep(2)  # Give Arduino time to reset

# Send data to Arduino
arduino.write(b'Hello Arduino\n')

# Read response from Arduino
response = arduino.readline().decode('utf-8').strip()
print(response)
