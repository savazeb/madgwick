import serial

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as s:
    while True:
        line = s.readline()
        print(line)
