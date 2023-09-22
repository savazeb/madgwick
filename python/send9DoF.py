import serial
import posix_ipc

mq = posix_ipc.MessageQueue("/9DoF", posix_ipc.O_CREAT)

with serial.Serial('/dev/ttyACM0', 115200, timeout=1) as s:
    while True:
        line = s.readline()
        try:
            feed = line.decode().rstrip()
            ax, ay, az, gx, gy, gz, mx, my, mz = feed.split(",")

            mq.send(feed.encode())
        except Exception as e:
            print(e)
