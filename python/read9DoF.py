import posix_ipc

mq = posix_ipc.MessageQueue("/9DoF", posix_ipc.O_CREAT)
while True:
    msg = mq.receive()
    print(msg)
