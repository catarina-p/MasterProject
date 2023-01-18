from harvest import Harvest
from threading import Thread
from time import time, sleep
import socket

localIP              = "10.15.20.195"
localPort            = 20001
bufferSize           = 1024

# Create a datagram socket
s = socket.socket(family=socket.AF_INET, type=socket.SOCK_DGRAM)
# Bind to address and ip
s.bind((localIP, localPort))
msg = 'null'

def read_socket():
    global msg
    while 1:
        bytesAddressPair = s.recvfrom(bufferSize)
        msg = bytesAddressPair[0].decode('utf-8')

t = Thread(target = read_socket)
t.daemon = True
t.start()
timer = time()
while(True):
    t = round(time() - timer, 3)
    print(msg, t)
    
    sleep(0.001)
