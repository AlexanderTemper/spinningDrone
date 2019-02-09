#!/usr/bin/python
import pygatt
import time
import socket


# Many devices, e.g. Fitbit, use random addressing - this is required to
# connect.
# sudo gatttool -t random -b C4:D0:0D:79:59:91 -I
# connect
# char-desc
# 2d30c083-f39f-4ce6-923f-3484ea480596 zum schreiben

HOST = '127.0.0.1'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)

ADDRESS_TYPE   = pygatt.BLEAddressType.random
DEVICE_ADDRESS = "C4:D0:0D:79:59:91"
adapter = pygatt.GATTToolBackend()


lastread = 0
inBuffer = [0 for i in range(1000)]
nextBuffer = 0

def twos_comp(val):
    """compute the 2's complement of int value val"""
    if (val & 0x80000000): # if sign bit is set e.g., 8bit: 128-255
        val = val - (0x100000000)        # compute negative value
    return val 


def tolong(a,b,c,d):
    value = a & 0xFF;
    value |= (b << 8) & 0xFFFF;
    value |= (c << 16) & 0xFFFFFF;
    value |= (d << 24) & 0xFFFFFFFF;
    return twos_comp(value)

def sendData():
    if nextBuffer == 13:
        if inBuffer[0] == 79:#O
            r = tolong(inBuffer[1],inBuffer[2],inBuffer[3],inBuffer[4])
            p = tolong(inBuffer[5],inBuffer[6],inBuffer[7],inBuffer[8])
            y = tolong(inBuffer[9],inBuffer[10],inBuffer[11],inBuffer[12])
            send = "Att %d %d %d \n" % (r,p,y)
            server.sendall(send)
        elif inBuffer[0] == 71:#G
            x = tolong(inBuffer[1],inBuffer[2],inBuffer[3],inBuffer[4])
            y = tolong(inBuffer[5],inBuffer[6],inBuffer[7],inBuffer[8])
            z = tolong(inBuffer[9],inBuffer[10],inBuffer[11],inBuffer[12])
            send = "Gyro %d %d %d \n" % (x,y,z)
            server.sendall(send)
        elif inBuffer[0] == 82:#R
            x = tolong(inBuffer[1],inBuffer[2],inBuffer[3],inBuffer[4])
            y = tolong(inBuffer[5],inBuffer[6],inBuffer[7],inBuffer[8])
            z = tolong(inBuffer[9],inBuffer[10],inBuffer[11],inBuffer[12])
            send = "Acc %d %d %d \n" % (x,y,z)
            server.sendall(send)
        elif inBuffer[0] == 77:#M
            x = tolong(inBuffer[1],inBuffer[2],inBuffer[3],inBuffer[4])
            y = tolong(inBuffer[5],inBuffer[6],inBuffer[7],inBuffer[8])
            z = tolong(inBuffer[9],inBuffer[10],inBuffer[11],inBuffer[12])
            send = "Mag %d %d %d \n" % (x,y,z)
            server.sendall(send)
        elif inBuffer[0] == 84:#T
            x = tolong(inBuffer[1],inBuffer[2],inBuffer[3],inBuffer[4])
            y = tolong(inBuffer[5],inBuffer[6],inBuffer[7],inBuffer[8])
            z = tolong(inBuffer[9],inBuffer[10],inBuffer[11],inBuffer[12])
            send = "Tof %d %d %d \n" % (x,y,z)
            server.sendall(send)
    elif inBuffer[0] == 68:#D
        send = "Debug%s \r\n" % ''.join(chr(inBuffer[i]) for i in range(0,nextBuffer))
        server.sendall(send)
    
    

def handle_data(handle, value):
    global lastread,inBuffer,nextBuffer
    for i in range (0, len(value)):
        r = value[i]
            
        if lastread == 65 and r == 66: # 65 = A , 66 = B
            sendData()
            nextBuffer = 0;
        elif lastread == 65:
            inBuffer[nextBuffer] = lastread
            nextBuffer += 1
            inBuffer[nextBuffer] = r
            nextBuffer += 1
        elif r != 65:
            inBuffer[nextBuffer] = r
            nextBuffer += 1
        lastread = r


def connect():
    print("Try connecting to "+DEVICE_ADDRESS);
    try:
        device = adapter.connect(DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
        print("connected to "+DEVICE_ADDRESS)
        return device
    except pygatt.exceptions.NotConnectedError:
        return None;


try:
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.connect((HOST, PORT))
    adapter.start()
    
    device = connect()
    while not device:
        device = connect()
        
    device.subscribe("2d30c082-f39f-4ce6-923f-3484ea480596",callback=handle_data)
    
    while True:
        time.sleep(100)
        
except KeyboardInterrupt:
    print('kill signal')
 
finally:
    print('sinding end to Server')
    server.sendall("end\n")
    print('Adapter Stop')
    adapter.stop()
