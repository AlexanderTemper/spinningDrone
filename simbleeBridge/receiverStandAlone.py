#!/usr/bin/python
import pygatt
import time
import binascii


# Many devices, e.g. Fitbit, use random addressing - this is required to
# connect.
# sudo gatttool -t random -b C4:D0:0D:79:59:91 -I
# connect
# char-desc
# 2d30c083-f39f-4ce6-923f-3484ea480596 zum schreiben

ADDRESS_TYPE   = pygatt.BLEAddressType.random
DEVICE_ADDRESS = "C4:D0:0D:79:59:91"
adapter = pygatt.GATTToolBackend()

counter = 0;

def sendControlData(device):
    global counter
    counter = counter + 1
    if counter == 256:
        counter = 0
    try: 
        print('send %d' % counter)
        device.char_write_handle(0x0014, [counter]) 
        return True
    except pygatt.exceptions.NotConnectedError:
        print('error')
        return False;

def handle_data(handle, value):
    print("Received data: %s" % (value))
    


def connect():
    print("Try connecting to "+DEVICE_ADDRESS);
    try:
        device = adapter.connect(DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
        print("connected to "+DEVICE_ADDRESS)
        return device
    except pygatt.exceptions.NotConnectedError:
        return None;


try:
    adapter.start()
    
    device = connect()
    while not device:
        device = connect()
        
    device.subscribe("2d30c082-f39f-4ce6-923f-3484ea480596",callback=handle_data)
    
    while True:
        time.sleep(0.01)
        sendControlData(device)
        
except KeyboardInterrupt:
    print('kill signal')
 
finally:
    print('Adapter Stop')
    adapter.stop()
