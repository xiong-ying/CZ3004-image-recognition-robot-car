#from config import *
from communicator.config import *
import time


def move_forward(n):
    forward = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x64,0x00,0x00,0x00,0x00,0x00,0xff])
    count =0
    while(count < n):
        ser.write(forward)
        time.sleep(0.5)
        count = count +1
#    ser.write(forward)

#DEFAULT 15 degree per rotate , 100 mm/sec
def move_forwardLeft(n):
    forwardLeft = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x64,0x00,0x00,0x03,0xe8,0x00,0xff])
    #ser.write(forwardLeft)
    count =0
    while(count < n):
        ser.write(forwardLeft)
        time.sleep(0.5)
        count = count +1

def move_forwardRight(n):
    forwardRight = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x64,0x00,0x00,0xfc,0x18,0x00,0xff])
    #ser.write(forwardRight)
    count =0
    while(count < n):
        ser.write(forwardRight)
        time.sleep(0.5)
        count = count +1

def move_backward(n):
    backward = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x9c,0x00,0x00,0x00,0x00,0x00,0xff])
#    ser.write(backward)
    count =0
    while(count < n):
        ser.write(backward)
        time.sleep(0.5)
        count = count +1

def move_backwardLeft(n):
    backwardLeft = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x9c,0x00,0x00,0x03,0xe8,0x00,0xff])
    #ser.write(backwardLeft)
    count =0
    while(count < n):
        ser.write(backwardLeft)
        time.sleep(0.5)
        count = count +1

def move_backwardRight(n):
    backwardRight = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x9c,0x00,0x00,0xfc,0x18,0x00,0xff])
    #ser.write(backwardRight)
    
    count =0
    while(count < n):
        ser.write(backwardRight)
        time.sleep(0.5)
        count = count +1

def mover_stop():
    stop = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff])
    ser.write(stop)



#move_backward()
#move_forwardLeft()
#move_forwardRight()
#time.sleep(1)

#ser.close()

