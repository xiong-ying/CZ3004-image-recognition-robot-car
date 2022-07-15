from communicator.config import *
import time
import datetime
import sys

def move_forward1(t):
    forward1 = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x64,0x00,0x00,0x00,0x00,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        #print("Time to terminate the code",time_out)
        time.sleep(0.1)
        ser.write(forward1)
        
def move_forward(t):
    forward = bytearray([0x5a,0x0c,0x01,0x01,0x00,0xc8,0x00,0x00,0x00,0x00,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(forward)
        
def move_backward1(t):
    backward = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x9c,0x00,0x00,0x00,0x00,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(backward)
        
def move_backward(t):
    backward = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x38,0x00,0x00,0x00,0x00,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(backward)

#DEFAULT 15 degree per rotate , 100 mm/sec
def move_forwardLeft(t):
    forwardLeft = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x64,0x00,0x00,0x03,0xe8,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(forwardLeft)

def move_backwardLeft(t):
    backwardLeft = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x9c,0x00,0x00,0xfc,0x18,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(backwardLeft)

def move_forwardRight(t):
    forwardRight = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x64,0x00,0x00,0xfc,0x18,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(forwardRight)
        
def move_backwardRight(t):
    backwardRight = bytearray([0x5a,0x0c,0x01,0x01,0xff,0x9c,0x00,0x00,0x03,0xe8,0x00,0xff])
    time_out = datetime.datetime.now() + datetime.timedelta(0,t)
    while datetime.datetime.now() < time_out:
        time.sleep(0.1)
        ser.write(backwardRight)

def mover_stop():
    stop = bytearray([0x5a,0x0c,0x01,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff])
    ser.write(stop)





