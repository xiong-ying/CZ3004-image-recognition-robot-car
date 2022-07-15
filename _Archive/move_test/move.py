
#!/usr/bin/python3
import serial
import time

ser = serial.Serial('/dev/ttyS0',115200)

    #cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])
    #ser.write(cmd)

    #time.sleep(1)

def distance_counter():
   # print ("Please enter the counter:")
    entry = int(input("Please enter the counter: "))
    #return entry
    counter = 0
    start = time.time()

    while counter < entry:
        
        cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x0, 0xe1, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])
        ser.write(cmd)
    
    
        counter += 1
    
        print("Run", counter)
    
        time.sleep(0.05)
    
        end = time.time()

        print(end - start)
    