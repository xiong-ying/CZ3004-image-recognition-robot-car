import serial
import time

while True:
    ser = serial.Serial('/dev/ttyS0',115200)

    
    counter = 0

    entry = int(input("Please enter the distance in cm: "))
    
    entry += 20;
    
    start = time.time()

    while counter < entry:
    
        cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x00, 0xe1, 0x00, 0x00, 0xfc, 0xe0, 0x00, 0xFF])
        ser.write(cmd)
        counter += 1
        print("Run", counter)
        time.sleep(0.05)

    end = time.time()
    print("Total time =", end - start)