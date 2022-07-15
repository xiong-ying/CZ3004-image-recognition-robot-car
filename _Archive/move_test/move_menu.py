import serial
import time


choice = ''

#start a loop that runs until the user enters q to quit
while choice != 'q':
    
    ser = serial.Serial('/dev/ttyS0',115200)
    counter = 0
    
    #cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF])
    #ser.write(cmd)
    #time.sleep(1)
    
    print("\n[1] Enter 1 for move forward.")
    print("[2] Enter 2 for move backward.")
    print("[3] Enter 3 for move left.")
    print("[4] Enter 4 for move right.")
    print("[q] Enter q to quit.")
    
    choice = input();
    
    if choice == '1':
        #print("Enter the forward distance: ")
        entry = int(input("Please enter the forward distance in cm: "))
        if entry <= 90:
            entry += 21
            
        elif entry <= 120:
            entry += 17
        
        else:
            entry += 15
            
        start = time.time()
        
        while counter < entry:
            cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x00, 0xe1, 0x00, 0x00, 0x00, 0x23, 0x00, 0xFF])
            ser.write(cmd)
            counter += 1
            #print("Run", counter)
            time.sleep(0.05)
            
        end = time.time()
        print("Total time =", end - start)
        
    elif choice == '2':
        #print("\nEnter the backward distance: \n")
        entry = int(input("Please enter the backward distance in cm: "))
        if entry <= 90:
            entry += 12
            
        elif entry <=110:
            entry += 4
            
        elif entry <=120:
            entry += 2
                      
        else:
            entry += 1
            
        start = time.time()
        
        while counter < entry:
            cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0xff, 0x01, 0x00, 0x00, 0xff, 0xdd, 0x00, 0xFF])
            ser.write(cmd)
            counter += 1
            print("Run", counter)
            time.sleep(0.05)
            
        end = time.time()
        print("Total time =", end - start)
        
    elif choice == '3':
        #print("\nEnter the right angle: \n")
        entry = int(input("Please enter the right angle: "))
        
        if entry == 90:
            entry -= 20
            
        elif entry == 180:
            entry -= 65
            
        elif entry == 270:
            entry -= 110
            
        elif entry == 360:
            entry -= 160
            
        start = time.time()
        
        while counter < entry:
            cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x00, 0xe1, 0x00, 0x00, 0xfc, 0xe0, 0x00, 0xFF])
            ser.write(cmd)
            counter += 1
            print("Run", counter)
            time.sleep(0.05)
            
        end = time.time()
        print("Total time =", end - start)
        
    elif choice == '4':
        #print("\nEnter the left angle: \n")
        entry = int(input("Please enter the left angle: "))
        if entry == 90:
            entry -= 5
            
        elif entry == 180:
            entry -= 40
            
        elif entry == 270:
            entry -= 70
            
        elif entry == 360:
            entry -= 104
            
        #entry += 20
        start = time.time()
        
        while counter < entry:
            cmd = bytearray([0x5a, 0x0c, 0x01, 0x01, 0x00, 0xe1, 0x00, 0x00, 0x03, 0x20, 0x00, 0xFF])
            ser.write(cmd)
            counter += 1
            print("Run", counter)
            time.sleep(0.05)
            
        end = time.time()
        print("Total time =", end - start)
        
    elif choice == 'q':
        print("\nBye!")
        
    else:
        print("\nPlease try again\n")
        
print("End of program")