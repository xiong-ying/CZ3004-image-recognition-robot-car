#Test string = 5,9,3|15,15,3|7,14,2|15,4,2|12.9,0|
#0=East, 1=North, 2=West, 3=South
#data = '5,9,3|15,15,3|7,14,2|15,4,2|12,9,0|'

from bluetooth import *
from motor import *
from detect import*
from path_finder import*
import time

server_sock = BluetoothSocket(RFCOMM)
server_sock.bind(("", PORT_ANY))
server_sock.listen(1)
port = server_sock.getsockname()[1]
uuid = "94f39d29-7d6d-437d-973b-fba39e49d4ee"

#obstacle = 0
#Forward 10cm
def forward1():
    move_forward1(2.41)
#Forward 20cm
def forward2():
    move_forward(2.4)
#Backward 10cm
def backward1():
    move_backward1(2.41)
#Backward 20cm
def backward2():
    move_backward(2.4)

def leftTurn90():
    move_backwardRight(3)
    move_forwardLeft(2)
    move_backward1(0.7)
    move_forwardLeft(1)
    move_backwardRight(1) #1.3 -> 1.1

def rightTurn90():
    move_backwardLeft(3)
    move_forwardRight(2.2)
    move_backward1(0.7)
    move_forwardRight(1)
    move_backwardLeft(1.25)
    #move_forward1(0.5)
    move_backward1(0.25)

def imageProcess():
    #obstacle += 1
    imageID = detectImg()
    client_sock.send("IMG-%s-%s" % (targetId, imageID))
    print("IMG-%s-%s" % (targetId, imageID))
    #imageID = detectImg()
    #print("TARGET,", obstacle, ",", imageID,)

def send(targetId, imgId):
    client_sock.send("IMG-%s-%s" % (targetId, imgId))
    print("IMG-%s-%s" % (targetId, imgId))

advertise_service(
    server_sock, "MDP-Server",
    service_id=uuid,
    service_classes=[uuid, SERIAL_PORT_CLASS],
    profiles=[SERIAL_PORT_PROFILE]
    # protocols = [ OBEX_UUID ]
)

print("Waiting for connection on RFCOMM channel %d" % port)

client_sock, client_info = server_sock.accept()
print("Accepted connection from ", client_info)

time.sleep(2)
send("Bluetooth", "Connected")
substr = []
substr_tuple = []
start_time = time.time()

try:
    while True:
        print("In while loop...")
        data = client_sock.recv(1024)
        # if len(data) == 0: break
        print("Received [%s]" % data)
        convert = data.decode('utf-8')

        if convert.find("|") != -1:
            start_time = time.time()
            substr = convert.split("|") #convert to a list

        #print("Orignal string:", substr)
        #remove the last element. ele is to store the popped element
            ele = substr.pop()
        #print("Updated string:", substr)
        #print(type(substr))
        #print("")

        #convert list of strings to list to tuples, https://www.geeksforgeeks.org/python-convert-list-of-strings-to-list-of-tuples/
        #to be passed to algo
            substr_tuple = list(map(eval, substr))
            print("List sent out:")
            print(substr_tuple)
            print(type(substr_tuple))
            movement, obstacles_seq = planPath(substr_tuple)
            print("testing:", movement, obstacles_seq)

            length = len(movement)
            print("The total command set is", length)

            length2 = len(obstacles_seq)
            print("The total obstacles is", length)
            print("Testing", obstacles_seq)

            n = 0
            while n < length:
                print("Command", n, movement[n])
                string = movement[n]

                i = 0
                while i < len(string):
                    move = string[i]
                    print(i, move)

                    if(move == "tr"):
                        rightTurn90()
                        time.sleep(1)
                        forward1()

                    elif(move == "fwd"): # if instruciton is "fwd"

                        try: # try if index i+1 is in range

                            if string[i+1] == "fwd": # if the next instruction is also "fwd"
                                forward2() # move 20 cm
                                i += 1 # skip the next iteration

                            else: # if next instruction is NOT "fwd"
                                forward1() # move 10 cm

                        except: # if index i+1 out of range, means it's the last instruction
                            forward1() # move 10 cm

                    elif(move == "tl"):
                        leftTurn90()
                        time.sleep(1)
                        forward1()
                    elif(move == "rev"):
                        backward1()

                    time.sleep(1)
                    i += 1

                print("Activate camera")
                imageID, valid = detectImg()

                # if image ID is 22, means scan nothing
                if imageID == 22:

                    # Initialize a list of function to move back 1 step, forward 2 steps, back 1 step
                    back_and_forth = [backward1(), forward2(), backward1()]

                    for movement in back_and_forth:
                        # do 1 movement at a time
                        movement

                        # if previous detect is successful, don't scan again, just finish the movement
                        if imageID != 22:
                            continue

                        # scan image again
                        print("Activate camera")
                        imageID, valid = detectImg()


                send(obstacles_seq[n], imageID)

                #imageProcess()
                #time.sleep(2)
                n += 1

    print("--- %s seconds ---" % (time.time() - start_time))

except IOError:
    pass

print("disconnected")
client_sock.close()
server_sock.close()
print("all done")
