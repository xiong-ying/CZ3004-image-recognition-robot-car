from bullseyeTurn import*
import time

def bullseye():
    
    print('bullseyes, turn')
    print('forward left')
    move_forwardLeft(16)
    time.sleep(0.1)
    
    print('forward right')
    move_forwardRight(8)
    time.sleep(0.1)
    
    print('backward right')
    move_backwardRight(6) #old value is 11
    time.sleep(0.1)
    
    print('forward right')
    move_forwardRight(6)
    time.sleep(0.1)
    
    print('backward')
    move_backward(1)
    
bullseye()
