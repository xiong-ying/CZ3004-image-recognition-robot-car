# import detect.py for image recognition

from detect import *


# function detectImg(): return detected image id

# Turn on the pi camera and start detecting images
# If found image: return image id, turn off the camera
# while image not found, or found bulleye: keep searching

imageId = detectImg()


# Display the image ID in console line

print("This photo id is ", imageId)
