# import detect.py for image recognition

from detect import *


# Call function detectImg():
# Turn on the pi camera and detecte images
# If found image, return image id, turn off the camera
# If not found, keep searching

imageId = detectImg()


# Display the image ID in console line

print("This photo id is ", imageId)
