# CZ3004 Multidisciplinary Design Project

# Project Title
Image Recognition Robotic Car

# Project Purpose
The project is created for NTU computer science module CZ3004.


## Background Info

On a 2 meters by 2 meters map, 5 obstacles are randomly placed on the map.

There is 1 distinct symbol pasted on one side of each obstacle.


## The ultimate objective

1. Send the obstacles information from controller app on Android tablet to robotic car.
2. The car automatically locates the 5 obstacles, find the shortest Hamiltonian path to traverse to all obstacles.
3. When the car reaches each obstacles, activates the pi-camera to scan the image and detect the symbol.
4. After detection is done, the car send back the image id to the controller app on Android tablet.


## Table of contents

This repo only consists of 2 components:

### 1. algo:
Algorithm to plan a path to traverse all 5 obstacles placed on the map.

How to use:
Refer to "main.py" for example of usage


### 2. image:
Image recognition to detect the symbol pasted on the obstacles.
Machine Learning Model - TensorFlow Lite
Computer Vision Tool - OpenCV

How to use:
On Rpi, install tflite_runtime, opencv. Then, refer to "main.py" for example of usage
