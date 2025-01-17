# Image Recognition Robotic Car

CZ3004 Multidisciplinary Design Project<br/>
2022.07 @ Nanyang Technological University<br/>
The project is created for NTU computer science module CZ3004.

## Project Purpose

On a 2 meters by 2 meters map, 5 obstacles are randomly placed on the map.<br/>

There is 1 distinct symbol pasted on one side of each obstacle.<br/>

Goal is to develop a robotic car that can find the shortest path to scan the sign on the obstacle, <br/>

and accurately recognise the symbol.

Presentation Video Link: [YouTube Video Link](https://www.youtube.com/watch?v=k-RgcFn8MNQ)

## The ultimate objective

1. Send the obstacles information from controller app on Android tablet to robotic car.<br/>
2. The car automatically locates the 5 obstacles, find the shortest Hamiltonian path to traverse to all obstacles.<br/>
3. When the car reaches each obstacles, activates the pi-camera to scan the image and detect the symbol.<br/>
4. After detection is done, the car send back the image id to the controller app on Android tablet.


## Table of contents

This repo only consists of 2 components:

### 1. algo:
Algorithm to plan a path to traverse all 5 obstacles placed on the map.<br/>

How to use: <br/>
Refer to "main.py" for example of usage


### 2. image:
Image recognition to detect the symbol pasted on the obstacles.<br/>
Machine Learning Model - TensorFlow Lite<br/>
Computer Vision Tool - OpenCV<br/>

How to use:<br/>
On Rpi, install tflite_runtime, opencv. Then, refer to "main.py" for example of usage
