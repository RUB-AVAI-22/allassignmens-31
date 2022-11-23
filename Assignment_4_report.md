## Autonomous Vehicles and Artificial Intelligence
# Assignment 4 Bounding Box Detection for the Turtlebot
Daniele Belmonte, Lars Alexander Paul Buck, Daniel Laurenz
Karl-Heinz Gerdes, Christof Hermeth, Liang Zhao

## 1. Introduction
The Turtlebot need to be able to recognize the objects of its surroundings. For the autonomous driving task, the objects of interest would be paper cones of 3 different colors: blue, yellow, and orange.

## 2. Design Decisions
YOLOv5 was chosen for the object detection task.

## 3. Dataset
The dataset consist of ___ images taken by the Turtlebot camera. 

The images are labeled with bounding boxes information

## 4. Training


## 5. Deployment on Turtlebot
The Coral USB Accelerator was connected to the Turtlebot to provide the processing power needed for running the neural network model.

The YOLOv5 model was originally trained with `pytorch` and then ported to `TensorFlow Lite` as `pytorch` was not yet supported by the Coral USB Accelerator.



## 6. Results
Some images
Processing time
More test cases?