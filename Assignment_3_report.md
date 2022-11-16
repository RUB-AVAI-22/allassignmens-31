## Autonomous Vehicles and Artificial Intelligence
# Assignment 3 Remote Control of Turtlebot3
Daniele Belmonte, Lars Alexander Paul Buck, Daniel Laurenz
Karl-Heinz Gerdes, Christof Hermeth, Liang Zhao

## 1. Introduction
We have some good reason to move the robot around.

## 2. Design Decisions
We created a publisher note to send `Twist` type messages under the topic `cmd_vel`. 

The built-in bringup code `robot.launch.py` creates a subscriber to the `cmd_vel` topic, which receives the `Twist` type messages.

The `Twist` type messages contains 6 velocity values:
- linear velocities in x, y, z directions: `linear.x, linear.y, linear.z`
- angular velocities in x, y, z directions: `angular.x, angular.y, angular.z`

Due to the physical layout of the robot, only `linear.x` and `angular.z` are used to control the motion of the robot.

## 3. Controls
To develop an RC car feeling, the TurtleBot moves at a constant speed in the desired direction when the linear direction keys `w`, `x` are pressed once. This makes it possible to steer while driving, using the steering keys `a`, `d`. With the `spacebar` or `s`  key, the Turtlebot stops.

We have also tried to implement a speed control, which allows to increase or slow down the speed, but this had some issues which we could not solve in time.

## 4. Results
The control works.

The truninig radius is related to the current linear speed. 

Stuttering behavior at low speed.

