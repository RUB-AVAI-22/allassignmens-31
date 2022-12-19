## Autonomous Vehicles and Artificial Intelligence
# Assignment 5 Testing
Daniele Belmonte, Lars Alexander Paul Buck, Daniel Laurenz
Karl-Heinz Gerdes, Christof Hermeth, Liang Zhao

# Integration Tests
In the integration tests all of the functionalities of an entire ROS node are tested as a group, as the ROS nodes of a robotic system often have multiple interacting functions. These tests aim to verify that all functions can interact with each other properly. Our implementation of integration tests for the current Turtlebot with Object Dection is explained the following sections.

## Camera Node
For our current setup, the Camera node has only one single function that capture and publish image frames. Therefore an integration test for this node is not necessary, because its functionality was already verified in the previous unit tests.

## Image Processing Node
The image processing node subscribes to multiple inputs from the camera node and the user node, conducting image processing tasks (resize, inference for object detection), and publish the resulted annotated image as output.

A tester node was created to execute the integration test. The tester node has corresponding publishers that feeds the image processing node prescribed inputs. And it also has a subscriber that catches the output. Finally the output are validated by comparing to the ground truth.

## User Node
The user node receives the annotated images from the image processing node and displays them on the GUI. It also takes user input for setting parameters such as frame rate or color mode, and publish them to the image processing node.

Similar to the previous test, another tester node was created with matching publishers and subscribers. The main difference is that, in order to test the GUI, the script gives a text prompt for the human tester to make an certain input over the GUI, then the tester node catches the messages from the user node and check for their correctness. This approach is less efficient than simulating the user keyboard and mouse input, but is sufficient for our purpose given our simple GUI.