import sys
import unittest
from datetime import time

import cv2
import launch
import launch_ros.actions
import launch_testing.actions
import rclpy
import os

from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import std_msgs.msg


def generate_test_description():
    file_path = os.path.dirname(os.path.dirname(__file__))
    Camera_Node= launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "demo_pkg", 'Camera_Node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )

    return launch.LaunchDescription([
        Camera_Node,
        launch_testing.actions.ReadyToTest()
    ],{"Camera_Node":Camera_Node}
    )

class TestCameraNode(unittest.TestCase):

    def setUpClass(cls) -> None:
        rclpy.init(args=None)

    def tearDownClass(cls) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_camera_node")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_image_recording(self):

        # get webcam image opencv
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()

        # test if image is recorded
        self.assertTrue(ret, "Image is not recorded")
        # test if image is not empty
        self.assertFalse(frame is None, "Image is empty")
        # test if image is not black
        self.assertFalse(frame.all() == 0, "Image is black")
        # test if image is not white
        self.assertFalse(frame.all() == 255, "Image is white")
        # test image width and height 640x480
        self.assertEqual(frame.shape[0], 480, "Image height is not 480")
        self.assertEqual(frame.shape[1], 640, "Image width is not 640")

        # test if image is changing
        ret, frame = cap.read()
        # wait 2 seconds
        time.sleep(2)
        ret, frame2 = cap.read()
        self.assertFalse(frame.all() == frame2.all(), "Image is not changing (exactly the same)")

