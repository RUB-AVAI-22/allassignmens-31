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

#launch feature Node

def generate_test_description():
    file_path = os.path.dirname(os.path.dirname(__file__))
    Image_Processing_Node = launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "demo_pkg", 'Image_Processing_Node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )

    return launch.LaunchDescription([
        Image_Processing_Node,
        launch_testing.actions.ReadyToTest()
    ],{"Image_Processing_Node":Image_Processing_Node}
    )

class TestImageProcessingNode(unittest.TestCase):

    def setUpClass(cls) -> None:
        rclpy.init(args=None)

    def tearDownClass(self) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_image_processing_node")

    def tearDown(self) -> None:
        self.node.destroy_node()

    def test_camera_image_processing_connection(self):

        rcv_imgs = []

        pub = self.node.create_publisher(Image, "video_frames", 10)
        sub = self.node.create_subscription(Image, "video_frames", lambda msg: rcv_imgs.append(msg), 10)

        try:
            img = Image()
            cap = cv2.VideoCapture(0)
            ret, frame = cap.read()
            img = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            br = CvBridge()
            img = br.cv2_to_imgmsg(img, encoding="bgr8")
            pub.publish(img)
            self.node.get_logger().info("publishing image data")
            # Are more than 2 pictures published after 10 seconds?
            end_time = time.time() + 10.0

            while time.time() < end_time:
                rclpy.spin_once(self.node)
                if len(rcv_imgs) > 2:
                    break

            self.assertGreater(len(rcv_imgs), 2)
            # assert if images are not None and assert that image width is 640 and height is 480
            for img in rcv_imgs:
                self.assertIsNotNone(img, "Image is None")
                self.assertEqual(img.width, 640, "Image width is not 640")
                self.assertEqual(img.height, 480, "Image height is not 480")
        finally:
            pub.destroy()
            sub.destroy()







