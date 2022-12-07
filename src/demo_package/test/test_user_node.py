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
    User_Node= launch_ros.actions.Node(
        executable=sys.executable,
        arguments=[os.path.join(file_path, "..", "demo_pkg", 'User_Node.py')],
        additional_env={'PYTHONUNBUFFERED': '1'},
        parameters=[{
        }]
    )

    return launch.LaunchDescription([
        User_Node,
        launch_testing.actions.ReadyToTest()
    ],{"User_Node":User_Node}
    )
class TestUserNode(unittest.TestCase):

    def setUpClass(cls) -> None:
        rclpy.init(args=None)

    def tearDownClass(self) -> None:
        rclpy.shutdown()

    def setUp(self) -> None:
        self.node = rclpy.create_node("test_user_node")

    def tearDown(self) -> None:
        self.node.destroy_node()

    # Testing the GUI will need to be done manually
    # We can test the connection between UserNode and ImageProcessingNode

    def test_camera_image_processing_connection(self):

        rcv_proc_imgs = []

        # opencv webcam image to ros image
        cap = cv2.VideoCapture(0)
        ret, frame = cap.read()
        bridge = CvBridge()
        ros_img = bridge.cv2_to_imgmsg(frame, "bgr8")

        pub = self.node.create_publisher(Image, "processed_image", 10)
        sub = self.node.create_subscription(Image, "processed_image", lambda msg: rcv_proc_imgs.append(msg), 10)

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
                if len(rcv_proc_imgs) > 2:
                    break

            self.assertGreater(len(rcv_proc_imgs), 2)
            # assert if images are not None and assert that image width is 640 and height is 480
            for img in rcv_proc_imgs:
                self.assertIsNotNone(img, "Image is None")
                self.assertEqual(img.width, 640, "Image width is not 640")
                self.assertEqual(img.height, 480, "Image height is not 480")
        finally:
            pub.destroy()
            sub.destroy()