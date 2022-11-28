# BY Lars Buck
# 26.10.2022
import rclpy  # Python library for ROS 2
import os

import torchvision.utils
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String, Int8, Bool
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import torch


class Ai_Node(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    image_data = np.ones(shape=[480, 640, 3], dtype=np.uint8)
    mode = "normal"

    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('AINode')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.

        # subscriber to user input node
        self.image_subscription = self.create_subscription(Image, "video_frames", self.detect_objects, 1)
        self.processed_image_publisher = self.create_publisher(Image, "processed_image", 1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.model = torch.hub.load("ultralytics/yolov5", 'custom', path = 'src/demo_package/resource/best-int8.tflite')

    def detect_objects(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        #self.get_logger().info('Receiving processed frame')

        # Convert ROS Image message to OpenCV image
        self.image_data = self.br.imgmsg_to_cv2(data)
        #self.image_data = cv2.resize(self.image_data, (480, 480), interpolation=cv2.INTER_AREA)
        results = self.model(self.image_data)
        data = results.pandas().xyxy[0]

        for index, border in data.iterrows():
            start = (int(border.xmin), int(border.ymin))
            end = (int(border.xmax), int(border.ymax))
            color = (255, 0, 0)

            cv2.rectangle(self.image_data, start, end, color, 2)

        results.print()
        data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB))
        self.processed_image_publisher.publish(data)
        self.get_logger().info("publishing image with bounding boxes")




def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    print(os.getcwd())
    # Create the node
    image_subscriber = Ai_Node()

    # Spin the node so the callback function is called.
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()