import rclpy  # Python library for ROS 2
import os

from rclpy.node import Node  # Handles the creation of nodes

from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan  # Image is the message type
from std_msgs.msg import String, Int8, Bool
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import matplotlib.pyplot as plt


class TestLidar(Node):
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
        self.lidar_subscription = self.create_subscription(LaserScan, "/scan", self.lidar, qos_profile_sensor_data)
        self.image_subscription = self.create_subscription(Image, "video_frames", self.images, 1)
        self.processed_image_publisher = self.create_publisher(Image, "processed_image", 1)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()


    def lidar(self, data: LaserScan):
        """
        Callback function.
        """


        fig, ax = plt.subplots()
        ax.imshow(self.image_data)
        y = data.ranges[180-31:180+31:-1]
        y = y / np.max(y) * 480
        x = np.linspace(0, 640, len(y))
        ax.plot(x, y)
        fig.savefig("lidar.png")
        fig.clf()

        fig.canvas.draw()

        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep="")
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        data = self.br.cv2_to_imgmsg(img)
        self.processed_image_publisher.publish(data)


    def images(self, data):
        self.image_data = cv2.cvtColor(self.br.imgmsg_to_cv2(data), cv2.COLOR_BGR2RGB)



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)
    print(os.getcwd())
    # Create the node
    image_subscriber = TestLidar()

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