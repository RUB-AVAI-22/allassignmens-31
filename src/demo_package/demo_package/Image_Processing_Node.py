# BY Lars Buck
# 26.10.2022
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String, Int8, Bool
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np


class ImageSubscriber(Node):
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
        super().__init__('image_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Image, 'video_frames', self.listener_callback, 1)

        # subscriber to user input node
        self.user_input_subscription = self.create_subscription(String, "user_input", self.print_test_msg, 1)
        self.user_input_subscription_freq = self.create_subscription(Int8, "user_input_freq", self.set_freq, 1)
        self.user_input_subscription_im_rn = self.create_subscription(Bool, "user_input_im_rn", self.send_im_rn, 1)

        # publisher to user input node
        self.processed_image_publisher = self.create_publisher(Image, "processed_image", 1)
        self.timer = self.create_timer(1, self.process_and_publish)

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

    def process_and_publish(self):

        if self.mode == "normal":
            data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB))
            self.processed_image_publisher.publish(data)
            self.get_logger().info("publishing normal image data")

        elif self.mode == "grayscale":
            data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2GRAY))
            self.processed_image_publisher.publish(data)
            self.get_logger().info("publishing grayscale image data")

        elif self.mode == "contour":
            img = cv2.Canny(self.image_data, 150, 150)
            data = self.br.cv2_to_imgmsg(img)
            self.processed_image_publisher.publish(data)
            self.get_logger().info("publishing contour image data")

        else:
            data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB))
            self.processed_image_publisher.publish(data)
            self.get_logger().info("publishing normal image data")

    def print_test_msg(self, msg):
        self.get_logger().info('-> %s' % msg.data)
        self.mode = msg.data

    def send_im_rn(self, msg):
        if msg.data:
            if self.mode == "normal":
                data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB))
                self.processed_image_publisher.publish(data)
                self.get_logger().info("publishing normal image data immediately")

            elif self.mode == "grayscale":
                data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2GRAY))
                self.processed_image_publisher.publish(data)
                self.get_logger().info("publishing grayscale image data immediately")

            elif self.mode == "contour":
                img = cv2.Canny(self.image_data, 150, 150)
                data = self.br.cv2_to_imgmsg(img)
                self.processed_image_publisher.publish(data)
                self.get_logger().info("publishing contour image data immediately")

            else:
                data = self.br.cv2_to_imgmsg(cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB))
                self.processed_image_publisher.publish(data)
                self.get_logger().info("publishing normal image data immediately")

    def set_freq(self, msg):
        self.get_logger().info('setting timer freq to %d' % msg.data)
        self.timer.cancel()
        self.timer = self.create_timer(msg.data, self.process_and_publish)

        print(self.timer.timer_period_ns)

    def listener_callback(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        self.get_logger().info('Receiving video frame')

        # Convert ROS Image message to OpenCV image
        self.image_data = self.br.imgmsg_to_cv2(data)


def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    image_subscriber = ImageSubscriber()

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
