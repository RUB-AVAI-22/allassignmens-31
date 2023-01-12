# BY Lars Buck
# 26.10.2022
import rclpy  # Python library for ROS 2
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image  # Image is the message type
from std_msgs.msg import String, Int8, Bool
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from geometry_msgs.msg import PoseWithCovariance

class odometrysubcriber(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    def __init__(self):
        """
        Class constructor to set up the node
        """
        # Initiate the Node class's constructor and give it a name
        super().__init__('odom_subscriber')

        # Create the subscriber. This subscriber will receive an Image
        # from the video_frames topic. The queue size is 10 messages.
        self.subscription = self.create_subscription(Odometry, '/odom', self.listener_callback, qos_profile_sensor_data)



    def listener_callback(self, msg):
        """
        Callback function.
        """
        # check value
        """   msg->data     pose = PoseStamped()

        pose.header.frame_id = "main"
        pose.pose.position.x = float(data.pose.pose.position.x)
        pose.pose.position.y = float(data.pose.pose.position.y)
        pose.pose.position.z = float(data.pose.pose.position.z)
        pose.pose.orientation.x = float(data.pose.pose.orientation.x)
        pose.pose.orientation.y = float(data.pose.pose.orientation.y)
        pose.pose.orientation.z = float(data.pose.pose.orientation.z)
        pose.pose.orientation.w = float(data.pose.pose.orientation.w)
        """
        posex=float(msg.pose.pose.position.x)
        posey = float(msg.pose.pose.position.y)
        self.get_logger().info("datax %s"%posex)
        self.get_logger().info("datax %s"%posey)



def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create the node
    odom_subscriber = odometrysubcriber()

    # Spin the node so the callback function is called.
    rclpy.spin(odom_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    odom_subscriber.destroy_node()

    # Shutdown the ROS client library for Python
    rclpy.shutdown()


if __name__ == '__main__':
    main()
