# BY Lars Buck
# 26.10.2022
import rclpy  # Python library for ROS 2
import os

import torchvision.utils
from rclpy.node import Node  # Handles the creation of nodes
from sensor_msgs.msg import Image, LaserScan  # Image is the message type
from std_msgs.msg import String, Int8, Bool
from rclpy.qos import qos_profile_sensor_data
from cv_bridge import CvBridge  # Package to convert between ROS and OpenCV Images
import cv2  # OpenCV library
import numpy as np
import torch
import matplotlib.pyplot as plt

class Ai_Node(Node):
    """
    Create an ImageSubscriber class, which is a subclass of the Node class.
    """
    image_data = np.ones(shape=[480, 640, 3], dtype=np.uint8)
    lidar_range = np.zeros(360, dtype=np.float)
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
        self.laser_subscription = self.create_subscription(
            LaserScan, # msg type
            '/scan', # topic name
            self.updateLidarData, # callback function
            qos_profile_sensor_data, # qos profile
        )

        # Used to convert between ROS and OpenCV images
        self.br = CvBridge()

        self.model = torch.hub.load("ultralytics/yolov5", 'custom', path = 'src/demo_package/resource/best.pt')
        
    def updateLidarData(self, laserMsg):
        self.lidar_range = np.array(laserMsg.ranges)

    def detect_objects(self, data):
        """
        Callback function.
        """
        # Display the message on the console
        #self.get_logger().info('Receiving processed frame')

        # Convert ROS Image message to OpenCV image
        self.image_data = self.br.imgmsg_to_cv2(data)
        self.image_data = cv2.cvtColor(self.image_data, cv2.COLOR_BGR2RGB)
        #self.image_data = cv2.resize(self.image_data, (480, 480), interpolation=cv2.INTER_AREA)
        results = self.model(self.image_data)

        #draw tensorflow lite bounding boxes
        labels = ["blue","orange","yellow"]
        results.render()
        #assign category to a label



        data = results.pandas().xyxy[0]
        y = np.flip(self.lidar_range[180 - 31: 180 + 31])
        y[y == 0] = np.nan
        x = np.linspace(0, 640, y.shape[0])
        
        cones = []
        for index, cone in data.iterrows():
            index_min = y.shape[0] / 640 * cone["xmin"]
            index_max = y.shape[0] / 640 * cone["xmax"]
            
            angle_min = index_min - y.shape[0]/2
            angle_max = index_max - y.shape[0]/2
            angle = angle_max - (angle_max-angle_min)/2
            
            distance = np.nanmin(y[int(index_min):int(index_max)])
            
            color = cone["name"]
            cones.append({"distance":distance, "color":color, "angle": angle})

        fig, ax = plt.subplots()
        ax.imshow(self.image_data)
        ax.plot(x, y*130, 'o')
        fig.canvas.draw()

        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        print(cones)
        data = self.br.cv2_to_imgmsg(img)
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
