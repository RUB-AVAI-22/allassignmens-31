import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Float32MultiArray
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovariance, PoseWithCovarianceStamped, PoseStamped
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import matplotlib.pyplot as plt
import math


gierwinkel =0
posx=0
posy=0
xcone=0
ycone=0

class OdomSub(Node):

    def __init__(self):
        super().__init__('odom_subscriber_node')
        self.x_pos = []
        self.y_pos = []
        self.yaw_pos = []
        self.coordcones=[]
        self.figure, self.ax = plt.subplots()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        
        #Cones message
        self.create_subscription(Odometry, '/odom', self.callback, qos_profile_sensor_data)
        self.cones_subscription = self.create_subscription(Float32MultiArray, "cones", self.cones_callback, 1)
        self.cones = []
        print('node started')

    def cones_callback(self, msg):
        cones = np.array(msg.data)
        self.cones = cones.reshape(-1, 3)
        global xcone,ycone
        """
        for cone in self.cones:
            xyarray.append([cone[0]*math.cos(- cone[2]*math.pi/180),cone[0]*math.sin(- cone[2]*math.pi/180)])
        """
        for cone in self.cones:
            asked=True
            xcone = math.cos(gierwinkel-cone[2]*math.pi/180)*cone[0]+posx
            ycone = math.sin(gierwinkel-cone[2]*math.pi/180)*cone[0]+posy
            for coordcone in self.coordcones:
                if math.isclose(xcone,coordcone[0],abs_tol=0.05):
                    asked=False
                if math.isclose(ycone,coordcone[0],abs_tol=0.05):
                    asked=False
            if asked:
                self.coordcones.append([xcone,ycone])
        print("cones x,y", self.coordcones)
        print(f"Cones detected: {self.cones}")

    def callback(self, msg):
        #pose = PoseStamped()
        posex = float(msg.pose.pose.position.x) #extrahiere x position (im odom koordinatensystem)
        posey = float(msg.pose.pose.position.y) #extrahiere y position (im odom koordinatensystem)
        quatw = float(msg.pose.pose.orientation.w) #extrahiere orientierung des roboters, darstellung in quaternionen
        quatz = float(msg.pose.pose.orientation.z)
        quatx = float(msg.pose.pose.orientation.x)
        quaty = float(msg.pose.pose.orientation.y)
        yaw = math.atan2(2*(quatw*quatz+quatx*quaty), 1-2*(quaty ** 2+quatz ** 2))  #umrechnen in gierwinkel bzw rotation um z-achse
        yawdegree = yaw * 180 / math.pi
        global gierwinkel, posx,posy
        gierwinkel=yaw
        posx=posex
        posy=posey
        print("odom topic", gierwinkel,posx,posy)
        #self.get_logger().info('-> %s' % msg.ranges)
        #self.get_logger().info('position x: %s' % posex)
        #self.get_logger().info('position y: %s' % posey)
        #self.get_logger().info('quaternion w: %s' % quatw)
        #self.get_logger().info('quaternion x: %s' % quatx)
        #self.get_logger().info('quaternion y: %s' % quaty)
        #self.get_logger().info('quaternion z: %s' % quatz)
        #self.get_logger().info('frame header: %s' % msg.header)
        #self.get_logger().info('winkel theta rad: %s' % yaw)
        #self.get_logger().info('winkel theta grad: %s' % yawdegree)

        self.x_pos = posex
        self.y_pos = posey
        self.yaw_pos = yaw
        self.plot_position()


    def plot_position(self):
        #xpos = 0
        #ypos = 0
        #plt.clf()

        #try:
         #   pose.remove()
          #  del pose
        #except:
         #   pass

        pose = self.ax.quiver(self.x_pos, self.y_pos, math.cos(self.yaw_pos), math.sin(self.yaw_pos))
        #plot cone
        if self.coordcones!=[]:
            self.ax.plot(self.coordcones[-1][0],self.coordcones[-1][1],marker='o')

        #plt.quiver(self.x_pos, self.y_pos, math.cos(self.yaw_pos), math.sin(self.yaw_pos))
        #plt.xlim([-1, 1])
        #plt.ylim([-1, 1])
        #self.get_logger().info('theta: %s' % self.yaw_pos)
        #plt.xlabel('X Position')
        #plt.ylabel('Y Position')
        #plt.title('Map')
        plt.pause(0.01)
        pose.remove()
        del pose
        #x_direct = math.cos(yaw)
        #y_direct = math.sin(yaw)
        #fig, ax = plt.subplots(figsize=(12, 7))
        #ax.quiver(posex, posey, x_direct, y_direct)

        #plt.draw()

def main(args=None):
    rclpy.init(args=args)
    sub = OdomSub()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
        main()
