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

gierwinkel = 0
posx = 0
posy = 0
xcone = 0
ycone = 0
colors = ['blue', 'orange', 'yellow']

class OdomSub(Node):

    def __init__(self):
        super().__init__('odom_subscriber_node')
        self.x_pos = []
        self.y_pos = []
        self.yaw_pos = []
        self.coordcones = []
        self.goalConeLeft = []
        self.goalConeRight = []
        self.eIntT = 0
        self.eIntR = 0
        self.figure, self.ax = plt.subplots()
        self.ax.set_xlim([-1, 1])
        self.ax.set_ylim([-1, 1])
        self.ax.set_xlabel('X Coordinate')
        self.ax.set_ylabel('Y Coordinate')
        self.ax.set_title('Map')

        # Cones message
        self.create_subscription(Odometry, '/odom', self.callback, qos_profile_sensor_data)
        self.cones_subscription = self.create_subscription(Float32MultiArray, "cones", self.cones_callback, 1)
        self.cones = []
        self.vel_publisher = self.create_publisher(Twist, "cmd_vel", 1)
        self.timer_ = self.create_timer(0.5, self.velpub)
        print('node started')

    def cones_callback(self, msg):
        cones = np.array(msg.data)
        self.cones = cones.reshape(-1, 3)
        global xcone, ycone
        """
        for cone in self.cones:
            xyarray.append([cone[0]*math.cos(- cone[2]*math.pi/180),cone[0]*math.sin(- cone[2]*math.pi/180)])
        """
        for cone in self.cones:
            asked = True
            xcone = math.cos(gierwinkel - cone[2] * math.pi / 180) * cone[0] + posx
            ycone = math.sin(gierwinkel - cone[2] * math.pi / 180) * cone[0] + posy
            for coordcone in self.coordcones:
                #xcoordcone = coordcone[0] * math.cos(- coordcone[2] * math.pi / 180)
                #ycoordcone = coordcone[0] * math.sin(- coordcone[2] * math.pi / 180)
                if math.isclose(xcone, coordcone[0], abs_tol=0.05) and math.isclose(ycone, coordcone[1], abs_tol=0.05):
                    asked = False
            if asked:
                self.coordcones.append([xcone, ycone])
                self.plot_cone(cone[1])

        print("cones x,y", self.coordcones)
        print(f"Cones detected: {self.cones}")

    def callback(self, msg):
        # pose = PoseStamped()
        posex = float(msg.pose.pose.position.x)  # extrahiere x position (im odom koordinatensystem)
        posey = float(msg.pose.pose.position.y)  # extrahiere y position (im odom koordinatensystem)
        quatw = float(msg.pose.pose.orientation.w)  # extrahiere orientierung des roboters, darstellung in quaternionen
        quatz = float(msg.pose.pose.orientation.z)
        quatx = float(msg.pose.pose.orientation.x)
        quaty = float(msg.pose.pose.orientation.y)
        yaw = math.atan2(2 * (quatw * quatz + quatx * quaty), 1 - 2 * (quaty ** 2 + quatz ** 2))  # umrechnen in gierwinkel bzw rotation um z-achse
        yawdegree = yaw * 180 / math.pi
        global gierwinkel, posx, posy
        gierwinkel = yaw
        posx = posex
        posy = posey
        print("odom topic", gierwinkel, posx, posy)
        # self.get_logger().info('-> %s' % msg.ranges)
        # self.get_logger().info('position x: %s' % posex)
        # self.get_logger().info('position y: %s' % posey)
        # self.get_logger().info('quaternion w: %s' % quatw)
        # self.get_logger().info('quaternion x: %s' % quatx)
        # self.get_logger().info('quaternion y: %s' % quaty)
        # self.get_logger().info('quaternion z: %s' % quatz)
        # self.get_logger().info('frame header: %s' % msg.header)
        # self.get_logger().info('winkel theta rad: %s' % yaw)
        # self.get_logger().info('winkel theta grad: %s' % yawdegree)

        self.x_pos = posex
        self.y_pos = posey
        self.yaw_pos = yaw
        self.plot_position()

    def velpub(self):
        #compute goal position and according velocity control inputs for longitudinal and rotational error to publish them
        msg = Twist()
        kpT = 1
        kpR = 0.5
        ki = 0.1
        deltat = 0.5
        min_distL = np.inf
        min_distR = np.inf
        # indexL = 0
        # indexR = 0
        print("test")
        for i in range(len(self.coordcones)):
            if (math.atan((self.coordcones[i][1] - self.y_pos) / (self.coordcones[i][0] - self.x_pos)) - self.yaw_pos > 0 and
                math.atan((self.coordcones[i][1] - self.y_pos) / (self.coordcones[i][0] - self.x_pos)) - self.yaw_pos < math.pi / 2):
                distL = abs(self.coordcones[i][0] - self.x_pos) + abs(self.coordcones[i][1] - self.y_pos)
                if distL < min_distL and distL > 0.05:
                    min_distL = distL
                    indexL = i
            if (math.atan((self.coordcones[i][1] - self.y_pos) / (self.coordcones[i][0] - self.x_pos)) - self.yaw_pos < 0 and
                math.atan((self.coordcones[i][1] - self.y_pos) / (self.coordcones[i][0] - self.x_pos)) - self.yaw_pos > -math.pi / 2):
                distR = abs(self.coordcones[i][0] - self.x_pos) + abs(self.coordcones[i][0] - self.y_pos)
                if distR < min_distR and distL > 0.05:
                    min_distR = distR
                    indexR = i

        try:
            self.goalConeLeft = [self.coordcones[indexL][0], self.coordcones[indexL][1]]
            self.goalConeRight = [self.coordcones[indexR][0], self.coordcones[indexR][1]]

            self.x_goal = (self.goalConeLeft[0] + self.goalConeRight[0]) / 2
            self.y_goal = (self.goalConeLeft[1] + self.goalConeRight[1]) / 2
            errorTranslation = math.sqrt((self.x_goal - self.x_pos) ** 2 + (self.y_goal - self.y_pos) ** 2)
            errorRotation = math.atan((self.y_goal - self.y_pos) / (self.x_goal - self.x_pos)) - self.yaw_pos
            deltat = 0.5
            self.eIntT = self.eIntT + errorTranslation*deltat
            self.eIntR = self.eIntR + errorRotation*deltat
            #PI-controller for longitudinal and rotational error
            vx = kpT * errorTranslation + ki * self.eIntT
            wz = kpR * errorRotation + ki * self.eIntR
            #clamping anti wind-up
            beschraenkungvx = 1
            beschraenkungwz = math.pi/2
            pos1 = -vx+beschraenkungvx
            neg1 = -vx-beschraenkungvx
            pos2 = -wz+beschraenkungwz
            neg2 = -wz-beschraenkungwz
            if vx > 0 and pos1 < 0:
                self.eIntT = self.eIntT - errorTranslation * deltat
                vx = beschraenkungvx
            elif vx < 0 and neg1 > 0:
                self.eIntT = self.eIntT - errorTranslation * deltat
                vx = -beschraenkungvx
            if wz > 0 and pos2 < 0:
                self.eIntR = self.eIntR - errorRotation * deltat
                wz = beschraenkungvx
            elif wz < 0 and neg2 > 0:
                self.eIntT = self.eIntT - errorRotation * deltat
                wz = -beschraenkungvx

            msg.linear.x = vx
            msg.angular.z = wz
            if errorTranslation < 0.02:
                msg.linear.x = 0
                self.eIntT = 0
            if errorRotation < math.pi / 32:
                msg.angular.z = 0
                self.eIntR = 0

            self.vel_publisher.publish(msg)
            self.plot_goal()
        except:
            pass

    def plot_cone(self, color):
        # plot cone
        global colors
        self.ax.plot(self.coordcones[-1][0], self.coordcones[-1][1], marker='o', color=colors[int(color)])

    def plot_goal(self):
        # plot current goal
        self.ax.plot(self.x_goal, self.y_goal, marker='x')

    def plot_position_robot(self):
        # plot current robot pose
        pose = self.ax.quiver(self.x_pos, self.y_pos, math.cos(self.yaw_pos), math.sin(self.yaw_pos))
        plt.pause(0.01)
        pose.remove()
        del pose

def main(args=None):
    rclpy.init(args=args)
    sub = OdomSub()
    rclpy.spin(sub)
    sub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
