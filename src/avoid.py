#!/usr/bin/env python3
#AUTHOR: Colby Beach 
#DATE: 2/7/24

import rospy
import math
from turtlesim.msg import Pose
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

class Robot:

    def __init__(self):
        self.turtlesim_pose = Pose()
        self.rotate_flag = False
        self.yaw = 0
        self.laserData = {}

    def degrees2radians(self, angle):
        return angle * (math.pi/180.0)



    def poseCallback(self, data):
        self.turtlesim_pose = data


    def laserCallback(self, data):
        for x in range (4):
            self.laserData.update({x*45:data.ranges[x*45]})

        
        if min(data.ranges[-20], data.ranges[0], data.ranges[20]) < 0.5 and min(data.ranges[-20], data.ranges[0], data.ranges[20]) > 0.0:
            print("Obstacle detected!")
            # self.rotate_flag = True




    # ## WITH THIS CODE I AM CHECKING WHICH 45 DEGREE ANGLE HAS THE MOST SPACE AND USING THAT 
    # ## TO HELP ROTATE THE ROBOT
    # def maxDistance(self):
    #     angle = max(self.laserData, key=self.laserData.get)
    #     print("Max distance angle:")
    #     print(angle)
    #     return angle


    # def odomCallback(self, msg):
    #     quaternion = (
    #         msg.pose.pose.orientation.x,
    #         msg.pose.pose.orientation.y,
    #         msg.pose.pose.orientation.z,
    #         msg.pose.pose.orientation.w
    #     )
    #     (roll, pitch, yaw) = euler_from_quaternion(quaternion)
    #     self.yaw = yaw
    

    # def rotate(self):
    #     publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #     twister = Twist()
    #     rate = rospy.Rate(10)

    #     start_angle = self.yaw

    #     while abs(self.yaw - start_angle) < self.degrees2radians(self.maxDistance()):
    #         twister.angular.z = 0.5 
    #         publisher.publish(twister)
    #         rate.sleep()

    #     twister.angular.z = 0
    #     publisher.publish(twister)



    # def move_forward(self):
    #     publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    #     twister = Twist()
    #     rate = rospy.Rate(10)

    #     t0 = rospy.get_rostime().secs
    #     while t0 == 0:
    #         t0 = rospy.get_rostime().secs

    #     twister.linear.x = 0.2  

    #     while not self.rotate_flag:
    #         publisher.publish(twister)
    #         rate.sleep()

    #     twister.linear.x = 0
    #     publisher.publish(twister)




def avoid_obstacles():
    rospy.init_node('robot_mover', anonymous=True)
    my_robot = Robot()

    rospy.Subscriber("/scan", LaserScan, my_robot.laserCallback)
    rospy.Subscriber('/odom', Odometry, my_robot.odomCallback)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # my_robot.move_forward()  
        # my_robot.rotate() 
        # my_robot.rotate_flag = False

        rate.sleep()

if __name__ == '__main__':
    try:
        avoid_obstacles()
    except rospy.ROSInterruptException:
        pass
