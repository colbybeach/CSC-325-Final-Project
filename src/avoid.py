#!/usr/bin/env python3

# Author: Hope Crisafi
# 02/08/24

import numpy as np
import rospy, math
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion

OBSTACLE_DISTANCE_THRESHOLD = 0.5

class Robot:
    def __init__(self):
        rospy.init_node('robot', anonymous=True)
        self.turtle_pose = Pose()
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #topic to publish to, message type
        self.laserScan_sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback)
        self.odometry_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.front = 0.0
        self.left = 0.0
        self.behind = 0.0
        self.right = 0.0
        self.yaw = 0.0
        self.nearing_object = False
        
    def poseCallback(self,data):
        self.turtle_pose.x = data.x
        self.turtle_pose.y = data.y
        self.turtle_pose.theta = data.theta

    def scanCallback(self, data):

        front_distances = data.ranges[-10:] + data.ranges[:10]
        self.front = np.mean([dist for dist in front_distances if not np.isinf(dist) and dist != 0])

        left_distances = data.ranges[80:100]
        self.left = np.mean([dist for dist in left_distances if not np.isinf(dist) and dist != 0])

        behind_distances = data.ranges[170:190]
        self.behind = np.mean([dist for dist in behind_distances if not np.isinf(dist) and dist != 0])

        right_distances = data.ranges[260:280]
        self.right = np.mean([dist for dist in right_distances if not np.isinf(dist) and dist != 0])


        self.nearing_object = self.front < OBSTACLE_DISTANCE_THRESHOLD

        print("Front: {:.2f}, Left: {:.2f}, Behind: {:.2f}, Right: {:.2f}".format(
            data.ranges[0], data.ranges[90], data.ranges[180], data.ranges[270]))



    def odom_callback(self, data):
        y = data.pose.pose.position.y
        x = data.pose.pose.position.x
        
        quart = [data.pose.pose.orientation.x, data.pose.pose.orientation.y,
            data.pose.pose.orientation.z, data.pose.pose.orientation.w]
        
        euler = euler_from_quaternion(quart)
        self.yaw = euler[2]


    def move(self, distance, isForward):
        outData = Twist()
        t0 = rospy.get_rostime().secs

        while t0 == 0:
            t0 = rospy.get_rostime().secs

        current_distance = 0
        rate = rospy.Rate(10)
    
        if isForward == True: 
            outData.linear.x = 0.2
        else: 
            outData.linear.x = -0.2
        
        while current_distance < distance:
            self.velocity_pub.publish(outData)
            t1 = rospy.get_rostime().secs
            current_distance = outData.linear.x * (t1-t0)
            rate.sleep()

    def stop_moving(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        self.velocity_pub.publish(vel_msg)


    def move_and_avoid(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.nearing_object:
                print("No object!")
                self.move(0.05, True)
            else:
                self.stop_moving()
                print("Obstacle detected, rotating")

        rate.sleep()

# ************** 
# HELPER METHODS
# **************  

    def normalize_angle(self, angle):
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def is_yaw_close(self, current_yaw, target_yaw, tolerance):
        return abs(self.normalize_angle(current_yaw - target_yaw)) < tolerance

def findDistance(x1,x0,y1,y0):
    newDistance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
    return newDistance


def degrees2radians(angle):
	return angle * (math.pi/180.0)



if __name__ == '__main__':
    robot = Robot()

    robot.move_and_avoid()

    rospy.spin()