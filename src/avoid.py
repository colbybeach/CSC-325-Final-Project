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
        self.front = np.mean([dist for dist in front_distances if not np.isinf(dist)])
        
        left_distances = data.ranges[80:100]
        self.left = np.mean([dist for dist in left_distances if not np.isinf(dist)])
        
        behind_distances = data.ranges[170:190]
        self.behind = np.mean([dist for dist in behind_distances if not np.isinf(dist)])
        
        right_distances = data.ranges[260:280]
        self.right = np.mean([dist for dist in right_distances if not np.isinf(dist)])

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

    def rotate(self, angle, clockwise):
        print("ROTATING")
        outData = Twist()

        angular_speed = degrees2radians(30)
        outData.angular.z = -angular_speed if clockwise else angular_speed

        target_yaw = self.normalize_angle(self.yaw + degrees2radians(angle) * (-1 if clockwise else 1))

        self.velocity_pub.publish(outData)
        rospy.sleep(0.1)

        current_yaw = self.yaw
        while not self.is_yaw_close(current_yaw, target_yaw, 0.05):
            current_yaw = self.yaw
            self.velocity_pub.publish(outData)

        self.stop_moving()

    def rotate_direction_most_space(self):
        if self.left > self.right:
            print("left greater")
            self.rotate(45, False)
        else:
            print("right greater")
            self.rotate(45, True)

    def move_and_avoid(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if not self.nearing_object:
                print("Moving forward")
                self.move(0.05, True)
            else:
                self.stop_moving()
                print("Obstacle detected, rotating")
                self.rotate_direction_most_space()
                if self.front > OBSTACLE_DISTANCE_THRESHOLD:
                    self.nearing_object = False
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


"""
to do the fancy stuff:
-I had to modify the rotate function better to fully accommodate the yaw instead of time
-rotate recorded the current and target yaw based on desired rotation angle (got data from callback)
-I had to make sure to stop the robot after the rotation was done or this caused issues
-I got some helper functions to normalize the yaw to be within -pi to pi. which solved an issue
    I had with the robot not rotating. I dont completely understand the math but someone on 
    stackoverflow posted about this and it worked
-I wrote a function that determines which way it should rotate by checking the left and right 
    laser data to see which is greater
-I also wrote a function that stops the robot, and this helped solve some other movement issues
    with rotating, strange how it solved issues that I did not think it would
-The laser callback had to be updated to account for the value being "inf", which would occur if 
    the robot was against a wall or if it had enough space that it went out of range
    -I ended up getting the mean of the range of measurements for each side for better accuracy
-All of this comes together in the move_and_avoid() method which avoids obsticles
"""