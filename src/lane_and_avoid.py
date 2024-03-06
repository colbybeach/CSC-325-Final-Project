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
from std_msgs.msg import Float64

OBSTACLE_DISTANCE_THRESHOLD = 0.5

class LaneAndAvoid:
    def __init__(self):

        rospy.init_node('lane_and_avoid', anonymous=True)

        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
        self.laserScan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.detect_lane_callback, queue_size = 1)

        self.twister = Twist()
        self.front = 0.0
        self.left = 0.0
        self.behind = 0.0
        self.right = 0.0
        self.yaw = 0.0
        self.nearing_object = False

        
        self.angular_z = 0.0  
        self.lastError = 0


    def scan_callback(self, data):

        front_distances = data.ranges[-10:] + data.ranges[:10]
        self.front = np.mean([dist for dist in front_distances if not np.isinf(dist) and dist != 0])

        # left_distances = data.ranges[80:100]
        # self.left = np.mean([dist for dist in left_distances if not np.isinf(dist) and dist != 0])

        # behind_distances = data.ranges[170:190]
        # self.behind = np.mean([dist for dist in behind_distances if not np.isinf(dist) and dist != 0])

        # right_distances = data.ranges[260:280]
        # self.right = np.mean([dist for dist in right_distances if not np.isinf(dist) and dist != 0])


        self.nearing_object = self.front < OBSTACLE_DISTANCE_THRESHOLD

        # print("Front: {:.2f}, Left: {:.2f}, Behind: {:.2f}, Right: {:.2f}".format(
        #     data.ranges[0], data.ranges[90], data.ranges[180], data.ranges[270]))


    def stop_robot(self):
        self.twister.linear.x = 0
        self.twister.linear.y = 0
        self.twister.linear.z = 0
        self.twister.angular.x = 0
        self.twister.angular.y = 0
        self.twister.angular.z = 0
        self.velocity_pub.publish(self.twister) 


    def detect_lane_callback(self, desired_center):
        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        ang_z = Kp * error + Kd * (error - self.lastError)
        self.angular_z = -max(ang_z, -2.0) if ang_z < 0 else -min(ang_z, 2.0)
        self.lastError = error



    def run(self):
        rate = rospy.Rate(10)  
        while not rospy.is_shutdown():
            if not self.nearing_object:
                print("No object!")
                self.twister.linear.x = 0.1
                self.twister.angular.z = self.angular_z
                self.velocity_pub.publish(self.twister)
            else:
                print("Obstacle detected, stopping.")
                self.stop_robot()


            rate.sleep()



if __name__ == '__main__':
    laneAndAvoid = LaneAndAvoid()
    laneAndAvoid.run()
    rospy.spin()