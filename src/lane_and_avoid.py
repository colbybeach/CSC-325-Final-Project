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
        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #topic to publish to, message type
        self.laserScan_sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback)


        self.front = 0.0
        self.left = 0.0
        self.behind = 0.0
        self.right = 0.0
        self.yaw = 0.0
        self.nearing_object = False

        self.twist = Twist()
        
        
        self.lastError = 0
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)


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




    def cbFollowLane(self, desired_center):

        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        


        if not self.nearing_object:
            print("No object!")
            self.twist.linear.x = 0.1
            self.twist.linear.y = 0
            self.twist.linear.z = 0
            self.twist.angular.x = 0
            self.twist.angular.y = 0
            self.twist.angular.z = -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0)
            self.velocity_pub.publish(self.twist)
            rate = rospy.Rate(10)

        else:
            self.twist.linear.x = 0
            print("Obstacle detected, rotating")

        rate.sleep()



if __name__ == '__main__':
    laneAndAvoid = LaneAndAvoid()
    rospy.spin()