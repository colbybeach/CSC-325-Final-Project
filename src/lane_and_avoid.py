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
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)


        self.front = 0.0
        self.left = 0.0
        self.behind = 0.0
        self.right = 0.0
        self.yaw = 0.0
        self.nearing_object = False

        self.twister = Twist()
        self.lastError = 0
        self.bringItAroundTown = False


    def scanCallback(self, data):

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

    

    def move_robot(self, linear_x, angular_z):
        self.twister.linear.x = linear_x
        self.twister.angular.z = angular_z
        self.velocity_pub.publish(self.twister)

    
    def degrees2radians(self, angle):
        return angle * (math.pi/180.0)




    def rotate(self, angle, isClockwise):
        print("ROTATING")
        outData = Twist()
        t0 = rospy.get_rostime().secs

        while t0 == 0:
            t0 = rospy.get_rostime().secs

        current_angle = 0
        rate = rospy.Rate(10)
        outData.angular.z = self.degrees2radians(30)

        if not isClockwise:
            outData.angular.z = -outData.angular.z


        while current_angle < self.degrees2radians(angle):
            print(current_angle)
            print(angle)
            self.velocity_pub.publish(outData)
            current_angle = (rospy.get_rostime().secs - t0) * self.degrees2radians(30)
            rate.sleep()

        #Always good practice to reset according to previous comments
        outData.angular.z = 0
        self.velocity_pub.publish(outData)
        rospy.sleep(0.5)

        

    def bring_it_around_town(self):
        print("Bring it around town function function function!")
        # Move backward for half a meter
        self.rotate(90, False)  # -90 degrees in radians

        print("1")
        # Move forward half a meter
        self.move(0.5, True)
        # Rotate right 90 degrees
        self.rotate(90, True)  # -90 degrees in radians
        # Move forward half a meter again
        self.move(0.5, True)
        # Rotate left again to complete the loop
        self.rotate(90, True)  # -90 degrees in radians
        # Move forward half a meter again
        self.move(0.5, True)
        # Rotate left again to complete the loop
        self.rotate(90, False)  # -90 degrees in radians
        print("Last")


    def move(self, distance, isForward):
        print("MOVING IN FUNCTION!")
        outData = Twist()
        t0 = rospy.get_rostime().secs

        while t0 == 0:
            t0 = rospy.get_rostime().secs

        current_distance = 0
        rate = rospy.Rate(10)
    
        if isForward == True: 
            outData.linear.x = 0.1
        else: 
            outData.linear.x = -0.1
        
        while abs(current_distance) < distance:
            self.velocity_pub.publish(outData)
            t1 = rospy.get_rostime().secs
            current_distance = outData.linear.x * (t1-t0)
            rate.sleep()
        

        #RESET Veloicty
        outData.linear.z = 0
        self.velocity_pub.publish(outData)
        
        print("Move end")
        rospy.sleep(0.5)

    def cbFollowLane(self, desired_center):

        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        rate = rospy.Rate(10)

        current_time = rospy.Time.now()


        if self.bringItAroundTown:
            print("Bring it around town!")
            self.bringItAroundTown = False
            self.bring_it_around_town()

        elif not self.nearing_object and not self.bringItAroundTown:
            print("No object!")
            self.move_robot(0.1, -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0))

        else:
            print("Obstacle detected!")
            self.move_robot(0,0)
            rospy.sleep(5)

            print("FINISH SLEEP!")
            if self.nearing_object:
                self.bringItAroundTown = True
            print("SUCCESS CHANGE")
            print(self.bringItAroundTown)


        rate.sleep()



if __name__ == '__main__':
    laneAndAvoid = LaneAndAvoid()
    rospy.spin()