#!/usr/bin/env python3

# Author: Hope Crisafi and Colby Beach
# 03/06/24

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


    '''
        INIT EVERYTHING
    '''
    def __init__(self):
        
        rospy.init_node('lane_and_avoid', anonymous=True)

        self.velocity_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) #topic to publish to, message type
        self.laserScan_sub = rospy.Subscriber('/scan', LaserScan, self.scanCallback)
        self.sub_lane = rospy.Subscriber('/detect/lane', Float64, self.cbFollowLane, queue_size = 1)


        self.front = 0.0
        self.nearing_object = False

        self.twister = Twist()
        self.lastError = 0
        self.bringItAroundTown = False


    '''
    LIDAR Scanner callback which gets the information in front
    of it and sets an instance variable to whether it senses an object or not
    '''
    def scanCallback(self, data):

        front_distances = data.ranges[-10:] + data.ranges[:10]
        self.front = np.mean([dist for dist in front_distances if not np.isinf(dist) and dist != 0])

        self.nearing_object = self.front < OBSTACLE_DISTANCE_THRESHOLD
        


    '''
    Rotates turtlebot based off of an angle and if it goes
    Clockwise or not
    '''
    def rotate(self, angle, isClockwise):
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
            self.velocity_pub.publish(outData)
            current_angle = (rospy.get_rostime().secs - t0) * self.degrees2radians(30)
            rate.sleep()


        #Reset Velocity 
        outData.angular.z = 0
        self.velocity_pub.publish(outData)
        rospy.sleep(0.5)


    '''
        Brings it around town baby
    '''
    def bring_it_around_town(self):    

        for i in range(3):
            self.rotate(90, i != 0)
            self.move(0.5, True)

        self.rotate(90, False)

    '''
        Moves the turtlebot based off of a distance and whether 
        it should go forwards or backwards
    '''
    def move(self, distance, isForward):
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
        

        #Reset Veloicty
        outData.linear.z = 0
        self.velocity_pub.publish(outData)
        
        rospy.sleep(0.5)


    '''
        Callback function for the /detect/lane publisher.
        We get the data for detecting the lane, and then inside
        our callback we check whether we should follow the lane,
        stop for an object in front, or go around that object.
    '''
    def cbFollowLane(self, desired_center):

        center = desired_center.data

        error = center - 500

        Kp = 0.0025
        Kd = 0.007

        angular_z = Kp * error + Kd * (error - self.lastError)
        self.lastError = error
        rate = rospy.Rate(10)

        if self.bringItAroundTown:
            self.bringItAroundTown = False
            self.bring_it_around_town()

        elif not self.nearing_object and not self.bringItAroundTown:
            self.set_twister(0.1, -max(angular_z, -2.0) if angular_z < 0 else -min(angular_z, 2.0))

        else:
            self.set_twister(0,0)
            rospy.sleep(5)
            if self.nearing_object:
                self.bringItAroundTown = True


        rate.sleep()
        


    '''
    HELPER FUNCTIONS! Helps to set twister value easily, and change degrees2radians 
    '''
    def set_twister(self, linear_x, angular_z):
        self.twister.linear.x = linear_x
        self.twister.angular.z = angular_z
        self.velocity_pub.publish(self.twister)

    
    def degrees2radians(self, angle):
        return angle * (math.pi/180.0)




if __name__ == '__main__':
    laneAndAvoid = LaneAndAvoid()
    rospy.spin()
