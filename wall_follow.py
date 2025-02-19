#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np
import time
#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

#PID CONTROL PARAMS
kp = 2#TODO
kd = 5#TODO
ki = 0#TODO
servo_offset = 0.0
prev_error = 0.0 
error = 0.0
integral = 0.0

#WALL FOLLOW PARAMS
ANGLE_RANGE = 270 # Hokuyo 10LX has 270 degrees scan
DESIRED_DISTANCE_RIGHT = 0.8 # meters
DESIRED_DISTANCE_LEFT = 0.55
VELOCITY = 2.00 # meters per second
CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

class WallFollow:
    """ Implement Wall Following on the car
    """
    def __init__(self):
        #Topics & Subs, Pubs
        lidarscan_topic = '/scan'
        drive_topic = '/drive'
        #drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'

        self.lidar_sub =rospy.Subscriber(lidarscan_topic, LaserScan,self.lidar_callback) #TODO: Subscribe to LIDAR
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)#TODO: Publish to drive
    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        #make sure to take care of nans etc.
        #TODO: implement
        anglelist=data.ranges
        mini=data.angle_min
        maxi=data.angle_max
        length=len(anglelist)
        angle1=length*((angle-90)*math.pi/180-mini)/(maxi-mini)
        return anglelist[math.floor(angle1)]

    def pid_control(self, error, velocity):
        global integral
        global prev_error
        global kp
        global ki
        global kd
        integral+=error
        angle = kp*error+ki*integral+kd*(error-prev_error)
        prev_error=error
        if angle<=20*np.pi/180:
            velocity=VELOCITY
        if angle>=20*np.pi/180:
            velocity=1
        #TODO: Use kp, ki & kd to implement a PID controller for 
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    #def followRight(self, data, leftDist):
        #Follow left wall as per the algorithm 
        #TODO:implement
        #return 0.0 

    def lidar_callback(self, data):
        """ 
        """
        c=70
        b=self.getRange(data, 0)
        a=self.getRange(data, c)
        alpha=math.atan((a*math.cos(c*math.pi/180)-b)/(a*math.sin(c*math.pi/180)))
        dist_t=b*math.cos(alpha)
        error = -dist_t+DESIRED_DISTANCE_RIGHT #TODO: replace with error returned by followLeft
        #send error to pid_control
        #rospy.loginfo_throttle(0.5, dist)
        self.pid_control(error, VELOCITY)
        
        
def main(args):
    rospy.init_node("WallFollow_node", anonymous=True)
    wf = WallFollow()
    #rospy.sleep(0.1)
    rospy.spin()

if __name__=='__main__':
	main(sys.argv)
