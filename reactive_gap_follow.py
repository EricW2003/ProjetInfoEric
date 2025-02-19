#!/usr/bin/env python
from __future__ import print_function
import sys
import math
import numpy as np

#ROS Imports
import rospy
from sensor_msgs.msg import Image, LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
cone_angle=90
angle_bulle=15
class reactive_follow_gap:
    def __init__(self):
        #Topics & Subscriptions,Publishers
        lidarscan_topic = '/scan'
        #drive_topic='/vesc/ackermann_cmd_mux/input/navigation'
        drive_topic = '/drive'

        self.lidar_sub = rospy.Subscriber(lidarscan_topic, LaserScan,self.lidar_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) #TODO
    
    def preprocess_lidar(self, ranges):
        """ Preprocess the LiDAR scan array. Expert implementation includes:
            1.Setting each value to the mean over some window
            2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = list(ranges)
        length=len(ranges)
        for i in range(length):
            if (i<=length*(1/2-cone_angle/(360*2))) or(i>=length*(1/2+cone_angle/(360*2))):
                proc_ranges[i]=0
        #    else:
        #        if proc_ranges[i]>=3:
        #           proc_ranges[i]=-1
        return proc_ranges
    def find_max_gap(self, free_space_ranges):
        in_gap=False
        start=None
        end=None
        startMax=0
        endMax=0
        for x in range(len(free_space_ranges)):
            if in_gap:
                if free_space_ranges[x]<=0:
                    if (endMax-startMax)<(end-start):
                        endMax=end
                        startMax=start
                        end=None
                        start=None
                        in_gap=False
                else:
                    end=x
            else:
                if free_space_ranges[x]>0:
                    in_gap=True
                    start=x
                    end=x    
        if in_gap:
            if (endMax-startMax)<(end-start):
                        endMax=end
                        startMax=start
                        end=None
                        start=None
                        in_gap=False
        return (startMax, endMax)
    def find_best_point(self, start_i, end_i, ranges):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
	Naive: Choose the furthest point within ranges and go there
        """
        best=(start_i+end_i)//2
        #for i in range(start_i, end_i+1):
        #   if ranges[i]>ranges[best]:
        #       best=i
        return best

    def lidar_callback(self, data):
        """ Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message
        """
        ranges = data.ranges
        #Find closest point to LiDAR
        ranges= self.preprocess_lidar(ranges)
        length=len(ranges)
        closest=int(length*(1/2-cone_angle/(360*2)))+1
        for i in range(int(length*(1/2-cone_angle/(360*2)))+1,int(length*(1/2+cone_angle/(360*2)))):
            if (ranges[i]<ranges[closest]) and (ranges[i]>0):
                closest=i
        #Eliminate all points inside 'bubble' (set them to zero) 
        closest_dist=ranges[closest]
        mini=data.angle_min
        maxi=data.angle_max
        length=len(ranges)
        angle2=int(length*((angle_bulle)*math.pi/180)/(maxi-mini))
        debut=max(0,closest-angle2)
        fin=min(length,angle2+closest)
        for i in range(debut,fin):
            ranges[i]=0
        #Find max length gap 
        start,end=self.find_max_gap(ranges)
        #Find the best point in the gap 
        best_point=self.find_best_point(start, end, ranges)
        final_angle=best_point/length*(maxi-mini)+mini
        #Change the best point to avoid collision in sharp angles (plus or minus correction angle)
        velocity=6 #à changer
        corr_angle=70
        if closest_dist<0.8:
            velocity=0.5
            if final_angle>0:
                final_angle+=corr_angle/180*math.pi
            else:
                final_angle+=-corr_angle/180*math.pi
        #Publish Drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = final_angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)
def main(args):
    rospy.init_node("FollowGap_node", anonymous=True)
    rfgs = reactive_follow_gap()
    rospy.sleep(0.05)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)