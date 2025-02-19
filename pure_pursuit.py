#!/usr/bin/env python
import rospy
import sys
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
# TODO: import ROS msg types and libraries
import numpy as np
import tf
import csv
import math
def read_csv_to_list_of_tuples(file_path):
    result = []
    with open(file_path, mode='r') as file:
        csv_reader = csv.reader(file)
        prev_x, prev_y = None, None
        for row in csv_reader:
            if len(row) >= 2:
                x, y = float(row[0]), float(row[1])
                if prev_x is None and prev_y is None:
                    result.append((x, y))
                    prev_x, prev_y = x, y
                else:
                    distance = math.sqrt((x - prev_x) ** 2 + (y - prev_y) ** 2)
                    if distance > 0.5:  # 50 cm = 0.5 meters
                        result.append((x, y))
                        prev_x, prev_y = x, y
    return result

# Exemple d'utilisation
file_path = '/home/wang/rcws/logs/wp-2025-01-10-15-19-53.csv'

#file_path = '/home/nvidia/Wangeric/wp-2025-01-17-15-48-25.csv'
waypoints = read_csv_to_list_of_tuples(file_path)
K=1#coefficient de proportionalite entre steering angle et la courbure
L=0.7 #distance entre le centre de la voiture et le point de rotation
class PurePursuit(object):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        # TODO: create ROS subscribers and publishers.
        odom_topic = '/odom'
        #odom_topic = '/pf/pose/odom'
        drive_topic = '/drive'
        #drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        self.marker_pub = rospy.Publisher ("/dynamic_viz",Marker, queue_size=10)
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry,self.pose_callback) #TODO
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10) #TODO
        
    def pose_callback(self, pose_msg):
        # TODO: find the current waypoint to track using methods mentioned in lecture 
        position1=pose_msg.pose.pose.position
        quaternion = np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w])
        orientation1= tf.transformations.euler_from_quaternion(quaternion)
        x,y=position1.x,position1.y #position de la voiture
        x1,y1=None,None
        x2,y2=None,None
        for point in waypoints:
            if np.cos(orientation1[2])*(point[0]-x)+np.sin(orientation1[2])*(point[1]-y)>=0:
                if (x-point[0])**2+(y-point[1])**2<L**2:
                    if x1 is None and y1 is None:
                        x1,y1=point[0],point[1]
                    else:   
                        if (x-point[0])**2+(y-point[1])**2>(x-x1)**2+(y-y1)**2:
                            x1,y1=point[0],point[1]
                else:
                    if x2 is None and y2 is None:
                        x2,y2=point[0],point[1]
                    else:
                        if (x-point[0])**2+(y-point[1])**2<(x-x2)**2+(y-y2)**2:
                            x2,y2=point[0],point[1]

        marker = Marker()
        marker.id = int(x1*x2*100000) # change this if more than one marker: each one should have a different id
        marker.header.frame_id = 'map'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x1
        marker.pose.position.y = y1
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
        a=(x1+x2)/2
        b=(y1+y2)/2 
        while (x1-x2)**2+(y1-y2)**2>0.01**2:
            a=(x1+x2)/2
            b=(y1+y2)/2
            if (a-x)**2+(b-y)**2<L**2:
                x1=a
                y1=b
            else:
                x2=a
                y2=b
        (xf,yf)=(a,b)
        
        # TODO: transform goal point to vehicle frame of reference
        xf,yf=(xf-x)*np.cos(orientation1[2])+(yf-y)*np.sin(orientation1[2]),-(xf-x)*np.sin(orientation1[2])+(yf-y)*np.cos(orientation1[2])
        # TODO: calculate curvature/steering angle

        gamma=2*yf/(L**2)
        steerangle=K*gamma
        if steerangle>0.4189:
            steerangle=0.4189       
        if steerangle<-0.4189:
            steerangle=-0.4189
        # TODO: publish drive message, don't forget to limit the steering angle between -0.4189 and 0.4189 radians
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        drive_msg.header.frame_id = "base_link"
        drive_msg.drive.steering_angle = steerangle
        drive_msg.drive.speed = 1
        self.drive_pub.publish(drive_msg)
        


def main(args):
    rospy.init_node('pure_pursuit_node')
    pp = PurePursuit()
    rospy.sleep(0.05)
    rospy.spin()
if __name__ == '__main__':
    main(sys.argv)