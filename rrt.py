#!/usr/bin/env python
import numpy as np
import time
from numpy import linalg as LA
import math
import random
import rospy
import sys
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import OccupancyGrid
import yaml
from PIL import Image
#from tf import transform_listener
import tf
import csv
#La fonction read_csv_to_list_of_tuples permet de lire un fichier csv et de le convertir en liste de tuples
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
                    if distance > 2:  # 2m
                        result.append((x, y))
                        prev_x, prev_y = x, y
    return result

# Obtention des waypoints
file_path = '/home/wang/rcws/logs/wp-2025-01-10-15-19-53.csv'
#file_path = '/home/nvidia/Wangeric/wp-2025-01-17-15-48-25.csv'
waypoints = read_csv_to_list_of_tuples(file_path)



p=0.4#portion du segment qu'on parcourt de nearest_point a sampled_point
goal_distance=0.6 #distance maximale entre le dernier noeud et le goal pour arreter
L=1 #distance du pure_pursuit
K=0.4#coefficient de proportionalite entre steering angle et la courbure
R=2 #rayon de la zone de recherche


# TODO: import as you need

# class def for tree nodes
# It's up to you if you want to use this
class Node(object):
    def __init__(self):
        self.x = None
        self.y = None
        self.parent = None
        self.cost = None # only used in RRT*
        self.is_root = False
resolution=0.05
width=2048
height=2048
# class def for RRT
class RRT(object):
    def __init__(self):
        # topics, not saved as attributes
        # TODO: grab topics from param file, you'll need to change the yaml file
        # you could add your own parameters to the rrt_params.yaml file,
        # and get them here as class attributes as shown above.

        # TODO: create subscribers
        odom_topic = '/odom'
        #odom_topic = '/pf/pose/odom'
        scan_topic = '/scan'
        rospy.Subscriber(odom_topic, Odometry, self.pf_callback)
        rospy.Subscriber(scan_topic, LaserScan, self.scan_callback)
        map_topic='/map'
        rospy.Subscriber(map_topic, OccupancyGrid, self.map_callback)
        # publishers
        # TODO: create a drive message publisher, and other publishers that you might need
        drive_topic = '/drive'
        #drive_topic = '/vesc/ackermann_cmd_mux/input/navigation'
        self.marker_pub = rospy.Publisher ("/dynamic_viz",Marker, queue_size=10)
        self.drive_pub = rospy.Publisher(drive_topic, AckermannDriveStamped, queue_size=10)
        self.occupancygrid = OccupancyGrid()
        self.dynamic_map = OccupancyGrid()
        self.orientation=None
        self.position1=Point()
        self.position1.x=0
        self.position1.y=0
        self.remember=[]
        return None
    def map_callback(self, map_msg):
        if len(self.dynamic_map.data)!=0:
            return None
        self.occupancygrid.info.resolution = map_msg.info.resolution
        self.occupancygrid.info.width = map_msg.info.width
        self.occupancygrid.info.height = map_msg.info.height
        width = self.occupancygrid.info.width
        height = self.occupancygrid.info.height
        resolution = self.occupancygrid.info.resolution
        self.occupancygrid.info.origin.position.x = map_msg.info.origin.position.x
        self.occupancygrid.info.origin.position.y = map_msg.info.origin.position.y
        self.occupancygrid.info.origin.position.z = map_msg.info.origin.position.z
        self.occupancygrid.data = map_msg.data
        self.dynamic_map.info.resolution = map_msg.info.resolution
        self.dynamic_map.info.width = map_msg.info.width
        self.dynamic_map.info.height = map_msg.info.height
        self.dynamic_map.info.origin.position.x = map_msg.info.origin.position.x
        self.dynamic_map.info.origin.position.y = map_msg.info.origin.position.y
        self.dynamic_map.info.origin.position.z = map_msg.info.origin.position.z
        self.dynamic_map.data = [0]*len(self.occupancygrid.data)
        return None
    def scan_callback(self, scan_msg):
        """
        LaserScan callback, you should update your occupancy grid here
        
        Args: 
            scan_msg (LaserScan): incoming message from subscribed topic
        Returns:
        """
        if len(self.dynamic_map.data)==0:
            return None
        for n in self.remember:
            self.dynamic_map.data[n]=0
        self.remember=[]
        for i in range(len(scan_msg.ranges)):
            if scan_msg.ranges[i]<scan_msg.range_max and scan_msg.ranges[i]>scan_msg.range_min:
                x=scan_msg.ranges[i]*math.cos(scan_msg.angle_min+i*scan_msg.angle_increment+self.orientation[2])+self.position1.x
                y=scan_msg.ranges[i]*math.sin(scan_msg.angle_min+i*scan_msg.angle_increment+self.orientation[2])+self.position1.y
                n=self.frommaptogrid(x,y)

                self.dynamic_map.data[n]=100
                self.remember.append(n)
        tree=[]
        new_node=Node()
        new_node.x=self.position1.x
        new_node.y=self.position1.y
        new_node.parent=None
        new_node.is_root=True
        tree.append(new_node)
        orientation1=self.orientation[2]
        point=(new_node.x,new_node.y)
        goal_point = waypoints[0]
        for p in waypoints:
            if math.cos(orientation1)*(p[0]-self.position1.x)+math.sin(orientation1)*(p[1]-self.position1.y)>=0 and math.sqrt((p[0]-point[0])**2+(p[1]-point[1])**2)>1:
                if math.cos(orientation1)*(goal_point[0]-self.position1.x)+math.sin(orientation1)*(goal_point[1]-self.position1.y)<0:
                    goal_point = p
                else:
                    if (p[0]-point[0])**2+(p[1]-point[1])**2<(goal_point[0]-point[0])**2+(goal_point[1]-point[1])**2:
                        goal_point = p

        goal_node=Node()
        goal_node.x=goal_point[0]
        goal_node.y=goal_point[1]
        new_node.parent=None
        new_node.is_root=False
        loop = True
        n=0
        seuil=1000 #seuil de la boucle
        while loop:
            n=n+1
            sampled_point = self.sample((new_node.x, new_node.y), orientation1)
            nearest_node = self.nearest(tree, sampled_point)
            new_node1 = self.steer(tree[nearest_node], sampled_point)
            if self.check_collision(tree[nearest_node], new_node1):
                tree.append(new_node1)
                if self.is_goal(new_node1, goal_point[0], goal_point[1]):
                    path = self.find_path(tree, new_node1)
                    loop = False
            if n==seuil:
                path=self.find_path(tree, tree[self.nearest(tree, goal_point)])
                loop=False
        marker = Marker()
        marker.id = int(7) # change this if more than one marker: each one should have a different id
        marker.header.frame_id = 'map'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = path[-1].x
        marker.pose.position.y = path[-1].y
        marker.pose.position.z = 0
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_pub.publish(marker)
    
        x,y=self.position1.x,self.position1.y #position de la voiture
        x1,y1=None,None
        x2,y2=None,None
        for point in path:
            if math.cos(orientation1)*(point.x-x)+math.sin(orientation1)*(point.y-y)>=0:
                if (x-point.x)**2+(y-point.y)**2<L**2:
                    if x1 is None and y1 is None:
                        x1,y1=point.x,point.y
                    else:   
                        if (x-point.x)**2+(y-point.y)**2>(x-x1)**2+(y-y1)**2:
                            x1,y1=point.x,point.y
                else:
                    if x2 is None and y2 is None:
                        x2,y2=point.x,point.y
                    else:
                        if (x-point.x)**2+(y-point.y)**2<(x-x2)**2+(y-y2)**2:
                            x2,y2=point.x,point.y

        if x1 is None and y1 is None:
            x1,y1=x2,y2
        if x2 is None and y2 is None:
            x2,y2=x1,y1
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

        marker = Marker()
        marker.id = int(5) # change this if more than one marker: each one should have a different id
        marker.header.frame_id = 'map'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = xf
        marker.pose.position.y = yf
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

        xf,yf=(xf-x)*math.cos(orientation1)+(yf-y)*math.sin(orientation1),-(xf-x)*math.sin(orientation1)+(yf-y)*math.cos(orientation1)
        gamma=2*yf/(L**2)
        steerangle=K*gamma
        if steerangle>0.4189:
            steerangle=0.4189       
        if steerangle<-0.4189:
            steerangle=-0.4189
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = rospy.Time.now()
        #drive_msg.drive.steering_angle = steerangle
        drive_msg.drive.steering_angle = steerangle
        drive_msg.drive.speed = 0.7
        self.drive_pub.publish(drive_msg)
        #Le point but a atteindre
        marker = Marker()
        marker.id = int(100000) # change this if more than one marker: each one should have a different id
        marker.header.frame_id = 'map'
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
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
        marker.color.g = 0.0
        marker.color.b = 1.0
        self.marker_pub.publish(marker)
        return None
    def pf_callback(self, pose_msg):
        """
        The pose callback when subscribed to particle filter's inferred pose
        Here is where the main RRT loop happens

        Args: 
            pose_msg (PoseStamped): incoming message from subscribed topic
        Returns:

        """

        self.position1=pose_msg.pose.pose.position
        self.position1.x=pose_msg.pose.pose.position.x
        self.position1.y=pose_msg.pose.pose.position.y
        quaternion = np.array([pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z,pose_msg.pose.pose.orientation.w])
        orientation1= tf.transformations.euler_from_quaternion(quaternion)
        self.orientation=orientation1
       
        return None
        
    def fromgridtomap(self,n):
        """
        This method should convert the grid coordinates to the (x,y) coordinates
        """
        i=n%width
        j=n//width
        x = self.occupancygrid.info.origin.position.x+i*resolution
        y = self.occupancygrid.info.origin.position.y+j*resolution
        return (x, y)
    def frommaptogrid(self,x,y):
        """
        This method should convert the (x,y) coordinates to the grid coordinates
        """
        i=int((x-self.occupancygrid.info.origin.position.x)/resolution)
        j=int((y-self.occupancygrid.info.origin.position.y)/resolution)
        return int(j*width+i)
    def point_aleatoire_dans_cercle(self):
        # Génère un angle uniformément entre 0 et 2π
        theta = 2 * math.pi * random.random()
        # Génère une valeur u uniformément entre 0 et 1 et calcule le rayon
        r = R * math.sqrt(random.random())
        # Convertit les coordonnées polaires en coordonnées cartésiennes
        x = r * math.cos(theta)
        y = r * math.sin(theta)
        return x, y
    def sample(self,point,orientation):
        """
        This method should randomly sample the free space, and returns a viable point

        Args:
        Returns:
            (x, y) (float float): a tuple representing the sampled point

        """
        loop=True
        if len(self.occupancygrid.data)==0:
            return point
        while loop:
            (x,y)=(self.point_aleatoire_dans_cercle()[0]+point[0]+R*math.cos(orientation),self.point_aleatoire_dans_cercle()[1]+point[1]+R*math.sin(orientation))
            n=self.frommaptogrid(x,y)
            if len(self.dynamic_map.data)==0:
                print("dynamic_map.data is empty")
                return (x,y)
            if self.occupancygrid.data[n]<50 and self.dynamic_map.data[n]<50:
                loop=False
        return (x,y)


    def nearest(self, tree, sampled_point):
        """
        This method should return the nearest node on the tree to the sampled point

        Args:
            tree ([]): the current RRT tree
            sampled_point (tuple of (float, float)): point sampled in free space
        Returns:
            nearest_node (int): index of neareset node on the tree
        """
        nearest_node = 0
        for i in range(len(tree)):
            if (tree[i].x-sampled_point[0])**2+(tree[i].y-sampled_point[1])**2<(tree[nearest_node].x-sampled_point[0])**2+(tree[nearest_node].y-sampled_point[1])**2:
                nearest_node = i
        return nearest_node

    def steer(self, nearest_node, sampled_point):
        """
        This method should return a point in the viable set such that it is closer 
        to the nearest_node than sampled_point is.

        Args:
            nearest_node (Node): nearest node on the tree to the sampled point
            sampled_point (tuple of (float, float)): sampled point
        Returns:
            new_node (Node): new node created from steering
        """
        p1=math.sqrt((nearest_node.x-sampled_point[0])**2+(nearest_node.y-sampled_point[1])**2)
        if p1==0:
            return nearest_node
        if p<=p1:
            p1=p
        g=math.sqrt((sampled_point[0]-nearest_node.x)**2+(sampled_point[1]-nearest_node.y)**2)
        x=nearest_node.x+math.sqrt(p1/g)*(sampled_point[0]-nearest_node.x)
        y=nearest_node.y+math.sqrt(p1/g)*(sampled_point[1]-nearest_node.y)
        new_node = Node()
        new_node.x=x
        new_node.y=y
        new_node.parent=nearest_node
        #Pas encore de fonction de coût

        return new_node
    

    def check_collision(self, nearest_node, new_node):
        """
        This method should return whether the path between nearest and new_node is
        collision free.

        Args:
            nearest (Node): nearest node on the tree
            new_node (Node): new node from steering
        Returns:
            collision (bool): whether the path between the two nodes are in collision
                              with the occupancy grid
        """
        dist=math.sqrt((nearest_node.x-new_node.x)**2+(nearest_node.y-new_node.y)**2)
        n=math.floor(dist/resolution)
        #print(n)
        for i in range(n):
            x=nearest_node.x+i*(new_node.x-nearest_node.x)/n
            y=nearest_node.y+i*(new_node.y-nearest_node.y)/n
            if self.occupancygrid.data[self.frommaptogrid(x,y)]>50 or self.dynamic_map.data[self.frommaptogrid(x,y)]>50:
                return False
        return True

    def is_goal(self, latest_added_node, goal_x, goal_y):
        """
        This method should return whether the latest added node is close enough
        to the goal.

        Args:
            latest_added_node (Node): latest added node on the tree
            goal_x (double): x coordinate of the current goal
            goal_y (double): y coordinate of the current goal
        Returns:
            close_enough (bool): true if node is close enoughg to the goal
        """
        if math.sqrt((latest_added_node.x-goal_x)**2+(latest_added_node.y-goal_y)**2)<goal_distance:
            return True
        else:
            return False

    def find_path(self, tree, latest_added_node):
        """
        This method returns a path as a list of Nodes connecting the starting point to
        the goal once the latest added node is close enough to the goal

        Args:
            tree ([]): current tree as a list of Nodes
            latest_added_node (Node): latest added node in the tree
        Returns:
            path ([]): valid path as a list of Nodes
        """
        current_node = latest_added_node
        path = []
        while current_node.parent is not None:
            path.append(current_node)
            current_node = current_node.parent
        path.append(current_node)
        path.reverse()
        return path


    # The following methods are needed for RRT* and not RRT
    def cost(self, tree, node):
        """
        This method should return the cost of a node

        Args:
            node (Node): the current node the cost is calculated for
        Returns:
            cost (float): the cost value of the node
        """
        return 0

    def line_cost(self, n1, n2):
        """
        This method should return the cost of the straight line between n1 and n2

        Args:
            n1 (Node): node at one end of the straight line
            n2 (Node): node at the other end of the straint line
        Returns:
            cost (float): the cost value of the line
        """
        return 0

    def near(self, tree, node):
        """
        This method should return the neighborhood of nodes around the given node

        Args:
            tree ([]): current tree as a list of Nodes
            node (Node): current node we're finding neighbors for
        Returns:
            neighborhood ([]): neighborhood of nodes as a list of Nodes
        """
        neighborhood = []
        return neighborhood

def main(args):
    
    rospy.init_node('rrt')
    rrt = RRT()
    rospy.sleep(0.05)
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)