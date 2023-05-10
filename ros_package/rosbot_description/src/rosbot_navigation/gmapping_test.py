#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from matplotlib import pyplot as plt
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
lp = lg.LaserProjection()
from nav_msgs.msg import OccupancyGrid
import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np

import tf

def callback(scan_sub, odom_sub):

    print("<<<<<<<>>>>>>>>>><<<<<<<<<<<<>>>>>>>>>>>>>>")
    
    listener = tf.TransformListener()
    listener.waitForTransform('/map', '/laser', rospy.Time(), rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransform('/map', '/laser', rospy.Time())
    
    print(trans)
    #change points to relative position

    # plt.scatter(points[:,0],points[:,1], s=0.1)
    # #set position of robot as red color
    # plt.scatter(x,y, s=10, c='r')
    # # ranges = np.array(msg.ranges)    

    # plt.show()

print(" test")
rospy.init_node('scan_values')
scan_sub = Subscriber('/scan', LaserScan)
odom_sub = Subscriber('/odom', Odometry)
ats = ApproximateTimeSynchronizer([scan_sub, odom_sub], queue_size=10, slop=0.1)
ats.registerCallback(callback)

rospy.spin()