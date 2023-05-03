#! /usr/bin/env python3
# unused for now please ignore
import rospy
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
lp = lg.LaserProjection()
import numpy as np
def callback(msg):

    print("<<<<<<<>>>>>>>>>><<<<<<<<<<<<>>>>>>>>>>>>>>")
    plt.show()
    # rospy.init_node('/scan', anonymous=False)
    # rospy.node

    pc2_msg = lp.projectLaser(msg)
    points = pc2.read_points_list(pc2_msg)
    points = np.array(points)
    plt.scatter(points[:,0],points[:,1], s=0.1)
    ranges = np.array(msg.ranges)
    
    # print(np.shape(ranges))
    # plt.plot(msg.ranges)
    # print(msg.ranges)

    plt.show()

rospy.init_node('scan_values')
sub = rospy.Subscriber('/scan', LaserScan, callback)


rospy.spin()