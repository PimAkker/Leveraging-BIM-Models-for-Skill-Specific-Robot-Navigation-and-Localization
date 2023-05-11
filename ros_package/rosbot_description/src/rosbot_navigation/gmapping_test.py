# #! /usr/bin/env python3

# import rospy
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from matplotlib import pyplot as plt
# import sensor_msgs.point_cloud2 as pc2
# import laser_geometry.laser_geometry as lg
# lp = lg.LaserProjection()
# from nav_msgs.msg import OccupancyGrid
# import rospy
# from sensor_msgs.msg import LaserScan
# from nav_msgs.msg import Odometry
# from message_filters import ApproximateTimeSynchronizer, Subscriber
# import numpy as np

# import tf

# def callback(scan_sub, odom_sub):

#     print("<<<<<<<>>>>>>>>>><<<<<<<<<<<<>>>>>>>>>>>>>>")
    
#     listener = tf.TransformListener()
#     listener.waitForTransform('/map', '/laser', rospy.Time(), rospy.Duration(1.0))
#     (trans, rot) = listener.lookupTransform('/map', '/laser', rospy.Time())
    
#     print(trans)
#     #change points to relative position

#     # plt.scatter(points[:,0],points[:,1], s=0.1)
#     # #set position of robot as red color
#     # plt.scatter(x,y, s=10, c='r')
#     # # ranges = np.array(msg.ranges)    

#     # plt.show()

# print(" test")
# rospy.init_node('scan_values')
# scan_sub = Subscriber('/scan', LaserScan)
# odom_sub = Subscriber('/odom', Odometry)
# ats = ApproximateTimeSynchronizer([scan_sub, odom_sub], queue_size=10, slop=0.1)
# ats.registerCallback(callback)


# rospy.spin()


import rospy
from nav_msgs.msg import OccupancyGrid

# Global variable to store the latest map message
latest_map = None

# Map callback function
def map_callback(msg):
    global latest_map
    latest_map = msg

# Create a ROS node
rospy.init_node('map_subscriber')

# Subscribe to the map topic
rospy.Subscriber('/map', OccupancyGrid, map_callback)

# Wait for the first map message to be received
while latest_map is None and not rospy.is_shutdown():
    rospy.sleep(0.1)

# Print the latest map info
if latest_map is not None:
    print('Latest map info:')
    print('Map width: ', latest_map.info.width)
    print('Map height: ', latest_map.info.height)
    print('Map resolution: ', latest_map.info.resolution)