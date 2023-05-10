#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid
from matplotlib import pyplot as plt
import rospy
from sensor_msgs.msg import LaserScan, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np
lp = lg.LaserProjection()
from time import sleep
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from tf.transformations import euler_from_quaternion
import pdb
import pcl_ros

def scan_callback(scan):
    print(" callback")

    
    cloud = lp.projectLaser(scan)
    #convert cloud into a numpy array
 
    cloud = np.array(list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))))


    

    #get the transformation between the base_link and the laser
    listener = tf.TransformListener()

    listener.waitForTransform('/map', scan.header.frame_id, rospy.Time(), rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransform('/map', scan.header.frame_id, rospy.Time(0))
    
    rot = euler_from_quaternion(rot)
    print(trans, rot)
    
    cloud_tf = cloud.copy()

    # Calculate the sine and cosine of the rotation angle
    cos_theta = np.cos(rot[2])
    sin_theta = np.sin(rot[2])

    # Calculate the new coordinates of the vector after rotation
    cloud_tf[:,0] = cloud[:,0] * cos_theta - cloud[:,1] * sin_theta
    cloud_tf[:,1] = cloud[:,0] * sin_theta + cloud[:,1] * cos_theta

    
    # pdb.set_trace()

    

    #apply the transformation to the point cloud

    # show the transformed and untransformed point cloud
    plt.ion()
    plt.show()

    ax = plt.subplot(1,2,1)
    plt.title("Untransformed")
    plt.scatter(cloud[:,0],cloud[:,1], s=0.1)
    ax = plt.subplot(1,2,2)
    plt.title("Transformed")
    plt.scatter(cloud_tf[:,0],cloud_tf[:,1], s=0.1)
    plt.pause(0.0000001)
    plt.clf()
    
    




if __name__ == '__main__':
    rospy.init_node('scan_listener')
    rospy.Subscriber('/scan', LaserScan, scan_callback)
    rospy.spin()
