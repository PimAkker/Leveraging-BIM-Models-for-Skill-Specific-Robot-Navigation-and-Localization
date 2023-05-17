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
from PIL import Image
from scipy.spatial.distance import cdist
import yaml


image = Image.open("/home/pim/ITP_project/ros_package/rosbot_description/src/rosbot_navigation/maps/maplarge_corrupted.png").convert('L', colors=2)
im_array = np.array(image)
zeros = im_array == 0
im_array[im_array != 0] = 0
im_array[zeros] = 1


with open("/home/pim/ITP_project/ros_package/rosbot_description/src/rosbot_navigation/maps/atlas_map.yaml", "r") as f:
    map_data = yaml.load(f)

map_resolution = map_data['resolution']
map_origin = map_data['origin']


def scan_callback(scan):
    print(" callback")

    
    cloud = lp.projectLaser(scan)
    #convert cloud into a numpy array
 
    cloud = np.array(list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))))

    #get the transformation between the map and the laser
    listener = tf.TransformListener()
    listener.waitForTransform('/map', scan.header.frame_id, rospy.Time(), rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransform('/map', scan.header.frame_id, rospy.Time(0))
    
    # change the quaternion rotations to euler rotations
    rot = euler_from_quaternion(rot)

    
    cloud_tf = cloud.copy()

    # Calculate the sine and cosine of the rotation angle
    cos_theta = np.cos(rot[2])
    sin_theta = np.sin(rot[2])

    # Calculate the new coordinates of the vector after rotation
    cloud_tf[:,0] = cloud[:,0] * cos_theta - cloud[:,1] * sin_theta
    cloud_tf[:,1] = cloud[:,0] * sin_theta + cloud[:,1] * cos_theta

    # transform the rotation 
    cloud_tf = cloud_tf + trans
    cloud_tf = cloud_tf [:,0:2]
 
    # plot_transformed(cloud, cloud_tf)

    
    # round the coordinates to the nearest integer
    map_coor = np.where(im_array==1)
    map_coor = np.array([map_coor[0],map_coor[1]])
    map_coor = map_coor * map_resolution
    
    # invert the y coordinates
    map_coor[0] = im_array.shape[0] - map_coor[0]
    # adjust the coordinates so that they match the offset from the map.yaml file
    map_coor[0] = map_coor[0] - map_coor[0].min() + map_origin[1]
    map_coor[1] = map_coor[1]  + map_origin[0]

    
    #remove map that is not on the laserscan
    map_coor = map_coor[:,map_coor[0] < cloud_tf[:,1].max()]

    #check whether the k nearest neigbhours are within the closeness_threshold
    k = 3
    closeness_threshold = 0.1

    # pdb.set_trace()
    # tree = KDTree(np.vstack((map_coor.T, cloud_tf)))
    # dist, _ = tree.query(map_coor.T, k=k)

    dist = cdist(map_coor.T, cloud_tf)
    num_neighbours = np.sum(closeness_threshold>dist,axis=0)
    no_close_neighbours = np.where(num_neighbours==0)[0]
    not_on_map_coor = cloud_tf[no_close_neighbours,:]
    print("no_close_neighbours", no_close_neighbours.shape)

    

    # no_close_neighbours = np.sum(dist>closeness_threshold,axis=1)
    # no_close_neighbours = np.where(no_close_neighbours<2)[0] 
    # not_on_map_coor = map_coor[:,no_close_neighbours]
    # print("no_close_neighbours", no_close_neighbours.shape)

    cloud_tf = np.delete(cloud_tf, no_close_neighbours, axis=0)

    plt.figure()
    plt.gca().set_aspect('equal', adjustable='box')

    plt.scatter(map_coor[1], map_coor[0], s=0.001, c='b')
    plt.scatter(cloud_tf[:,0], cloud_tf[:,1], s=0.1, c='g')
    plt.scatter(not_on_map_coor[:,0],not_on_map_coor[:,1], s = 0.1, c='r')
    


    plt.show()
    pdb.set_trace()

def plot_transformed(cloud, cloud_tf):
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
    rospy.init_node('map_comparer')
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.spin()



