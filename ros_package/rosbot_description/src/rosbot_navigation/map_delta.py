#!/usr/bin/env python
import rospy
import tf
from sensor_msgs.msg import LaserScan
from matplotlib import pyplot as plt
import matplotlib.patches as patches

import rospy
from sensor_msgs.msg import LaserScan
import sensor_msgs.point_cloud2 as pc2
import laser_geometry.laser_geometry as lg
import numpy as np
lp = lg.LaserProjection()
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion
import pdb

from PIL import Image
from scipy.spatial.distance import cdist
from scipy.cluster.hierarchy import fclusterdata
import yaml
import argparse 
import time

parser = argparse.ArgumentParser(description='Calculate the map delta')
parser.add_argument('--large_map', default=False, type=bool,help='If true this will clip the map to the max size of the laser scan, usefull for large maps')
parser.add_argument('--plot_transformed', default=False, type=bool ,help='Plot the laserscan and the transformed laserscan (VERY SLOW! only for debugging)')
parser.add_argument('--plot_delta', default=True,type=bool, help='Plot the map and the laserscan with the points that are not on the map (VERY SLOW! only for debugging)')
parser.add_argument('--closeness_threshold', default=0.2, type=float, help='The threshold that a laserscan point needs to be close to a map point to be considered on the map in meters')
parser.add_argument('--update_ratio', default=0.5, type=float, help='The ratio of the points that need to be on the map to update the map')

args = parser.parse_args()
large_map = args.large_map
plot_transformed_bool = args.plot_transformed
closeness_threshold = args.closeness_threshold
plot_delta_bool = args.plot_delta
update_ratio_threshold = args.update_ratio
update_counter = 0
update_counter_threshold = 5


def scan_callback(scan):
    """
    callback function for the laser scan this function will be called every time laserscan is called 

    input : scan : the laser scan message
    output: None
    """
    #decide whether to update the map
    

    print("updating...")
    cur_time = time.time()
    cloud, cloud_tf = get_transformed_cloud(scan)

    if plot_transformed_bool:
        plot_transformed(cloud, cloud_tf)

    map_coor = get_map(cloud_tf)

    not_on_map_coor, cloud_tf = find_deltas(cloud_tf, map_coor)

    if update_checker(not_on_map_coor, cloud_tf):
        print("Updating map")
        clustered_coor = cluster_points(not_on_map_coor)
        square_info, square_corners = convert_data_for_sending(clustered_coor)
        if plot_delta_bool:
            plot_delta(map_coor, cloud_tf, clustered_coor, square_info, square_corners)
    else:
        print('The coordinates have not converged yet, map not updating')
    
    print("no update")

    print("this function took", time.time()-cur_time)
def convert_data_for_sending(clustered_coor):
    """
    This takes in the clustered data and converts it so that it can be send to the database
    input : clustered_coor : the clustered coordinates 
            corners : the corners of the squares
    
    output : squares description: the description of the squares where columns are x_len, y_len, and centerpoint_x and centerpoint_y
    """
    squares_description = np.empty((0,4), float)
    corners = np.empty((0,4), float)
    
    for i in range(clustered_coor.shape[0]):
        x_max, x_min = clustered_coor[i][:,0].max(), clustered_coor[i][:,0].min()
        y_max, y_min = clustered_coor[i][:,1].max(), clustered_coor[i][:,1].min()
        # get the length of the sides and the centerpoint coordinates
        x_len = x_max - x_min
        y_len = y_max - y_min
        centerpoint = np.array([(x_max+x_min)/2, (y_max+y_min)/2])

        # add the data to the squares description
        squares_description = np.vstack((squares_description, np.array(([x_len, y_len, centerpoint[0], centerpoint[1]]))))
        corners = np.vstack((corners, np.array(([x_min, x_max, y_min, y_max]))))


    return squares_description, corners
   
 
    
def find_deltas(cloud_tf, map_coor):
    """
    Find the points that are on the laserscan but not on the map

    input : cloud_tf : the transformed cloud
            map_coor : the coordinates of the map in the laser scan coordinate 
            
    output : not_on_map_coor : the coordinates of the points that are not on the map
             cloud_tf : the transformed cloud without the points that are not on the map
    """

    dist = cdist(cloud_tf, map_coor)
    num_neighbours = np.sum(dist<closeness_threshold,axis=1, keepdims=False)
    no_close_neighbours = np.where(num_neighbours==0)[0]
    not_on_map_coor = cloud_tf[no_close_neighbours,:]
    print("no_close_neighbours", no_close_neighbours.shape)

    cloud_tf = np.delete(cloud_tf, no_close_neighbours, axis=0)

    return not_on_map_coor, cloud_tf

def get_map(cloud_tf=None):
    """
    Get the map and transform it to a coordinate system that matches the laser scan

    input : cloud_tf : the transformed cloud

    output : map_coor : the coordinates of the map in the laser scan coordinate system
    """
    image = Image.open("/home/pim/ITP_project/ros_package/rosbot_description/src/rosbot_navigation/maps/simple_room_v3.png").convert('L', colors=2)
    im_array = np.array(image)
    zeros = im_array == 0
    im_array[im_array != 0] = 0
    im_array[zeros] = 1

    with open("/home/pim/ITP_project/ros_package/rosbot_description/src/rosbot_navigation/maps/simple_room_v3.yaml", "r") as f:
        map_data = yaml.load(f)

    map_resolution = map_data['resolution']
    map_origin = map_data['origin']

    # get the coordinates of the map
    map_coor = np.where(im_array==1)
    map_coor = np.array([map_coor[0],map_coor[1]])

    # convert the coordinates to meters
    map_coor = map_coor * map_resolution
    
    # invert the y coordinates
    map_coor[0] = im_array.shape[0] - map_coor[0]

    # adjust the coordinates so that they match the offset from the map.yaml file
    map_coor[0] = map_coor[0] - map_coor[0].min() + map_origin[1]
    map_coor[1] = map_coor[1]  + map_origin[0]

    
    # remove map that is not on the laserscan useful for larger maps 
    if large_map:
        map_coor = map_coor[:,map_coor[0] < cloud_tf[:,1].max()]

    # rotate the map so that it matches the laser scan (could be cleaner)
    map_coor = map_coor.T
    map_coor = np.array([map_coor[:,1], map_coor[:,0]]).T

    return map_coor

def get_transformed_cloud(scan):
    """
    Get the transformed cloud from the laser scan and use the map.yaml file to transform it to the map coordinate system

        input : scan : the laser scan 
        output : 
        cloud : the original cloud (two dimensional)
        cloud_tf : the transformed cloud (two dimensional)
    """

    #convert the laser scan to a pointcloud
    cloud = lp.projectLaser(scan)

    #convert cloud into a numpy array
    cloud = np.array(list(pc2.read_points(cloud, skip_nans=True, field_names=("x", "y", "z"))))

    #get the transformation between the map and the laser
    listener = tf.TransformListener()
    listener.waitForTransform('/map', scan.header.frame_id, rospy.Time(), rospy.Duration(1.0))
    (trans, rot) = listener.lookupTransform('/map', scan.header.frame_id, rospy.Time(0))
    
    # change the quaternion rotations to euler rotation
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

    # discard the z coordinate
    cloud_tf = cloud_tf [:,0:2]
    
    return cloud, cloud_tf



def plot_transformed(cloud, cloud_tf):
    """ 
    Plot the transformed cloud and the original cloud (for debugging only)

    input:  cloud : the original cloud
            cloud_tf : the transformed cloud
    output: None
    """ 

    
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
    plt.show()

def plot_delta(map_coor, cloud_tf, clusters, square_info, corners):
    """
    plot the map, the transformed cloud and the points that are not on the map

    input : map_coor : the coordinates of the map
            cloud_tf : the transformed cloud
            not_on_map_coor : the coordinates of the points that are not on the map
            clusters: x,y coordinates of the clusters
            square_info: the length and width of the squares and the centerpoint of the squares 
            corners : the coordinates of the corners of the squares of the clusters
    output : None
    """
    
    plt.ion()
    plt.gca().set_aspect('equal', adjustable='box')


    # plot the map, the transformed cloud and the points
    plt.scatter(map_coor[:,0], map_coor[:,1], s=0.1, c='b',alpha=0.1)
    plt.scatter(cloud_tf[:,0], cloud_tf[:,1], s=0.1, c='g')

    # plot the clusters and add the legend
    for i in range(clusters.shape[0]):
        centerpoint = square_info[i][0:2]

        # plot the cluster laserscan points
        plt.scatter(clusters[i][:,0],clusters[i][:,1], s=0.1)
        plt.scatter(centerpoint[0], centerpoint[1], s=0.3, marker='x')
        # plot the square
        x_min, x_max, y_min, y_max = corners[i]
        plt.plot([x_min, x_max, x_max, x_min, x_min], [y_min, y_min, y_max, y_max, y_min])


    plt.legend()
   
    plt.title("found {} objects not on map".format(clusters.shape[0]))
    plt.pause(0.1)
    plt.clf()
 


def update_checker(not_on_map, cloud):
    """
    Check whether the number of laserscan point that correspond to the map is more than a certain threshold

    input : not_on_map : the number of laserscan points that are not on the map
            cloud : the original cloud
           
    output : booleon : Whether this is true.
    """

    num_not_on_map = float(not_on_map.shape[0])
    num_on_map = float(cloud.shape[0])
    ratio = num_not_on_map/(num_not_on_map+num_on_map)
    print('ratio: ', ratio)
    if  ratio < update_ratio_threshold:
        update_counter = update_counter + 1
    else:
        update_counter = 0
    if update_counter > update_counter_threshold:
        return True
    else:
        return False
        
    
        
    

def cluster_points(not_on_map_coor):
    """
    Cluster the points that are not on the map for further processing
    input : np.array 2xN : the coordinates of the points that are not on the map
    output : np.array num_clustersxN : the coordinates of the clustered points in a list of arrays
    """
    min_num_in_cluster = 20
    min_dist_between_clusters = 0.4
    clusters = fclusterdata(not_on_map_coor, min_dist_between_clusters, criterion='distance')
    _, counts = np.unique(clusters, return_counts=True)

    # make new arrays each for one cluster
    clusters = np.array([not_on_map_coor[clusters==i] for i in range(1,counts.shape[0]+1)])
    clusters = clusters[counts>min_num_in_cluster]

    print("found {} objects that are not on the map".format(clusters.shape[0]))

    return clusters

if __name__ == '__main__':
    rospy.init_node('map_comparer')
    rate = rospy.Rate(1) # run the loop every second
    rospy.Subscriber('/scan', LaserScan, scan_callback)

    rospy.spin()



