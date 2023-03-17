import csv
from get_geometry import *
from gridmap import *
import numpy as np
from matplotlib import pyplot as plt
from matplotlib import colors
import math



def check_grid(coordinate, g_size, map, origin):
    x_coor, y_coor = coordinate
    x_real = x_coor - origin[0]
    y_real = origin[1] - y_coor
    x_grid = int(x_real//g_size)
    y_grid = int(y_real//g_size)
    map[x_grid][y_grid] = 1

    return map

def plot_grid(map):
    cmap = colors.ListedColormap(['White','Black'])
    plt.figure(figsize=(10,10))
    plt.pcolor(map[::-1],cmap=cmap,edgecolors='k',linewidths=0)

    ax = plt.gca()
    ax.set_aspect('equal')

def create_map(g_size, x, y):
    max_x = max(x)
    max_y = max(y)
    min_x = min(x)
    min_y = min(y)

    origin = [min_x, max_y]
    dis_x = max_x - min_x
    grids_x = math.ceil(dis_x/g_size)
    dis_y = max_y - min_y
    grids_y = math.ceil(dis_y/g_size)

    map = np.zeros((grids_x, grids_y))

    return map, origin

def generate_gridmap(coordinates, x, y, g_size):
    map, origin = create_map(g_size, x, y)
    for coordinate in coordinates:
        map = check_grid(coordinate, g_size, map, origin)
    plot_grid(map.T)