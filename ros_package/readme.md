Hi, this is the readme for running the ROS packages of this project.

## Requirements

We assume you are running ros melodic on ubuntu 18 and know the basics of ROS.
We use the following packages for python 2.7
- numpy
- scipy
- matplotlib
- PIL


## Getting started

Because there are unrelated files in this repository you need a seperate catkin workspace for this project. You can create one with the following commands. Please look at the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) if you are not familiar with catkin workspaces.

The workspace should be in another folder to the ITP_project you just have presumably already cloned.

We will create a link to the ITP_project/ros_package/rosbot_description folder in the src folder of the catkin workspace. To do this do the following:

- Create a catkin workspace (if you haven't already)
    - ``mkdir -p ~/catkin_ws/src``
    - ``cd ~/catkin_ws/``
    - ``catkin_make``
- Clone the repo in a <b>different</b> folder of your choice or pull the newest version
    - ``cd ..``
    - ``git clone \<repo link>
- Create a link to the rosbot_description folder. 
    - ``ln -s <path to ITP_project_git>/ros_package/rosbot_description <path to ros workspace>/src/``
    - for example ``ln -s ~/ITP_project/ros_package/rosbot_description ~/catkin_ws/src/``
    - Navigate to your catkin workspace and build it with ``catkin_make``

Now you should have a catkin workspace with a link to the rosbot_description folder. You can now build the workspace with .  

## Running the simulation
Because of the way the files are structured you need to change the .world file that refers to the 3D model of your world. In our example this is atlas.world. To do this go to:

 ```<ITP-repo-path>/ros_package/rosbot_description/src/rosbot_gazebo/worlds/atlas.world```

  and change the <b> TWO </b> lines 
``<uri>/home/pim/ros_workspace/src/rosbot_description/src/rosbot_navigation/maps/atlas_reframed.stl</uri>`` to your own path. 
- Open a new terminal 
- source the terminal as ``source ~/<ros_workspace_name>/devel/setup.sh 
`` for example ``source ~/catkin_ws/devel/setup.sh``
- run ``roslaunch rosbot_description rosbot_rviz.launch`` 
- Open a new terminal
- source again as in previous step
- run ``roslaunch rosbot_navigation amcl_and_path.launch``

## How to run map_delta.py

This function calculates the differences between the given map and the laserscan. It only calculates object that are on the laserscan but not on the map, it may  be useful to change this in the future. 

The function can be run as following:
- Run the simulation as above.
- Start a new terminal and source it (``source ~/<ros_workspace_name>/devel/setup.sh``)
- In the terminal type ``rosrun rosbot_navigation map_delta.py`` 

### map_delta.py options
map_delta.py has commands build in that can help with debugging or faster running. An example of running these commands:

``rosrun rosbot_navigation map_delta.py --closeness_threshold 0.2 -- large_map True``

All commands that can be used with map_delta:


| Argument              | Default | Input type | Description                                                                                                 |
|-----------------------|---------|------------|-------------------------------------------------------------------------------------------------------------|
|large_map           | False   | Bool       | If true this will clip the map to the max size of the laser scan, useful for large maps                     |
|plot_transformed    | False   | Bool       | Plot the laserscan and the transformed laserscan (VERY SLOW! only for debugging)                            |
|plot_delta          | False   | Bool       | Plot the map and the laserscan with the points that are not on the map (VERY SLOW! only for debugging)      |
|closeness_threshold   | 0.1     | float      | How close a laserscan point must be to any point on the given map to be considered part of the map |

NOTE: these values can also be changed in the code itself.

If you wish to look at or make changes to map_delta.py, simply naviage to: 
    
    ``<ITP-repo-path>/ros_package/rosbot_description/src/rosbot_navigation/map_delta.py``
