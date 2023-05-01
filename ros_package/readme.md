Hi, this is the readme for running the ROS packages of this project.

## requirements

We assume you are running ros melodic or ubuntu 18 and now the basics of ROS.

## getting started

Because there are unrelated files in this repository you need a seperate catkin workspace for this project. You can create one with the following commands. Please look at the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) if you are not familiar with catkin workspaces.

The workspace should be in another folder to the ITP_project you just have presumably already cloned.

We will create a link to the ITP_project/ros_package/rosbot_description folder in the src folder of the catkin workspace. To do this do the following:

- create a catkin workspace
    - ``mkdir -p ~/catkin_ws/src``
    - ``cd ~/catkin_ws/``
    - ``catkin_make``
- clone the repo in another folder (for example)
    - ``cd ..``
    - ``git clone \<repo link>
- create a link to the rosbot_description folder
    - ``cd ~/catkin_ws/src``
    - ``ln -s \<path to ITP_project>/ros_package/rosbot_description \<path to ros workspace>/src/``

Now you should have a catkin workspace with a link to the rosbot_description folder. You can now build the workspace with ``catkin_make``.  

## running the simulation

# finish this later