Hi, this is the readme for running the ROS packages of this project.

## requirements

We assume you are running ros melodic or ubuntu 18 and now the basics of ROS.

## getting started

Because there are unrelated files in this repository you need a seperate catkin workspace for this project. You can create one with the following commands. Please look at the [ROS tutorials](http://wiki.ros.org/ROS/Tutorials) if you are not familiar with catkin workspaces.

The workspace should be in another folder to the ITP_project you just have presumably already cloned.

We will create a link to the ITP_project/ros_package/rosbot_description folder in the src folder of the catkin workspace. To do this do the following:

- Create a catkin workspace (if you haven't already)
    - ``mkdir -p ~/catkin_ws/src``
    - ``cd ~/catkin_ws/``
    - ``catkin_make``
- Clone the repo in another folder of your choice or pull the newest version
    - ``cd ..``
    - ``git clone \<repo link>
- Create a link to the rosbot_description folder
    - ``ln -s <path to ITP_project_git>/ros_package/rosbot_description <path to ros workspace>/src/``
    - for example ``ln -s ~/ITP_project/ros_package/rosbot_description ~/catkin_ws/src/``
Now you should have a catkin workspace with a link to the rosbot_description folder. You can now build the workspace with ``catkin_make``.  

## running the simulation
Because of the way the files are structured you need to change the .world file that refers to the 3D model of your world. In our example this is atlas.world. To do this go to: ``/home/pim/ITP_project/ros_package/rosbot_description/src/rosbot_gazebo/worlds/atlas.world`` and change the lines 
``<uri>/home/pim/ros_workspace/src/rosbot_description/src/rosbot_navigation/maps/atlas_reframed.stl</uri>`` to your own path. 
- Open a new terminal 
- source the terminal as ``source ~/<ros_workspace_name>/devel/setup.sh 
`` for example ``source ~/catkin_ws/devel/setup.sh``
- run ``roslaunch rosbot_description rosbot_rviz.launch`` 
- Open a new terminal
- source again as in previous step
- run ``roslaunch rosbot_navigation amcl_and_path.launch``
# <---finish this later-->