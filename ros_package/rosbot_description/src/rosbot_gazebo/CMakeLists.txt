cmake_minimum_required(VERSION 2.8.3)
project(rosbot_gazebo)

find_package(catkin REQUIRED)

catkin_package(
)

install(DIRECTORY launch
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
## Add your launch file
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)
