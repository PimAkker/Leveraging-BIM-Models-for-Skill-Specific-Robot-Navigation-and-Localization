execute_process(COMMAND "~/ITP_project/ros_workspace/build/rosbot_description/src/rosbot_description/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(~/ITP_project/ros_workspace/build/rosbot_description/src/rosbot_description/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
