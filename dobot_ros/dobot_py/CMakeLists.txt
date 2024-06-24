cmake_minimum_required(VERSION 3.0.2)
project(dobot_py)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  genmsg
  geometry_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS roscpp rospy std_msgs message_runtime geometry_msgs

)

include_directories(

  ${catkin_INCLUDE_DIRS}
)

catkin_package()

catkin_install_python(
  PROGRAMS nodes/tk_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

