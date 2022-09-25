cmake_minimum_required(VERSION 3.0.2)
project(traffic_light_fetcher)


find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  nodes/${PROJECT_NAME}/tl_fetcher
  nodes/${PROJECT_NAME}/tl_analysis
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
