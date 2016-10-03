cmake_minimum_required(VERSION 2.8.3)
project(gazebo2rviz)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  pysdf
  gazebo_ros
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

find_package(Boost REQUIRED)

catkin_python_setup()

add_service_files(
  FILES
  GetSdf.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES gazebo2rviz
  CATKIN_DEPENDS roscpp rospy pysdf rospkg gazebo_ros std_msgs message_runtime
  DEPENDS gazebo
)

###########
## Build ##
###########
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)	

link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(get_sdf_service src/get_sdf_service.cpp)
target_link_libraries(get_sdf_service ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(get_sdf_service ${catkin_EXPORTED_TARGETS} ${${PROJECT_NAME}_EXPORTED_TARGETS})


#############
## Install ##
#############
install(PROGRAMS
  scripts/gazebo2tf_node.py
  scripts/gazebo2marker_node.py
  scripts/sdf2marker_node.py
  scripts/sdf2tfstatic_node.py
  scripts/sdf2extract_tfstatic_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS get_sdf_service
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
