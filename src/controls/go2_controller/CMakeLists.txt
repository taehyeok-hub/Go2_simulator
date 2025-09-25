cmake_minimum_required(VERSION 3.0.2)
project(go2_controller)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  std_msgs
  roscpp
  roslib
  rospy
  sensor_msgs
  urdf
  tf
  xacro
  tf_conversions
  gazebo_ros
  controller_manager
  eigen_conversions
)

find_package(PkgConfig REQUIRED)
find_package(gazebo REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS eigen_conversions geometry_msgs sensor_msgs tf tf_conversions
  DEPENDS gazebo_ros roscpp rospy std_msgs Boost
)

include_directories(
  include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME}
  include/go2_controller.h
  src/go2_controller.cpp
  src/Forward_Kinematics.cpp
  src/Inverse_Kinematics.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

add_executable(go2_node src/go2_controller_node.cpp)
add_dependencies(go2_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(go2_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
