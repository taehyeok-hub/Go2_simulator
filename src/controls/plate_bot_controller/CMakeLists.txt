cmake_minimum_required(VERSION 3.5)
project(plate_bot_controller)

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
)

find_package(catkin REQUIRED COMPONENTS
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(PkgConfig REQUIRED)
pkg_check_modules(pinocchio REQUIRED pinocchio)

find_package(gazebo REQUIRED)

if(CUDA_FOUND)
    add_definitions(-DUSE_CUDA)
    set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS};--expt-relaxed-constexpr")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS eigen_conversions geometry_msgs sensor_msgs tf tf_conversions
  DEPENDS gazebo_ros roscpp rospy std_msgs Boost pinocchio
)

link_directories(${GAZEBO_LIBRARY_DIRS})


include_directories(
  ${Boost_INCLUDE_DIR} 
  ${catkin_INCLUDE_DIRS} 
  ${GAZEBO_INCLUDE_DIRS} 
  ${PCL_LIBRARY_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
)

link_directories(
  ${pinocchio_LIBRARY_DIRS}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  include/plate_bot_controller.h
  src/plate_bot_controller.cpp
)

set_property(TARGET plate_bot_controller PROPERTY CXX_STANDARD 14)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${pinocchio_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")


add_executable(plate_bot_node src/plate_bot_controller_node.cpp)
add_dependencies(plate_bot_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(plate_bot_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)


target_link_libraries(${PROJECT_NAME})

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  
  
)


