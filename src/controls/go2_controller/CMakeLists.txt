cmake_minimum_required(VERSION 3.5)
project(go2_controller)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 14) # Pinocchio 사용을 위해 권장

# 1. 패키지 찾기
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

set(ENV{PKG_CONFIG_PATH} "$ENV{PKG_CONFIG_PATH}:/home/pth/pth/model/Go2_simulator/src/controls/pinocchio/lib/pkgconfig")

pkg_check_modules(pinocchio REQUIRED pinocchio)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS eigen_conversions geometry_msgs sensor_msgs tf tf_conversions
  DEPENDS gazebo_ros roscpp rospy std_msgs Boost
)

# 2. 인클루드 경로 설정
set(PINOCCHIO_DIR /home/pth/pth/model/Go2_simulator/src/controls/pinocchio)
include_directories(
  include
  include/Tools
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${pinocchio_INCLUDE_DIRS}
  # ${lcm_INCLUDE_DIRS}
  # deploy/lcm_types
)

add_subdirectory(include/Tools)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${pinocchio_LIBRARY_DIRS}
)


# 3. 라이브러리 생성 및 링크
add_library(${PROJECT_NAME}
  include/go2_controller.h
  src/go2_controller.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${pinocchio_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  Tools
)

# 4. 실행 파일 생성 및 링크
add_executable(go2_node src/go2_controller_node.cpp)
add_dependencies(go2_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(go2_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${pinocchio_LIBRARIES}
)

install(DIRECTORY launch rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
