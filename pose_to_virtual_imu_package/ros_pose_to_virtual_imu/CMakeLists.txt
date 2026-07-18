cmake_minimum_required(VERSION 3.0.2)
project(ros_pose_to_virtual_imu)

# C++11 即可。-O2 用于减少在线计算开销。
add_compile_options(-std=c++11 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(pose_to_virtual_imu_node src/pose_to_virtual_imu_node.cpp)
target_link_libraries(pose_to_virtual_imu_node ${catkin_LIBRARIES})
