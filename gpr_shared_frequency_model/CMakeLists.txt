cmake_minimum_required(VERSION 3.0.2)
project(gpr_shared_frequency_model)

add_compile_options(-std=c++14 -O2)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

add_executable(train_shared_gpr_node src/train_shared_gpr_node.cpp)
target_link_libraries(train_shared_gpr_node
  ${catkin_LIBRARIES}
  yaml-cpp
)

add_executable(predict_shared_gpr_node src/predict_shared_gpr_node.cpp)
target_link_libraries(predict_shared_gpr_node
  ${catkin_LIBRARIES}
  yaml-cpp
)
