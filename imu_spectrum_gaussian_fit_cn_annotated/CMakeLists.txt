cmake_minimum_required(VERSION 3.0.2)
project(imu_spectrum_gaussian_fit)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFTW3 REQUIRED fftw3)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
  ${FFTW3_INCLUDE_DIRS}
)

add_executable(imu_gaussian_fit_node
  src/imu_gaussian_fit_node.cpp
)

target_link_libraries(imu_gaussian_fit_node
  ${catkin_LIBRARIES}
  ${CERES_LIBRARIES}
  ${FFTW3_LIBRARIES}
)

install(TARGETS imu_gaussian_fit_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config examples scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
