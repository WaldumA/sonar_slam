cmake_minimum_required(VERSION 2.8.3)
project(sonar_slam) 

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
   eigen_conversions
   nav_msgs
   roscpp
   sensor_msgs
   std_msgs
   tf2
   tf2_geometry_msgs
   tf2_ros
   vortex_msgs 
   message_generation
   sonar_msgs
   
)

## Adding OPENCV
find_package(OpenCV REQUIRED)

## Adding EIGEN
find_package(Eigen3)
set(EIGEN_PACKAGE EIGEN3)
if(NOT EIGEN3_FOUND)
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})
  set(EIGEN_PACKAGE Eigen)
endif()

catkin_package(
   INCLUDE_DIRS
      include
   CATKIN_DEPENDS
      eigen_conversions
      nav_msgs
      roscpp
      sensor_msgs
      std_msgs
      tf2
      tf2_geometry_msgs
      tf2_ros
      vortex_msgs
      sonar_msgs
      
   DEPENDS
      ${EIGEN_PACKAGE}
)


include_directories(
 include

  PRIVATE

  PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
include_directories(SYSTEM ${Boost_INCLUDE_DIR} ${EIGEN_INCLUDE_DIRS})


add_library(search_lib src/search_algorithms.cpp)
add_library(feature_extractor_lib src/line_extractor.cpp src/search_algorithms.cpp src/uncertainty_calculating.cpp src/ekf_slam.cpp)

add_executable(sonar_slam_node src/sonar_slam.cpp)


target_link_libraries(sonar_slam_node PRIVATE ${catkin_LIBRARIES} ${EIGEN3_LIBRARIES} ${OpenCV_LIBRARIES} search_lib feature_extractor_lib)


