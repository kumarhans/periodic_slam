cmake_minimum_required(VERSION 2.8.3)
project(feature_tracker)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
    cv_bridge
    camera_model
 
  rospy

  roslib

  image_transport
  random_numbers
  tf
  pcl_conversions
  pcl_ros
  gazebo_msgs

    )

find_package(OpenCV REQUIRED)
find_package(GTSAM REQUIRED QUIET)
find_package(GTSAMCMakeTools REQUIRED)


include_directories(
    ${catkin_INCLUDE_DIRS}
    ${GTSAM_INCLUDE_DIR}
  ${GTSAMCMakeTools_INCLUDE_DIRECTORIES}
    )



add_message_files(
  FILES
  FeatureMeasurement.msg
  CameraMeasurement.msg
  TrackingInfo.msg
  PhaseFrames.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
)


set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(feature_tracker
    src/feature_tracker_node.cpp
    src/parameters.cpp
    src/feature_tracker.cpp
    )

target_link_libraries(feature_tracker ${catkin_LIBRARIES} ${OpenCV_LIBS} gtsam)
