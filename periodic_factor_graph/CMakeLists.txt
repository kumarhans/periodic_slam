cmake_minimum_required(VERSION 2.8.3)
project(periodic_factor_graph)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  roslib
  cv_bridge
  image_transport
  random_numbers
  tf
  pcl_conversions
  pcl_ros
  gazebo_msgs
)

find_package(GTSAM REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAMCMakeTools REQUIRED)
set(BOOST_FIND_MINIMUM_VERSION 1.43)
set(BOOST_FIND_MINIMUM_COMPONENTS serialization system filesystem thread program_options date_time timer chrono regex )

##################
## ROS messages ##
##################
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
  INCLUDE_DIRS include
  CATKIN_DEPENDS 
  DEPENDS PCL
)

 

include_directories(
	include
	${catkin_INCLUDE_DIRS}
  ${EIGEN_3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
  ${GTSAMCMakeTools_INCLUDE_DIRECTORIES}
)
 


link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${GTSAM_LIBRARY_DIRS}
)

 
 


 
 
#add_executable(stereoISAM2 src/StereoISAM2.cpp src/parameters.cpp)
#target_link_libraries(stereoISAM2 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

#add_executable(stereoISAM2_IMU src/StereoISAM2_IMU.cpp src/parameters.cpp)
#target_link_libraries(stereoISAM2_IMU ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

#add_executable(dualGraph src/DualGraph.cpp src/parameters.cpp)
#target_link_libraries(dualGraph ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)

add_executable(stereoISAM2_IMU_VO src/StereoISAM2_IMU_VO.cpp src/parameters.cpp)
target_link_libraries(stereoISAM2_IMU_VO ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam gtsam_unstable)
 
add_executable(stereoISAM2_IMU_VO_BATCH src/StereoISAM2_IMU_VO_BATCH.cpp src/parameters.cpp)
target_link_libraries(stereoISAM2_IMU_VO_BATCH ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} gtsam gtsam_unstable)

 
 

 
 


 

 
