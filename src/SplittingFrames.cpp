//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

//Standard utilities
#include <iostream>

#include "SplittingFrames.h"
#include <gazebo_msgs/LinkStates.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <periodic_slam/PhaseFrames.h>
 
 
using namespace std;
using namespace gtsam;

bool start = false;


SplittingFrames::SplittingFrames(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle):nh_(nodehandle),it_(imagehandle){
    initializeSubsAndPubs();
    // angleHigh = 1000.0;
    // angleLow = -1000.0;
    angleHigh = vector<float> { -.5, .1, 10 };    
    angleLow = vector<float> { -10, -.1, .5 };
    count = 0;
    initializaing = true;
    imageIn = false;
}

void SplittingFrames::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");

    Leftsub.subscribe(it_,"/simulated/camera/left/image_raw", 1);
    Rightsub.subscribe(it_,"/simulated/camera/right/image_raw", 1);

    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), Leftsub, Rightsub);
    sync -> registerCallback(boost::bind(&SplittingFrames::stereoCallback, this, _1, _2 ));
    
    gazSUB = nh_.subscribe("/gazebo/link_states", 1000, &SplittingFrames::gazCallback,this);
    imuSub = nh_.subscribe("/camera_imu", 1000, &SplittingFrames::imuCallback, this);

    image_pub_left = it_.advertise("/simulated/camera/left/image_raw/1", 1);
    image_pub_right = it_.advertise("/simulated/camera/right/image_raw/1", 1);

    imu_pub = nh_.advertise<sensor_msgs::Imu>("/camera/imu_noisy", 1);

}

void SplittingFrames::stereoCallback(
    const sensor_msgs::ImageConstPtr& cam0_img,
    const sensor_msgs::ImageConstPtr& cam1_img) {

 
        currImageL = cam0_img;
        currImageR = cam1_img;

        for (int i = 0; i < angleHigh.size(); i ++){
          double u = angleHigh[i];
          double l = angleLow[i];
          if (start == false && currPitch < u && currPitch > l && i == 1){
            image_pub_left.publish(cam0_img);
            image_pub_right.publish(cam1_img);
          }
          else if (currPitch < u && currPitch > l && i == 2){
            start = true;
            image_pub_left.publish(cam0_img);
            image_pub_right.publish(cam1_img);
          } 
        }         
}


void SplittingFrames::imuCallback(const sensor_msgs::Imu &imu_msg){

    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;

    sensor_msgs::Imu imu_mess_new;

    lA.x += distributionIMU(generator);
    lA.y += distributionIMU(generator);
    lA.z += distributionIMU(generator);

    if (abs(lA.z) > 50){
      lA.z = 50*lA.z/lA.z;
    }

    imu_mess_new.angular_velocity = aV;
    imu_mess_new.linear_acceleration = lA;

    imu_mess_new.header = imu_msg.header;
    imu_pub.publish(imu_mess_new);
}


void SplittingFrames::gazCallback(const gazebo_msgs::LinkStates &msgs){
    currPitch = Rot3::Quaternion(msgs.pose[11].orientation.w, msgs.pose[11].orientation.x, msgs.pose[11].orientation.y,msgs.pose[11].orientation.z).pitch();   
}
    
int main(int argc, char **argv)
{
  //Iitialize ROS and Subscribers
  ros::init(argc, argv, "SplittingFrameser");
  ros::NodeHandle nh; 
  image_transport::ImageTransport it(nh);

  SplittingFrames grapher(nh, it);
  ros::Rate loop_rate(100);

  while (ros::ok()) {        
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

