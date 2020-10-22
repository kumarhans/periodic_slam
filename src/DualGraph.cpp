//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>

//Standard utilities
#include <iostream>

#include "DualGraph.h"
#include <gazebo_msgs/LinkStates.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <periodic_slam/PhaseFrames.h>
 
 
using namespace std;
using namespace gtsam;

DualGraph::DualGraph(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle):nh_(nodehandle),it_(imagehandle){
    initializeSubsAndPubs();
    // angleHigh = 1000.0;
    // angleLow = -1000.0;
    angleHigh = 1.0;
    angleLow = .4;
    count = 0;
    initializaing = true;
    imageIn = false;
}

void DualGraph::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");

    Leftsub.subscribe(it_,"/left_r200/camera/color/image_raw", 1);
    Rightsub.subscribe(it_,"/right_r200/camera/color/image_raw", 1);

    sync = new message_filters::Synchronizer<MySyncPolicy> (MySyncPolicy(10), Leftsub, Rightsub);
    sync -> registerCallback(boost::bind(&DualGraph::stereoCallback, this, _1, _2 ));
    
    gazSUB = nh_.subscribe("/gazebo/link_states", 1000, &DualGraph::gazCallback,this);
    phaseFramePub = nh_.advertise<periodic_slam::PhaseFrames>("/StereoFrames", 1);

}

void DualGraph::stereoCallback(
    const sensor_msgs::ImageConstPtr& cam0_img,
    const sensor_msgs::ImageConstPtr& cam1_img) {
        currImageL = cam0_img;
        currImageR = cam1_img;

        periodic_slam::PhaseFrames frame;
        frame.imageLeft = *cam0_img;
        frame.imageRight = *cam1_img;
        frame.pitch = currPitch;

        phaseFramePub.publish(frame);

       


        //cout << "got imgae" << endl;

}


void DualGraph::gazCallback(const gazebo_msgs::LinkStates &msgs){
    currPitch = Rot3::Quaternion(msgs.pose[7].orientation.w, msgs.pose[7].orientation.x, msgs.pose[7].orientation.y,msgs.pose[7].orientation.z).pitch();   
}
    
int main(int argc, char **argv)
{
  //Iitialize ROS and Subscribers
  ros::init(argc, argv, "DualGrapher");
  ros::NodeHandle nh; 
  image_transport::ImageTransport it(nh);

  DualGraph grapher(nh, it);
  ros::Rate loop_rate(100);

  while (ros::ok()) {        
    ros::spinOnce();
    loop_rate.sleep();
  }


  return 0;
}

