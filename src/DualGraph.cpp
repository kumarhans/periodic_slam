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
 
 
using namespace std;

DualGraph::DualGraph(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle):nh_(nodehandle),it_(imagehandle){
    initializeSubsAndPubs();
    // angleHigh = 1000.0;
    // angleLow = -1000.0;
    angleHigh = 1.0;
    angleLow = .98;
    count = 0;
    initializaing = true;
    imageIn = false;
}

void DualGraph::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");

    Leftsub = it_.subscribe("/left_r200/camera/color/image_raw", 0, &DualGraph::imageCallbackLeft,this);
    Rightsub = it_.subscribe("/right_r200/camera/color/image_raw", 0, &DualGraph::imageCallbackRight,this);
    gtSUB = nh_.subscribe("/cmd_phase", 1000, &DualGraph::GTCallback,this);
    poseSUB = nh_.subscribe("/cmd_pos", 100, &DualGraph::poseCallback,this);

    leftPubUp = it_.advertise("/DualFactor/up/leftImage", 1);
    rightPubUp = it_.advertise("/DualFactor/up/rightImage", 1);
    leftPubDown = it_.advertise("/DualFactor/down/leftImage", 1);
    rightPubDown = it_.advertise("/DualFactor/down/rightImage", 1);
}

void DualGraph::imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg) {
    currImageL = msg;
        //Because we setup right subscriber second, we assume that if this is getting images left one is too (Bad Assumption)
    if (!imageIn){
        try {
        cv_ptr = cv_bridge::toCvShare(msg, "mono8")->image;
        cout << cv_ptr.empty()<< endl;
        } catch (cv_bridge::Exception& e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            imageIn = false;
            return;
        }
        imageIn = true;
    }  
}

void DualGraph::imageCallbackRight(const sensor_msgs::ImageConstPtr &msg) {
    currImageR = msg;
}

void DualGraph::poseCallback(const geometry_msgs::Twist &msg){
    static tf::TransformBroadcaster br;
    GtTransform.setOrigin( tf::Vector3(msg.linear.x,msg.linear.y, msg.linear.z-1.25) );
    q.setRPY(msg.angular.x, msg.angular.y, msg.angular.z);
    GtTransform.setRotation(q);
    br.sendTransform(tf::StampedTransform(GtTransform, ros::Time::now(), "map", "ground_truth"));
}

void DualGraph::GTCallback(const std_msgs::Float64 &msg) {
    currPitch = msg.data;
    count += 1;
    
    if (currPitch != 0.0){
        DualGraph::initializaing = false;
    }            
    if (((currPitch > - angleHigh && currPitch < -angleLow) || DualGraph::initializaing) && imageIn && (count % 30 == 0)) {
        //cout << currPitch << endl;
        leftPubUp.publish(currImageL);
        rightPubUp.publish(currImageR);
    } else if (((currPitch > angleLow && currPitch < angleHigh) || DualGraph::initializaing) && imageIn && (count % 30 == 0)){
        leftPubDown.publish(currImageL);
        rightPubDown.publish(currImageR);
    }
        
}

 
