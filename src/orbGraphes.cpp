//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

//Standard utilities
#include <iostream>
#include <typeinfo>

#include <DualGraph.h>
 
using namespace std;
 

//Need Higher Pitch Rate 


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