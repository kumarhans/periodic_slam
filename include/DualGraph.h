#ifndef DUALG_h
#define DUALG_h

//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf/transform_broadcaster.h>


 
class DualGraph
{
public:
    
    DualGraph(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle);
    void initializeSubsAndPubs();


private:

    float currPitch;
    sensor_msgs::ImageConstPtr currImageL;
    sensor_msgs::ImageConstPtr currImageR;
    float angleHigh;
    float angleLow;
    int count;
    
    tf::Transform GtTransform;
    tf::Quaternion q;

    bool initializaing;
    bool imageIn;
    cv::Mat cv_ptr;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber Leftsub;
    image_transport::Subscriber Rightsub;
    ros::Subscriber gtSUB;
    ros::Subscriber poseSUB;
    image_transport::Publisher leftPubUp;
    image_transport::Publisher rightPubUp;
    image_transport::Publisher leftPubDown;
    image_transport::Publisher rightPubDown;

    void imageCallbackLeft(const sensor_msgs::ImageConstPtr &msg);

    void imageCallbackRight(const sensor_msgs::ImageConstPtr &msg);

    void poseCallback(const geometry_msgs::Twist &msg);

    void GTCallback(const std_msgs::Float64 &msg);

};


#endif