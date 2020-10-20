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
#include <gazebo_msgs/LinkStates.h>

#include <message_filters/subscriber.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>



 
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
    ros::Subscriber gazSUB;
    image_transport::SubscriberFilter Leftsub;
    image_transport::SubscriberFilter Rightsub;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;


    ros::Subscriber poseSUB;
    image_transport::Publisher leftPubUp;
    image_transport::Publisher rightPubUp;
    image_transport::Publisher leftPubDown;
    image_transport::Publisher rightPubDown;

    void stereoCallback(const sensor_msgs::ImageConstPtr& cam0_img, const sensor_msgs::ImageConstPtr& cam1_img);

    void poseCallback(const geometry_msgs::Twist &msg);

    void gazCallback(const gazebo_msgs::LinkStates &msgs);

};


#endif