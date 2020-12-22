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
#include <sensor_msgs/Imu.h>


 
class SplittingFrames
{
public:
    
    SplittingFrames(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle);
    void initializeSubsAndPubs();


private:

    float currPitch;
    sensor_msgs::ImageConstPtr currImageL;
    sensor_msgs::ImageConstPtr currImageR;
    std::vector<float> angleHigh;
    std::vector<float> angleLow;
    int count;
    
    tf::Transform GtTransform;
    tf::Quaternion q;

    bool initializaing;
    bool imageIn;
    cv::Mat cv_ptr;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    ros::Subscriber gazSUB;
    ros::Subscriber imuSub;
    image_transport::SubscriberFilter Leftsub;
    image_transport::SubscriberFilter Rightsub;
    image_transport::Publisher image_pub_left;
    image_transport::Publisher image_pub_right;
    
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;

    // Set up random noise generators
    std::default_random_engine generator;
    std::normal_distribution<double> distributionVO{0.0,0.0};
    std::normal_distribution<double> distributionIMU{0.0,1.0};


    ros::Subscriber poseSUB;
    ros::Publisher phaseFramePub;
    ros::Publisher imu_pub;


    void stereoCallback(const sensor_msgs::ImageConstPtr& cam0_img, const sensor_msgs::ImageConstPtr& cam1_img);

    void poseCallback(const geometry_msgs::Twist &msg);
    void imuCallback(const sensor_msgs::Imu &imu_msg);
    void gazCallback(const gazebo_msgs::LinkStates &msgs);

};


#endif