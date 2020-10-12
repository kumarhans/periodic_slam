#ifndef STER_h
#define STER_h

//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include "opencv2/features2d/features2d.hpp"
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include "parameters.h"
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <periodic_slam/CameraMeasurement.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gazebo_msgs/LinkStates.h>

 

 
class StereoISAM2
{
public:
    
    StereoISAM2(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle);
    ~StereoISAM2();

    //Visualization Paramters
    bool visualize;    
    cv::Mat debug_image;
    nav_msgs::Path pathGT;
    nav_msgs::Path pathOPTI;
    static pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr;

    //Initialize State of Robot
    gtsam::Pose3 currPose; 
    gtsam::Vector3 currVelocity;
    gtsam::imuBias::ConstantBias currBias;  // assume zero initial bias
    static gtsam::Pose3 gtPose;
    int landmark;
    int frame;
    int bias;

    //Initilize GTSAM Variables
    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::Values currentEstimate;

    boost::shared_ptr<gtsam::PreintegrationParams> IMUparams;
    gtsam::PreintegratedImuMeasurements accum;
    double kGravity;

    
private:

    void initializeSubsAndPubs();
    void initializeFactorGraph();
     
    //Ros Subscribers
    ros::NodeHandle nh;
    image_transport::ImageTransport it;
    ros::Subscriber imuSub;
    ros::Subscriber gazSub;
    ros::Subscriber camSub;

    //Ros Publishers
    image_transport::Publisher debug_pub;
    ros::Publisher pose_pub;
    ros::Publisher pathOPTI_pub;
    ros::Publisher pathGT_pub;
    ros::Publisher point_pub;

    //Ros Callbacks
    void camCallback(const periodic_slam::CameraMeasurementPtr& camera_msg);
    void imuCallback(const sensor_msgs::Imu &imu_msg);
    void sendTfs();
    static void GTCallback(const geometry_msgs::Twist &msg);
    static void gazCallback(const gazebo_msgs::LinkStates &msgs);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;
     
};


#endif