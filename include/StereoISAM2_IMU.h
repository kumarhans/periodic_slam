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
#include <StereoISAM2.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <factor_graph/CameraMeasurement.h>
#include <pcl/common/centroid.h>
 

 
class StereoISAM2
{
public:
    
    StereoISAM2(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle);
    ~StereoISAM2();

    bool init;
    bool depthMap;
    bool visualize;

    cv::Mat prev_image_left;
    cv::Mat prev_image_right;

    cv::Mat curr_image_left;
    cv::Mat curr_image_right;

    cv::Mat features0;
    cv::Mat features1;
    
    cv::Mat debug_image;

    std::vector<cv::Point2f> currPoints;
    std::vector<cv::Point2f> prevPoints;
    std::vector<cv::KeyPoint> keypoints;

    cv::Mat disparity;
    cv::Mat Q;
    cv::Mat depth_map_prev;
    cv::Mat depth_map_curr;

    cv::Mat H_init;
    cv::Mat H_curr;
    cv::Mat pose;
    nav_msgs::Path path;
    gtsam::Pose3 currPose; 
    inline static gtsam::Pose3 gtPose;
    gtsam::Values currentEstimate;
  
 

    int landmark;
    int frame;
 
    
    inline static pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr{new pcl::PointCloud<pcl::PointXYZRGB>()};
   
     
  
    
private:

    void initializeSubsAndPubs();
    void initializeFactorGraph();
    void addFactors();
    void addVisualFactor(int frameNum, int landmarkNum, int ul, int ur, int v);

    void getInitialRot(double angleDown, double height);
    void visualizePoints(std::vector<cv::Point3f>& currWorldPoints, std::vector<cv::Point3f>& prevWorldPoints);
    gtsam::ISAM2 isam;

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    std::map<size_t, gtsam::SmartStereoProjectionPoseFactor::shared_ptr> smartFactors;
    //gtsam::Cal3_S2Stereo::shared_ptr K;

    

    ros::NodeHandle nh;
    image_transport::ImageTransport it;

    image_transport::SubscriberFilter left_sub;
    image_transport::SubscriberFilter right_sub;
    ros::Subscriber gtSUB;
    ros::Subscriber camSub;

    image_transport::Publisher debug_pub;
    ros::Publisher pose_pub;
    ros::Publisher path_pub;
    ros::Publisher vis_pub;
    ros::Publisher point_pub;


    void imageCallback(const sensor_msgs::ImageConstPtr& left_msg, const sensor_msgs::ImageConstPtr& right_msg);
    void camCallback(const factor_graph::CameraMeasurementPtr& camera_msg);
    void sendTfs();
    static void GTCallback(const geometry_msgs::Twist &msg);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> *sync;
     
};


#endif