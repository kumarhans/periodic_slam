#ifndef STER_h
#define STER_h

//Some Ros Packages
#include <ros/package.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <sensor_msgs/Image.h>


#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
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

    //State of Robot
    gtsam::Pose3 currPose; 
    gtsam::Vector3 currVelocity;
    gtsam::imuBias::ConstantBias currBias;  
    gtsam::Pose3 gtPose = gtsam::Pose3();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptr;
    double phase;

    //Counters 
    int landmark;
    int frame;
    int bias;
    double prevAV;
    int loopKeyDown;
    int loopKeyUp;
    double begin;
    double graphError;
    std::vector<int> landmarkOffsets;
    
    //Initilize GTSAM Variables
    gtsam::ISAM2 isam;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::Values currentEstimate;

    //Initialize IMU Variables
    boost::shared_ptr<gtsam::PreintegrationParams> IMUparams;
    gtsam::PreintegratedImuMeasurements accum;
    double kGravity;
    std::deque<double> imu_times;
    std::deque<gtsam::Vector3> imu_linaccs;
    std::deque<gtsam::Vector3> imu_angvel;
    std::deque<gtsam::Vector3> imu_orientation;

    // Set up random noise generators
    std::default_random_engine generator;
    std::normal_distribution<double> distributionVO{0.0,0.0};
    std::normal_distribution<double> distributionIMU{0.0,0.0};
    


private:

    // Some Helper Functions
    void initializeSubsAndPubs();
    void initializeFactorGraph();
    void sendTfs();
    gtsam::ImuFactor create_imu_factor(double updatetime);
    gtsam::Point3 triangulateFeature(periodic_slam::FeatureMeasurement feature);
     
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
    void gazCallback(const gazebo_msgs::LinkStates &msgs);
     
     
};


#endif