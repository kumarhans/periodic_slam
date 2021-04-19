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
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include "parameters.h"
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <geometry_msgs/Twist.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <periodic_factor_graph/CameraMeasurement.h>
#include <pcl/common/centroid.h>
#include <sensor_msgs/Imu.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gazebo_msgs/LinkStates.h>
#include <gtsam_unstable/nonlinear/IncrementalFixedLagSmoother.h>

 

 
class StereoISAM2
{
public:
    
    StereoISAM2(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle);
    ~StereoISAM2();

    //Visualization Paramters
    bool visualize;    
    bool initialized;
    cv::Mat debug_image;
    nav_msgs::Path pathGT;
    nav_msgs::Path pathOPTI;

    //State of Robot
    gtsam::Pose3 priorPose; 
    gtsam::Vector3 priorVelocity;
    gtsam::imuBias::ConstantBias priorBias; 
    gtsam::NavState prev_state;
    gtsam::NavState prop_state;
    gtsam::imuBias::ConstantBias prev_bias;

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
    bool estimatorInit;
    bool accel_init_done;
    

    int lastUp;
    int lastDown;
    gtsam::Pose3 startUp;
    gtsam::Pose3 startDown;

    double begin;
    double graphError;
    std::vector<int> landmarkOffsets;
    std::vector<int> landmarkIDs;
    std::map<int, std::pair<double,double>> idMap;
    
    //Initilize GTSAM Variables
    gtsam::IncrementalFixedLagSmoother smootherISAM2;
    gtsam::FixedLagSmoother::KeyTimestampMap newTimestamps;
    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initialEstimate;
    gtsam::Values currentEstimate;

    //Initialize IMU Variables
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> IMUparams;
    std::shared_ptr<gtsam::PreintegrationType> preintegrated;
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
    void sendTfs(double timestep);
    void do_accel_init();
    void do_nominal_init();
    void pubTrackCount( int count);
    void pubTrackLength( double length);
    gtsam::CombinedImuFactor create_imu_factor(double updatetime);
    gtsam::Point3 triangulateFeature(periodic_factor_graph::FeatureMeasurement feature);
    boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> imuParams();
     
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
    ros::Publisher pub_track_length;
    ros::Publisher pub_track;
    


    //Ros Callbacks
    void camCallback(const periodic_factor_graph::CameraMeasurementPtr& camera_msg);
    void imuCallback(const sensor_msgs::Imu &imu_msg);
    void gazCallback(const gazebo_msgs::LinkStates &msgs);
     
     
};


#endif