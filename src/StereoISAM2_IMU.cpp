/**
 * @file ISAM2_SmartFactorStereo_IMU.cpp
 * @brief test of iSAM2 with smart stereo factors and IMU preintegration,
 * originally used to debug valgrind invalid reads with Eigen
 * @author Nghia Ho
 *
 * Setup is a stationary stereo camera with an IMU attached.
 * The data file is at examples/Data/ISAM2_SmartFactorStereo_IMU.txt
 * It contains 5 frames of stereo matches and IMU data.
 */

#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
#include <StereoISAM2_IMU.h>
//Libraries for Image Stuff
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"
 
#include "parameters.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <periodic_slam/CameraMeasurement.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>


#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

using namespace cv;
using namespace std;
using namespace gtsam;
using symbol_shorthand::X;
using symbol_shorthand::V;
using symbol_shorthand::B;
using symbol_shorthand::L;
 

 
StereoISAM2::StereoISAM2(ros::NodeHandle &nodehandle,image_transport::ImageTransport &imagehandle):nh(nodehandle),it(imagehandle){
    initializeSubsAndPubs();
    initializeFactorGraph();
    init = false;
}

StereoISAM2::~StereoISAM2 () {
    delete sync;
    for (auto i : smartFactors){
        i.second->~SmartStereoProjectionPoseFactor();
    }
}

 
Pose3 StereoISAM2::gtPose{
  []{
    Pose3 a;
    return a;
  }() // Call the lambda right away
};

pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoISAM2::landmark_cloud_msg_ptr{
  []{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr landmark_cloud_msg_ptrs(new pcl::PointCloud<pcl::PointXYZRGB>());
    return landmark_cloud_msg_ptrs;
  }() // Call the lambda right away
};




void StereoISAM2::GTCallback(const geometry_msgs::Twist &msg)
{
  gtPose = Pose3(Rot3::Ypr(msg.angular.z, msg.angular.y, msg.angular.x), Point3(msg.linear.x, msg.linear.y, msg.linear.z));
}

void StereoISAM2::sendTfs(){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    Point3 t;  
    Rot3 r;  

    //Send gtsam tf
    t = currPose.translation();
    r = currPose.rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "Optimized Pose"));

    //Send ground truth tf
    t = gtPose.translation();
    r = gtPose.rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "True Pose"));

    //Send camera tf
    t = bodyToSensor.translation();
    r = bodyToSensor.rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "True Pose", "Camera"));

}

void StereoISAM2::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");

    gtSUB = nh.subscribe("/cmd_pos", 1000, &StereoISAM2::GTCallback);
    imuSub = nh.subscribe("/camera_imu", 1000, &StereoISAM2::imuCallback, this);
 
    debug_pub = it.advertise("/ros_stereo_odo/debug_image", 1);
    point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("landmark_point_cloud", 10);
    path_pub = nh.advertise<nav_msgs::Path>("/vo/path", 1);
   
}

void StereoISAM2::initializeFactorGraph(){
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    ISAM2 isam(parameters);
    currPose = Pose3(Rot3::Ypr(initYaw,initPitch,initRoll), Point3(initX,initY,initZ));

    kGravity = 9.81;
    IMUparams = PreintegrationParams::MakeSharedU(kGravity);
    IMUparams->setAccelerometerCovariance(I_3x3 * 0.1);
    IMUparams->setGyroscopeCovariance(I_3x3 * 0.1);
    IMUparams->setIntegrationCovariance(I_3x3 * 0.1);
    IMUparams->setUse2ndOrderCoriolis(false);
    IMUparams->setOmegaCoriolis(Vector3(0, 0, 0));

    frame = 0;
    bias = 0;

 
    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
    graph.add(PriorFactor<Pose3>(X(0), currPose, priorPoseNoise));

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
    graph.add(PriorFactor<Vector3>(V(0), currVelocity, velnoise));

    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), currBias, biasnoise);
    initialEstimate.insert(B(frame), currBias);

    accum = PreintegratedImuMeasurements(IMUparams);


 
}

 
 
void StereoISAM2::imuCallback(const sensor_msgs::Imu &imu_msg){
    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;

    initialEstimate.insert(X(frame), currPose);
    initialEstimate.insert(V(frame), currVelocity);
    double dt = .01;

    if (frame > 0) {  
      if (frame % 5 == 0) {
        bias++;
        Vector6 covvec;
        covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        auto cov = noiseModel::Diagonal::Variances(covvec);
        auto f = boost::make_shared<BetweenFactor<imuBias::ConstantBias> >(
            B(bias-1), B(bias), imuBias::ConstantBias(), cov);
        graph.add(f);
        initialEstimate.insert(B(bias), imuBias::ConstantBias());
      }
      // Predict acceleration and gyro measurements in (actual) body frame
      Vector3 measuredAcc(lA.x,lA.y,lA.z);
      Vector3 measuredOmega(aV.x,aV.y,aV.z);
      accum.integrateMeasurement(measuredAcc, measuredOmega, dt);

      // Add Imu Factor
      ImuFactor imufac(X(frame - 1), V(frame - 1), X(frame), V(frame), B(bias), accum);
      graph.add(imufac);

      // insert new velocity, which is wrong
      accum.resetIntegration();
    }

    // Incremental solution
    isam.update(graph, initialEstimate);
    currentEstimate = isam.calculateEstimate();
    currPose = currentEstimate.at<Pose3>(X(frame));
    currVelocity = currentEstimate.at<Vector3>(V(frame));
    currBias = currentEstimate.at<imuBias::ConstantBias>(B(bias));
    cout << currPose << endl;
    graph.resize(0);
    sendTfs();
    //graph = NonlinearFactorGraph();
    initialEstimate.clear();
    frame ++;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_ros_isam2");
    ros::NodeHandle nh; 
    readParameters(nh);
    ros::Duration(0.5).sleep();

    image_transport::ImageTransport it(nh);
    
    
    StereoISAM2 node(nh, it);
    
    

    ros::Rate loop_rate(100);
    ros::spin();

    return 0;
}
