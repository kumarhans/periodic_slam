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

    landmark = 0;
    frame = 0;
 
    // Pose prior - at identity
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
    graph.add(PriorFactor<Pose3>(X(0), currPose, priorPoseNoise));
 
}

 
 
void StereoISAM2::imuCallback(const sensor_msgs::Imu &imu_msg){
    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;
    cout << lA.x << endl;
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


// /* ----------------------------------------------------------------------------
//  * GTSAM Copyright 2010, Georgia Tech Research Corporation,
//  * Atlanta, Georgia 30332-0415
//  * All Rights Reserved
//  * Authors: Frank Dellaert, et al. (see THANKS for the full author list)
//  * See LICENSE for the license information
//  * -------------------------------------------------------------------------- */

// /**
//  * @file ImuFactorExample2
//  * @brief Test example for using GTSAM ImuFactor and ImuCombinedFactor with ISAM2.
//  * @author Robert Truax
//  */

// #include <gtsam/geometry/PinholeCamera.h>
// #include <gtsam/geometry/Cal3_S2.h>
// #include <gtsam/inference/Symbol.h>
// #include <gtsam/navigation/ImuBias.h>
// #include <gtsam/navigation/ImuFactor.h>
// #include <gtsam/navigation/Scenario.h>
// #include <gtsam/nonlinear/ISAM2.h>
// #include <gtsam/slam/BetweenFactor.h>

// #include <vector>

// using namespace std;
// using namespace gtsam;

// // Shorthand for velocity and pose variables
// using symbol_shorthand::V;
// using symbol_shorthand::X;

// const double kGravity = 9.81;

// /* ************************************************************************* */
// int main(int argc, char* argv[]) {
//   auto params = PreintegrationParams::MakeSharedU(kGravity);
//   params->setAccelerometerCovariance(I_3x3 * 0.1);
//   params->setGyroscopeCovariance(I_3x3 * 0.1);
//   params->setIntegrationCovariance(I_3x3 * 0.1);
//   params->setUse2ndOrderCoriolis(false);
//   params->setOmegaCoriolis(Vector3(0, 0, 0));

//   Pose3 delta(Rot3::Rodrigues(-0.1, 0.2, 0.25), Point3(0.05, -0.10, 0.20));

//   // Start with a camera on x-axis looking at origin
//   double radius = 30;
//   const Point3 up(0, 0, 1), target(0, 0, 0);
//   const Point3 position(radius, 0, 0);
//   const auto camera = PinholeCamera<Cal3_S2>::Lookat(position, target, up);
//   const auto pose_0 = camera.pose();

//   // Now, create a constant-twist scenario that makes the camera orbit the
//   // origin
//   double angular_velocity = M_PI,  // rad/sec
//       delta_t = 1.0 / 18;          // makes for 10 degrees per step
//   Vector3 angular_velocity_vector(0, -angular_velocity, 0);
//   Vector3 linear_velocity_vector(radius * angular_velocity, 0, 0);
//   auto scenario = ConstantTwistScenario(angular_velocity_vector,
//                                         linear_velocity_vector, pose_0);

//   // Create a factor graph
//   NonlinearFactorGraph newgraph;

//   // Create (incremental) ISAM2 solver
//   ISAM2 isam;

//   // Create the initial estimate to the solution
//   // Intentionally initialize the variables off from the ground truth
//   Values initialEstimate, totalEstimate, result;

//   // Add a prior on pose x0. This indirectly specifies where the origin is.
//   // 0.1 rad std on roll, pitch, yaw, 30cm std on x,y,z.
//   auto noise = noiseModel::Diagonal::Sigmas(
//       (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.3)).finished());
//   newgraph.addPrior(X(0), pose_0, noise);

//   // Add imu priors
//   Key biasKey = Symbol('b', 0);
//   auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
//   newgraph.addPrior(biasKey, imuBias::ConstantBias(), biasnoise);
//   initialEstimate.insert(biasKey, imuBias::ConstantBias());
//   auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));

//   Vector n_velocity(3);
//   n_velocity << 0, angular_velocity * radius, 0;
//   newgraph.addPrior(V(0), n_velocity, velnoise);

//   initialEstimate.insert(V(0), n_velocity);

//   // IMU preintegrator
//   PreintegratedImuMeasurements accum(params);

//   // Simulate poses and imu measurements, adding them to the factor graph
//   for (size_t i = 0; i < 36; ++i) {
//     double t = i * delta_t;
//     if (i == 0) {  // First time add two poses
//       auto pose_1 = scenario.pose(delta_t);
//       initialEstimate.insert(X(0), pose_0.compose(delta));
//       initialEstimate.insert(X(1), pose_1.compose(delta));
//     } else if (i >= 2) {  // Add more poses as necessary
//       auto pose_i = scenario.pose(t);
//       initialEstimate.insert(X(i), pose_i.compose(delta));
//     }

//     if (i > 0) {
//       // Add Bias variables periodically
//       if (i % 5 == 0) {
//         biasKey++;
//         Symbol b1 = biasKey - 1;
//         Symbol b2 = biasKey;
//         Vector6 covvec;
//         covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
//         auto cov = noiseModel::Diagonal::Variances(covvec);
//         auto f = boost::make_shared<BetweenFactor<imuBias::ConstantBias> >(
//             b1, b2, imuBias::ConstantBias(), cov);
//         newgraph.add(f);
//         initialEstimate.insert(biasKey, imuBias::ConstantBias());
//       }
//       // Predict acceleration and gyro measurements in (actual) body frame
//       Vector3 measuredAcc = scenario.acceleration_b(t) -
//                             scenario.rotation(t).transpose() * params->n_gravity;
//       Vector3 measuredOmega = scenario.omega_b(t);
//       accum.integrateMeasurement(measuredAcc, measuredOmega, delta_t);

//       // Add Imu Factor
//       ImuFactor imufac(X(i - 1), V(i - 1), X(i), V(i), biasKey, accum);
//       newgraph.add(imufac);

//       // insert new velocity, which is wrong
//       initialEstimate.insert(V(i), n_velocity);
//       accum.resetIntegration();
//     }

//     // Incremental solution
//     isam.update(newgraph, initialEstimate);
//     result = isam.calculateEstimate();
//     newgraph = NonlinearFactorGraph();
//     initialEstimate.clear();
//   }
//   GTSAM_PRINT(result);
//   return 0;
// }
// /* ************************************************************************* */
 
 