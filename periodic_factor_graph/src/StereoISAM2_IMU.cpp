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
#include <StereoISAM2.h>
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


#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Imu.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <periodic_factor_graph/CameraMeasurement.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gazebo_msgs/LinkStates.h>


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
}

StereoISAM2::~StereoISAM2 () {}


void StereoISAM2::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");

    debug_pub = it.advertise("/ros_stereo_odo/debug_image", 1);
    point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("landmark_point_cloud", 10);
    pathOPTI_pub = nh.advertise<nav_msgs::Path>("/vo/pathOPTI", 1);
    pathGT_pub = nh.advertise<nav_msgs::Path>("/vo/pathGT", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/vo/pose", 1);

    gazSub = nh.subscribe("/gazebo/link_states", 1000, &StereoISAM2::gazCallback, this);
    ros::Duration(0.5).sleep();
    imuSub = nh.subscribe("/camera_imu", 1000, &StereoISAM2::imuCallback, this);
    camSub = nh.subscribe("features", 1000, &StereoISAM2::camCallback, this);



}



void StereoISAM2::initializeFactorGraph(){
    ROS_INFO("Initializing Factor Graph");

    //SET ISAM2 PARAMS
    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.1;
    parameters.evaluateNonlinearError = true;
    ISAM2 isam(parameters);

    //Set IMU PARAMS
    kGravity = 9.81; //simulation imu does not record gravity 
    IMUparams = PreintegrationParams::MakeSharedU(kGravity);
    IMUparams->setAccelerometerCovariance(I_3x3 * 0.5);
    IMUparams->setGyroscopeCovariance(I_3x3 * 0.1);
    IMUparams->setIntegrationCovariance(I_3x3 * 0.5);
    IMUparams->setUse2ndOrderCoriolis(false);
    IMUparams->setOmegaCoriolis(Vector3(0, 0, 0));
    accum = PreintegratedImuMeasurements(IMUparams);

    //Start Counters
    frame = 0;
    loopKeyDown = -1;
    loopKeyUp = -1;
    landmarkOffsets.push_back(-1);
    landmarkOffsets.push_back(-1);
    landmarkOffsets.push_back(-1);
    currPose = Pose3(Rot3::Ypr(initYaw,initPitch,initRoll), Point3(initX,initY,initZ));
    graphError = 0.0;
    landmark_cloud_msg_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.1), Vector3::Constant(0.1)).finished());
    graph.add(PriorFactor<Pose3>(X(0), currPose, priorPoseNoise));
    initialEstimate.insert(X(0), currPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.1, 0.1, 0.1));
    graph.add(PriorFactor<Vector3>(V(0), currVelocity, velnoise));
    initialEstimate.insert(V(0), currVelocity);

    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), currBias, biasnoise);
    initialEstimate.insert(B(0), currBias);

}



void StereoISAM2::camCallback(const periodic_factor_graph::CameraMeasurementPtr& camera_msg){
     
        vector<periodic_factor_graph::FeatureMeasurement> feature_vector = camera_msg->features;
        double timestep = camera_msg->header.stamp.toSec();
        sendTfs();

 
        // Set Noise Models for Camera Factors
        auto gaussian = noiseModel::Isotropic::Sigma(3, 50.0);
        auto huber = noiseModel::Robust::Create(
            noiseModel::mEstimator::Huber::Create(1.345), gaussian);
        noiseModel::Isotropic::shared_ptr pose_landmark_noise = noiseModel::Isotropic::Sigma(3, 30.0); // one pixel in u and v
        gtsam::Cal3_S2Stereo::shared_ptr K{new gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline)};

        
        
        if (frame > 0){
          initialEstimate.insert(X(frame), currPose);
          initialEstimate.insert(V(frame), currVelocity);
          //initialEstimate.insert(B(0), currBias);

            //Add Imu Factor
            ImuFactor imufac = create_imu_factor(timestep);
            graph.add(imufac);
        }

 
        ISAM2Result result = isam.update(graph, initialEstimate);
  
 
        currentEstimate = isam.calculateEstimate();
        currPose = currentEstimate.at<Pose3>(X(frame));
        currVelocity = currentEstimate.at<Vector3>(V(frame));
        currBias = currentEstimate.at<imuBias::ConstantBias>(B(0));
        graphError = graph.error(currentEstimate);


        graph.resize(0);
        initialEstimate.clear();
        accum.resetIntegration();
        frame ++;
        
}




 
 
void StereoISAM2::imuCallback(const sensor_msgs::Imu &imu_msg){

    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;
    Vector3 measuredAcc(lA.x,lA.y,lA.z);
    Vector3 measuredOmega(aV.x,aV.y,aV.z);
    
    double timestep = imu_msg.header.stamp.toSec();
    imu_times.push_back(timestep);
    imu_linaccs.push_back(measuredAcc);
    imu_angvel.push_back(measuredOmega);   
}


ImuFactor StereoISAM2::create_imu_factor(double updatetime) {

    int imucompound = 0;
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            // Preintegrate this measurement!
            accum.integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt);
        }
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Preintegrate this measurement!
        accum.integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    ImuFactor imufac(X(frame - 1), V(frame - 1), X(frame), V(frame), B(0), accum); 
    return imufac;
}




void StereoISAM2::gazCallback(const gazebo_msgs::LinkStates &msgs){
    gtPose = Pose3(Rot3::Quaternion(msgs.pose[8].orientation.w, msgs.pose[8].orientation.x, msgs.pose[8].orientation.y,msgs.pose[8].orientation.z), Point3(msgs.pose[8].position.x, msgs.pose[8].position.y, msgs.pose[8].position.z));

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


    // Publish landmark PointCloud message (in world frame)
    landmark_cloud_msg_ptr->clear();
    landmark_cloud_msg_ptr->header.frame_id = "world";
    landmark_cloud_msg_ptr->height = 1;
    gtsam::Point3 point;
    for (const int i: landmarkIDs) {
        point = currentEstimate.at<Point3>(L(i));  
        pcl::PointXYZRGB pcl_world_point = pcl::PointXYZRGB(200,100,0);
        if (i < 2000){
            pcl_world_point = pcl::PointXYZRGB(200,0,100);
        } else if (i > 3000){
            pcl_world_point = pcl::PointXYZRGB(100,0,200);
        }

        pcl_world_point.x = point.x();
        pcl_world_point.y = point.y();
        pcl_world_point.z = point.z(); 
        landmark_cloud_msg_ptr->points.push_back(pcl_world_point);   
    }
    landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
    point_pub.publish(landmark_cloud_msg_ptr);
    landmark_cloud_msg_ptr->clear();


    //Publish GT Trajectory
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="/world";
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose.position.x =  gtPose.x();
    poseStamped.pose.position.y = gtPose.y();
    poseStamped.pose.position.z = gtPose.z();
    pathGT.header.frame_id = "world";
    pathGT.poses.push_back(poseStamped);
    pathGT.header.stamp = poseStamped.header.stamp;
    pathGT_pub.publish(pathGT);


    //Publish SLAM Trajectory
    gtsam::Pose3 pose;
    for (int i = 0; i < frame; i ++){
        pose = currentEstimate.at<Pose3>(X(i));   
        poseStamped.pose.position.x =  pose.x();
        poseStamped.pose.position.y = pose.y();
        poseStamped.pose.position.z = pose.z();
        pathOPTI.header.frame_id = "world";
        pathOPTI.poses.push_back(poseStamped);
        pathOPTI.header.stamp = poseStamped.header.stamp;
        pathOPTI_pub.publish(pathOPTI); 
    }
    
    //Publish Pose
    r = pose.rotation();
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    poseStamped.pose.orientation.x = q.x();
    poseStamped.pose.orientation.y = q.y();
    poseStamped.pose.orientation.z = q.z();
    poseStamped.pose.orientation.w = q.w();
    pose_pub.publish(poseStamped);
    pathOPTI.poses.clear();
    
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
