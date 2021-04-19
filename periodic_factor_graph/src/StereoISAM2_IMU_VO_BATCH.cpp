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
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

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

#include <chrono> 
using namespace std::chrono; 
  




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

boost::shared_ptr<gtsam::PreintegratedCombinedMeasurements::Params> StereoISAM2::imuParams() {
  // We use the sensor specs to build the noise model for the IMU factor.
  double accel_noise_sigma = 1.0;
  double gyro_noise_sigma = 0.1;
  double accel_bias_rw_sigma = 0.01;
  double gyro_bias_rw_sigma = 0.001;
  Matrix33 measured_acc_cov = I_3x3 * pow(accel_noise_sigma, 2);
  Matrix33 measured_omega_cov = I_3x3 * pow(gyro_noise_sigma, 2);
  Matrix33 integration_error_cov =
      I_3x3 * .00001;  // error committed in integrating position from velocities
  Matrix33 bias_acc_cov = I_3x3 * pow(accel_bias_rw_sigma, 2);
  Matrix33 bias_omega_cov = I_3x3 * pow(gyro_bias_rw_sigma, 2);
  Matrix66 bias_acc_omega_int =
      I_6x6 * 1e-5;  // error in the bias used for preintegration

  auto p = PreintegratedCombinedMeasurements::Params::MakeSharedU(gravMag);
  // PreintegrationBase params:
  p->accelerometerCovariance =
      measured_acc_cov;  // acc white noise in continuous
  p->integrationCovariance =
      integration_error_cov;  // integration uncertainty continuous
  // should be using 2nd order integration
  // PreintegratedRotation params:
  p->gyroscopeCovariance =
      measured_omega_cov;  // gyro white noise in continuous
  // PreintegrationCombinedMeasurements params:
  p->biasAccCovariance = bias_acc_cov;      // acc bias in continuous
  p->biasOmegaCovariance = bias_omega_cov;  // gyro bias in continuous
  p->biasAccOmegaInt = bias_acc_omega_int;

  return p;
}


void StereoISAM2::initializeSubsAndPubs(){
    ROS_INFO("Initializing Subscribers and Publishers");

    point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("landmark_point_cloud", 10);
    pathOPTI_pub = nh.advertise<nav_msgs::Path>("vo/pathOPTI", 1);
    pathGT_pub = nh.advertise<nav_msgs::Path>("vo/pathGT", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vo/pose", 1);
    pub_track = nh.advertise<std_msgs::Int32>("track_count", 1000);
    pub_track_length = nh.advertise<std_msgs::Float32>("track_length", 1000);

    gazSub = nh.subscribe("/gazebo/link_states", 1000, &StereoISAM2::gazCallback, this);
    ros::Duration(0.5).sleep();
    //imuSub = nh.subscribe("/mobile_robot/camera_imu", 1000, &StereoISAM2::imuCallback, this);
    imuSub = nh.subscribe("/camera/imu", 1000, &StereoISAM2::imuCallback, this);
    camSub = nh.subscribe("/features", 1000, &StereoISAM2::camCallback, this);
    
}



void StereoISAM2::initializeFactorGraph(){
    ROS_INFO("Initializing Factor Graph");

    //SET ISAM2 PARAMS
    

    //Set IMU PARAMS
    IMUparams = imuParams();
    preintegrated = std::make_shared<gtsam::PreintegratedCombinedMeasurements>(IMUparams, priorBias);
    
    //Start Counters
    accel_init_done = false;
    estimatorInit = false;
    priorPose = Pose3(Rot3::Ypr(initYaw,initPitch,initRoll), Point3(initX,initY,initZ));
    graphError = 0.0;
    frame = 0;
    landmark_cloud_msg_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());    
    
}

void StereoISAM2::do_accel_init() {
    gtsam::Vector3 acc_avg;
     
    if (imu_times.size() < 30){
        return;
    } else {
        for (auto &accel : imu_linaccs) {
            acc_avg += accel;   
        }
        acc_avg /= imu_times.size();
    }
    cout << "Gravity-aligning with accel. vector:\n" << acc_avg << endl;
    gtsam::Vector3 gravity_vec;
    gravity_vec << 0.0, 0.0, gravMag;
    auto initial_att = gtsam::Rot3(
        Eigen::Quaterniond().setFromTwoVectors(acc_avg, gravity_vec));
    gtsam::Pose3 initial_pose_(initial_att, gtsam::Point3());
    cout <<  "Gravity vector after alignment: " << (initial_pose_ * acc_avg) << endl;
    priorPose = priorPose*initial_pose_;

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
    initialEstimate.insert(X(0), priorPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
    initialEstimate.insert(V(0), priorVelocity);
     
    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
    initialEstimate.insert(B(0), priorBias);
    
    prev_state = gtsam::NavState(priorPose, priorVelocity);
    prop_state = prev_state;
    prev_bias = priorBias;

    accel_init_done = true;
}


void StereoISAM2::do_nominal_init() {

    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), priorPose, priorPoseNoise));
    initialEstimate.insert(X(0), priorPose);

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), priorVelocity, velnoise));
    initialEstimate.insert(V(0), priorVelocity);
     
    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), priorBias, biasnoise);
    initialEstimate.insert(B(0), priorBias);
     
    prev_state = gtsam::NavState(priorPose, priorVelocity);
    prop_state = prev_state;
    prev_bias = priorBias;

    accel_init_done = true;
}


void StereoISAM2::pubTrackCount(const int count){
    std_msgs::Int32 msg;
    msg.data = count;
    pub_track.publish(msg); 
}

void StereoISAM2::pubTrackLength(const double length){
    std_msgs::Float32 msg;
    msg.data = length;
    pub_track_length.publish(msg); 
}


void StereoISAM2::camCallback(const periodic_factor_graph::CameraMeasurementPtr& camera_msg){
      
        vector<periodic_factor_graph::FeatureMeasurement> feature_vector = camera_msg->features;
        double timestep = camera_msg->header.stamp.toSec();
         
        if (!accel_init_done) {return;}
 
        auto gaussian = noiseModel::Isotropic::Sigma(3, 4.0);
        if (camera_msg->section.data == 2 && estimatorInit){
            auto gaussian = noiseModel::Isotropic::Sigma(3, 10.0);
        }

        auto huber = noiseModel::Robust::Create(
            noiseModel::mEstimator::GemanMcClure::Create(1.25), gaussian);


        gtsam::Cal3_S2Stereo::shared_ptr K{new gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline)};

        noiseModel::Isotropic::shared_ptr prior_landmark_noise = noiseModel::Isotropic::Sigma(3, 3000); 
        auto huberPrior = noiseModel::Robust::Create(
            noiseModel::mEstimator::Cauchy::Create(1.0), prior_landmark_noise);
        
        
        if (camera_msg->section.data > 3 || camera_msg->section.data == -1){
            return;
        }
         

        if (frame > 0){
          //Add Imu Factor
          CombinedImuFactor imufac = create_imu_factor(timestep);
          graph.add(imufac);

          prop_state = preintegrated->predict(prev_state, prev_bias);
          initialEstimate.insert(X(frame), prop_state.pose());
          initialEstimate.insert(V(frame), prop_state.v());
          initialEstimate.insert(B(frame), prev_bias);       
        }

     
   
        if (camera_msg->section.data != -1){

            if(std::find(estimators.begin(), estimators.end(), camera_msg->section.data) != estimators.end() || !estimatorInit){
                if (!estimatorInit && std::find(estimators.begin(), estimators.end(), camera_msg->section.data)!= estimators.end() ) {estimatorInit = true;}
                
                for (int i = 0; i < feature_vector.size(); i++){
                    
                    periodic_factor_graph::FeatureMeasurement feature = feature_vector[i];
                    
              
                    if ( (feature.u0 - feature.u1 ) > 20 || (feature.u0 - feature.u1 ) < 3 || (abs(feature.v0- feature.v1) > .4)){
                        continue;
                    }
                    Point3 world_point = triangulateFeature(feature);
                    int landmark_id = feature.id + (camera_msg->section.data)*100000;
                    if (std::find(landmarkIDs.begin(),landmarkIDs.end(),landmark_id) == landmarkIDs.end()){
                        initialEstimate.insert(L(landmark_id), world_point);
                        //graph.add(PriorFactor< Point3>(L(landmark_id), world_point, prior_landmark_noise));
                    }
                    landmarkIDs.push_back(landmark_id);
                    int feature_id = landmark_id;

                    

         
                    GenericStereoFactor<Pose3, Point3> visualFactor(StereoPoint2(feature.u0, feature.u1, feature.v0), 
                    huber, X(frame), L(landmark_id), K,bodyToSensor);

                    graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(visualFactor);
                    
                }
            }  
        } 
        
       
        

         
        if (frame == 300){
            cout << "started" << endl;
            LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
            currentEstimate = optimizer.optimize();
            sendTfs(timestep);
            currentEstimate.print("Final result:\n");
            cout << "sent" << endl;
        }

        
        prev_state = prop_state;
       
 
       
        preintegrated->resetIntegrationAndSetBias(prev_bias);
        frame ++;
         
        
        
        
}
 

Point3 StereoISAM2::triangulateFeature(periodic_factor_graph::FeatureMeasurement feature){ 

    double d = feature.u0 - feature.u1;
    double z = fx*baseline/d;
    double x = (feature.u0-cx)*z/fx;
    double y = (feature.v0-cy)*z/fy;
    Point3 camera_point = Point3(x,y,z); 
    Point3 body_point = bodyToSensor.transformFrom(camera_point);
    Point3 world_point = prop_state.pose().transformFrom(body_point);
    return world_point;
}
 
 
void StereoISAM2::imuCallback(const sensor_msgs::Imu &imu_msg){
 
    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;
    Vector3 measuredAcc(lA.x,lA.y,lA.z);
    Vector3 measuredOmega(aV.x,aV.y,aV.z);
    

    if (!accel_init_done){
        if (useGrav){
            do_accel_init();
        } else {
            do_nominal_init();
        }
    }
     
    double timestep = imu_msg.header.stamp.toSec();
    imu_times.push_back(timestep);
    imu_linaccs.push_back(measuredAcc);
    imu_angvel.push_back(measuredOmega); 
   
}


CombinedImuFactor StereoISAM2::create_imu_factor(double updatetime) {

    int imucompound = 0;
    while(imu_times.size() > 1 && imu_times.at(1) <= updatetime) {
        double dt = imu_times.at(1) - imu_times.at(0);
        if (dt >= 0) {
            // Preintegrate this measurement!
            preintegrated->integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt);
        }
        imu_angvel.erase(imu_angvel.begin());
        imu_linaccs.erase(imu_linaccs.begin());
        imu_times.erase(imu_times.begin());
        imucompound++;
    }
    double dt_f = updatetime - imu_times.at(0);
    if (dt_f > 0) {
        // Preintegrate this measurement!
        preintegrated->integrateMeasurement(imu_linaccs.at(0), imu_angvel.at(0), dt_f);
        imu_times.at(0) = updatetime;
        imucompound++;
    }
    auto preint_imu_combined =
          dynamic_cast<const PreintegratedCombinedMeasurements&>(
              *preintegrated);
    CombinedImuFactor imufac(X(frame - 1), V(frame - 1), X(frame),
                                V(frame), B(frame - 1), B(frame),
                                preint_imu_combined);
    return imufac;

}




void StereoISAM2::gazCallback(const gazebo_msgs::LinkStates &msgs){
    gtPose = Pose3(Rot3::Quaternion(msgs.pose[11].orientation.w, msgs.pose[11].orientation.x, msgs.pose[11].orientation.y,msgs.pose[11].orientation.z), Point3(msgs.pose[11].position.x, msgs.pose[11].position.y, msgs.pose[11].position.z));
}

void StereoISAM2::sendTfs(double timestep){
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    Point3 t;  
    Rot3 r;  

    //Send gtsam tf
    t = prev_state.pose().translation();
    r = prev_state.pose().rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "Body"));

    //Send ground truth tf
    t = gtPose.translation();
    r = gtPose.rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "True Pose"));
  
    //Send camera tf
    t = bodyToSensor.translation();
    r = bodyToSensor.rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "Body", "LCam"));
    transform.setOrigin(tf::Vector3(baseline,0,0));
    q.setRPY(0,0,0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "LCam", "RCam"));


    //Publish landmark PointCloud message (in world frame)
    landmark_cloud_msg_ptr->clear();
    landmark_cloud_msg_ptr->header.frame_id = "world";
    landmark_cloud_msg_ptr->height = 1;
    gtsam::Point3 point;
    gtsam::Point3 point2;
    for (const int i: landmarkIDs) {
        if (!currentEstimate.exists(L(i))) {continue;}
        else{
            point = currentEstimate.at<Point3>(L(i));  
            pcl::PointXYZRGB pcl_world_point = pcl::PointXYZRGB(200,100,0);
            if (i < 200000){
                pcl_world_point = pcl::PointXYZRGB(200,0,100);
            } else if (i > 300000){
                pcl_world_point = pcl::PointXYZRGB(100,0,200);
            }
 
            pcl_world_point.x = point.x();
            pcl_world_point.y = point.y();
            pcl_world_point.z = point.z();
            landmark_cloud_msg_ptr->points.push_back(pcl_world_point);
        }
    }

    landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
    point_pub.publish(landmark_cloud_msg_ptr);
 
     

    //Publish GT Trajectory
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="/world";
    poseStamped.header.stamp = ros::Time(timestep);
    poseStamped.pose.position.x =  gtPose.x();
    poseStamped.pose.position.y = gtPose.y();
    poseStamped.pose.position.z = gtPose.z();
    pathGT.header.frame_id = "world";
    pathGT.poses.push_back(poseStamped);
    pathGT.header.stamp = poseStamped.header.stamp;
    pathGT_pub.publish(pathGT);
 
    // gtsam::Pose3 pose = prev_state.pose(); 
    // poseStamped.pose.position.x =  pose.x();
    // poseStamped.pose.position.y = pose.y();
    // poseStamped.pose.position.z = pose.z();
    // pathOPTI.header.frame_id = "world";
    // pathOPTI.poses.push_back(poseStamped);
    // pathOPTI.header.stamp = poseStamped.header.stamp;
    // pathOPTI_pub.publish(pathOPTI); 

    //Publish SLAM Trajectory
    gtsam::Pose3 pose;
    for (int i = 0; i <= frame-1; i ++){
         
        //if (!currentEstimate.exists(X(i))) {continue;}
        pose = currentEstimate.at<Pose3>(X(i)); 
        cout << pose.x() << endl;  
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
    //pathOPTI.poses.clear();
    
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_ros_isam2");
    ros::NodeHandle nh("~"); 
    readParameters(nh);
    ros::Duration(0.5).sleep();

    image_transport::ImageTransport it(nh);
    StereoISAM2 node(nh, it);
 
    ros::Rate loop_rate(100);
    ros::spin();

    return 0;
}