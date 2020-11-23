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
#include <periodic_slam/CameraMeasurement.h>
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



void StereoISAM2::camCallback(const periodic_slam::CameraMeasurementPtr& camera_msg){
     
        vector<periodic_slam::FeatureMeasurement> feature_vector = camera_msg->features;
        double timestep = camera_msg->header.stamp.toSec();

 
        // Set Noise Models for Camera Factors
        auto gaussian = noiseModel::Isotropic::Sigma(3, 30.0);
        auto huber = noiseModel::Robust::Create(
            noiseModel::mEstimator::Huber::Create(1.345), gaussian);
        noiseModel::Isotropic::shared_ptr pose_landmark_noise = noiseModel::Isotropic::Sigma(3, 30.0); // one pixel in u and v
        gtsam::Cal3_S2Stereo::shared_ptr K{new gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline)};
        std::vector<pair<GenericStereoFactor<Pose3, Point3>,int>> factorMap;

        
        if (camera_msg->section.data == 2 && landmarkOffsets[3] != -1 ){
            return;
        }

            
        if (camera_msg->section.data != -1){
            for (int i = 0; i < feature_vector.size(); i++){
                periodic_slam::FeatureMeasurement feature = feature_vector[i];
            
                if (landmarkOffsets[camera_msg->section.data-1] == -1){
                    landmarkOffsets[camera_msg->section.data-1] = feature.id;
                }
                int landmark_id = feature.id-landmarkOffsets[camera_msg->section.data-1]+((camera_msg->section.data)*200);
                double uL = feature.u0;
                double uR = feature.u1;
                double v = feature.v0;

                
                // if (phase < .6 && phase > -.6){
                //     uL += distributionVO(generator);
                //     uR += distributionVO(generator);
                //     v += distributionVO(generator);
                //     //break;
                //     pose_landmark_noise = noiseModel::Isotropic::Sigma(3, 30.0); // one pixel in u and v
                // } 
                
                double d = uL - uR;
                double z = fx*baseline/d;
                double x = (uL-cx)*z/fx;
                double y = (v-cy)*z/fy;
                
                Point3 camera_point = Point3(x,y,z); 
                Point3 body_point = bodyToSensor.transformFrom(camera_point);
                Point3 world_point = currPose.transformFrom(body_point) ;

                 
                
            
                if (!currentEstimate.exists(L(landmark_id))) {
                    pcl::PointXYZRGB pcl_world_point = pcl::PointXYZRGB(200,100,0);
                    if (camera_msg->section.data == 1){
                        pcl_world_point = pcl::PointXYZRGB(200,0,100);
                    } else if (camera_msg->section.data == 2){
                        pcl_world_point = pcl::PointXYZRGB(100,0,200);
                    }

                    
                    pcl_world_point.x = world_point.x();
                    pcl_world_point.y = world_point.y();
                    pcl_world_point.z = world_point.z(); 
                    landmark_cloud_msg_ptr->points.push_back(pcl_world_point); 

                    initialEstimate.insert(L(landmark_id), world_point);
                }
                
                
                GenericStereoFactor<Pose3, Point3> visualFactor(StereoPoint2(uL, uR, v), 
                huber, X(frame), L(landmark_id), K,bodyToSensor);


                //graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(visualFactor);

                factorMap.push_back(pair<GenericStereoFactor<Pose3, Point3>,int>(visualFactor,landmark_id));
            
                // Removing this causes greater accuracy but earlier gtsam::IndeterminantLinearSystemException)
                //Add prior to the landmark as well    
                //graph.emplace_shared<PriorFactor<Point3> >(L(landmark_id), world_point, prior_landmark_noise);
            }
        } 
        

        
        

        
        
     
        sendTfs();
        
        if (frame > 0){
            // if (imu_times.size() < 5){
            //     return;
            // }
          initialEstimate.insert(X(frame), currPose);
          initialEstimate.insert(V(frame), currVelocity);
          //initialEstimate.insert(B(0), currBias);

        //   Vector6 covvec;
        //   covvec << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1;
        //   auto cov = noiseModel::Diagonal::Variances(covvec);
        //   auto f = boost::make_shared<BetweenFactor<imuBias::ConstantBias> >(
        //       B(frame-1), B(frame), imuBias::ConstantBias(), cov);
        //   graph.add(f);
        //   graph2.add(f);

          //Add Imu Factor
            // if (camera_msg->section.data == 2){
                ImuFactor imufac = create_imu_factor(timestep);
                graph.add(imufac);

            
            

        }

        // cout << "feature size " << feature_vector.size() << endl;
        // cout << "graph size" << graph.size() << endl;
 
        ISAM2Result result = isam.update(graph, initialEstimate);
  
        // printf("Error: %f\n", *(result.errorAfter));
       

        
      
    

        currentEstimate = isam.calculateEstimate();
        currPose = currentEstimate.at<Pose3>(X(frame));
        currVelocity = currentEstimate.at<Vector3>(V(frame));
        currBias = currentEstimate.at<imuBias::ConstantBias>(B(0));


        // for (int i = 0; i < factorMap.size() ; i ++){
        //     pair<GenericStereoFactor<Pose3, Point3>,int> p = factorMap.at(i);
        //     GenericStereoFactor<Pose3, Point3> visFact = p.first;
        //     int landmarkId = p.second;
        //     Vector e = visFact.evaluateError(currentEstimate.at<Pose3>(X(frame)),currentEstimate.at<Point3>(L(landmarkId)));
    
        //     // cout << "state: " << frame << " landmark: " <<  p.second << endl;
        //     // cout << e << endl;            

        // }
        graphError = graph.error(currentEstimate);

        //cout << graphError << endl;
    
        // cout << "\n" << endl;
        // cout << "\n" << endl;

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
    gtPose = Pose3(Rot3::Quaternion(msgs.pose[7].orientation.w, msgs.pose[7].orientation.x, msgs.pose[7].orientation.y,msgs.pose[7].orientation.z), Point3(msgs.pose[7].position.x, msgs.pose[7].position.y, msgs.pose[7].position.z));

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
    landmark_cloud_msg_ptr->header.frame_id = "world";
    landmark_cloud_msg_ptr->height = 1;
    landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
    point_pub.publish(landmark_cloud_msg_ptr);
    //landmark_cloud_msg_ptr->clear();


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
