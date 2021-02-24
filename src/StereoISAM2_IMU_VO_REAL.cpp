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

    point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("landmark_point_cloud", 10);
    pathOPTI_pub = nh.advertise<nav_msgs::Path>("vo/pathOPTI", 1);
    pathGT_pub = nh.advertise<nav_msgs::Path>("vo/pathGT", 1);
    pose_pub = nh.advertise<geometry_msgs::PoseStamped>("vo/pose", 1);
    pub_track = nh.advertise<std_msgs::Int32>("track_count", 1000);
    pub_track_length = nh.advertise<std_msgs::Float32>("track_length", 1000);

    gazSub = nh.subscribe("/gazebo/link_states", 1000, &StereoISAM2::gazCallback, this);
    ros::Duration(0.5).sleep();
    imuSub = nh.subscribe("/mobile_robot/camera_imu", 1000, &StereoISAM2::imuCallback, this);
    camSub = nh.subscribe("/features", 1000, &StereoISAM2::camCallback, this);
    
}



void StereoISAM2::initializeFactorGraph(){
    ROS_INFO("Initializing Factor Graph");

    //SET ISAM2 PARAMS
    ISAM2Params parameters;
    double lag = 5.0;
    parameters.relinearizeThreshold = 0.003; // Set the relin threshold to zero such that the batch estimate is recovered
    parameters.relinearizeSkip = 1; // Relinearize every time
    smootherISAM2 = IncrementalFixedLagSmoother(lag, parameters);


    //Set IMU PARAMS
    kGravity = 9.80; //simulation imu does not record gravity 
    IMUparams = PreintegrationParams::MakeSharedU(kGravity);
    IMUparams->setAccelerometerCovariance(I_3x3 * .1);
    IMUparams->setGyroscopeCovariance(I_3x3 * .03);
    IMUparams->setIntegrationCovariance(I_3x3 * .1);
    IMUparams->setUse2ndOrderCoriolis(true);
    IMUparams->setOmegaCoriolis(Vector3(0, 0, 0));
    accum = PreintegratedImuMeasurements(IMUparams);

    //Start Counters
    estimatorInit = false;
    currPose = Pose3(Rot3::Ypr(initYaw,initPitch,initRoll), Point3(initX,initY,initZ));
    graphError = 0.0;
    frame = 0;
    landmark_cloud_msg_ptr = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    double firstTimestep = ros::Time::now().toSec();
    // Pose prior 
    auto priorPoseNoise = noiseModel::Diagonal::Sigmas(
        (Vector(6) << Vector3::Constant(0.01), Vector3::Constant(0.01)).finished());
    graph.add(PriorFactor<Pose3>(X(0), currPose, priorPoseNoise));
    initialEstimate.insert(X(0), currPose);
    newTimestamps[X(0)] = firstTimestep;

    //Velocity Prior 
    auto velnoise = noiseModel::Diagonal::Sigmas(Vector3(0.01, 0.01, 0.01));
    graph.add(PriorFactor<Vector3>(V(0), currVelocity, velnoise));
    initialEstimate.insert(V(0), currVelocity);
    newTimestamps[V(0)] = firstTimestep;

    //Bias Prior
    auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
    graph.addPrior<imuBias::ConstantBias>(B(0), currBias, biasnoise);
    initialEstimate.insert(B(0), currBias);
    newTimestamps[B(0)] = firstTimestep;
 
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


void StereoISAM2::camCallback(const periodic_slam::CameraMeasurementPtr& camera_msg){
      
        vector<periodic_slam::FeatureMeasurement> feature_vector = camera_msg->features;
        double timestep = camera_msg->header.stamp.toSec();
        // cout << timestep << endl;
        

        // if (imu_times.size() < 2) {
        //     return;
        // }   

        // Set Noise Models for Camera Factors
        

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
            sendTfs(timestep);
            return;
        }
        // if (camera_msg->section.data > 3){
        //     return;
        // }
        std::vector<int> newLands;

        // cv::Mat out_img = 
        gtsam::Point3 point2;
        int matchCount = 0;
        int trackCount = 0;
        gtsam::Point3 samePoint(0.0, 0.0, 0.0);
        auto pointnoise = noiseModel::Diagonal::Sigmas(Vector3::Constant(0.0001));
        if (camera_msg->section.data != -1){

            if(std::find(estimators.begin(), estimators.end(), camera_msg->section.data) != estimators.end() || !estimatorInit){
                if (!estimatorInit && std::find(estimators.begin(), estimators.end(), camera_msg->section.data)!= estimators.end() ) {estimatorInit = true;}
                
                for (int i = 0; i < feature_vector.size(); i++){
                    

                    
                    periodic_slam::FeatureMeasurement feature = feature_vector[i];

                     
    
                    

                    int landmark_id = feature.id + (camera_msg->section.data)*100000;
                    if (std::find(landmarkIDs.begin(),landmarkIDs.end(),landmark_id) != landmarkIDs.end()){
                        trackCount ++;
                    }
                    landmarkIDs.push_back(landmark_id);
                    int feature_id = landmark_id;
                    if ( idMap.find(feature_id) == idMap.end()){
                        idMap[feature_id].first = timestep;
                    } else{
                        idMap[feature_id].second = timestep;
                    }

                    if ( (feature.u0 - feature.u1 ) > 20 || (feature.u0 - feature.u1 ) < 5 || (abs(feature.v0- feature.v1) > .2)){
                        continue;
                    }
                    Point3 world_point = triangulateFeature(feature);

        
                    if (!currentEstimate.exists(L(landmark_id))) {
                        
                        newLands.push_back(landmark_id);
                        initialEstimate.insert(L(landmark_id), world_point);
                        newTimestamps[L(landmark_id)] = timestep;
                        
                    } else {
                        newTimestamps[L(landmark_id)] = timestep;
                    }
                    
                    //cout << "vertDistance" << abs(feature.v0- feature.v1) << endl;
                    
                    GenericStereoFactor<Pose3, Point3> visualFactor(StereoPoint2(feature.u0, feature.u1, feature.v0), 
                    huber, X(frame), L(landmark_id), K,bodyToSensor);

                    graph.emplace_shared<GenericStereoFactor<Pose3, Point3> >(visualFactor);
                    graph.add(PriorFactor< Point3>(L(landmark_id), world_point, prior_landmark_noise   ));
                }
            } else{
                sendTfs(timestep);
                return;
            }
        } 
        //cout << "totalFeatss: " << idMap.size() << endl;
        int trackfeatCount = 0;
        double featureTime = 0.0;
        for (auto idPair: idMap){
            //cout << idPair.first << " " << idPair.second.first << " " << idPair.second.second << endl;
            if (idPair.second.second != 0.0){
                trackfeatCount ++;
                featureTime += abs(idPair.second.first-idPair.second.second); 
            }
        }
        double averageTime = featureTime/trackfeatCount;
        pubTrackCount(trackCount);
        pubTrackLength(averageTime);
        
        
         // Draw new features.
    // for (const auto& new_cam0_point : curr_cam0_points) {
    //   cv::Point2f pt0 = new_cam0_point.second;
    //   cv::Point2f pt1 = curr_cam1_points[new_cam0_point.first] +
    //     Point2f(img_width, 0.0);

    //   circle(out_img, pt0, 3, new_feature, -1);
    //   circle(out_img, pt1, 3, new_feature, -1);
    // }

    // cv_bridge::CvImage debug_image(cam0_curr_img_ptr->header, "bgr8", out_img);
    // debug_stereo_pub.publish(debug_image.toImageMsg());


        

        if (frame > 0){
          initialEstimate.insert(X(frame), currPose);
          initialEstimate.insert(V(frame), currVelocity);
          newTimestamps[X(frame)] = timestep;
          newTimestamps[V(frame)] = timestep;
          //initialEstimate.insert(B(frame), currBias);
          //newTimestamps[B(frame)] = timestep;
          
          

          //Add Imu Factor
          ImuFactor imufac = create_imu_factor(timestep);
          graph.add(imufac);

          //imuBias::ConstantBias zero_bias(Vector3(0.1, 0.1, 0.1), Vector3(0.1, 0.1, 0.1));
          auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
          if (frame > 1){
            //auto biasnoise = noiseModel::Diagonal::Sigmas(Vector6::Constant(0.1));
            graph.addPrior<imuBias::ConstantBias>(B(frame-1), currBias, biasnoise);
            initialEstimate.insert(B(frame-1), currBias);
            newTimestamps[B(frame-1)] = timestep;
            
          }
        //   if (frame>2){
        //       graph.add(BetweenFactor<imuBias::ConstantBias>(B(frame-2), 
        //                                               B(frame-1), 
        //                                               zero_bias, biasnoise));
        //   }
          


          
        }
        
        


         
        smootherISAM2.update(graph, initialEstimate,newTimestamps);
        
        // currentEstimate = smootherISAM2.calculateEstimate();
        // currPose = currentEstimate.at<Pose3>(X(frame));
        // currVelocity = currentEstimate.at<Vector3>(V(frame));
        // graphError = graph.error(currentEstimate);
        
        // graph.resize(0);
         

        // for (const int i: newLands) {
        //     for (const int j: landmarkIDs) {
        //         if ((abs(j-i)<5000) || !currentEstimate.exists(L(j)) ) {continue;}
        //         else{
        //             double dist = (currentEstimate.at<Point3>(L(j))-currentEstimate.at<Point3>(L(i))).norm();
        //             if (dist < .03){
        //                 cout << "dist: " << dist << " i: " << i << "j: " << j << endl;
        //                 matchCount ++;
        //                 graph.add(BetweenFactor<Point3>(L(i), L(j), samePoint, pointnoise));
        //                 break;
        //             }
        //         }
        //     }
        //     if (matchCount > 10){
        //         break;
        //     }
        // }
        // cout << "MacthCount: " << matchCount << endl;

        // smootherISAM2.update(graph);
        for(size_t i = 1; i < 7; ++i) { // Optionally perform multiple iSAM2 iterations
            smootherISAM2.update();
        }

        currentEstimate = smootherISAM2.calculateEstimate();
        currPose = currentEstimate.at<Pose3>(X(frame));
        currVelocity = currentEstimate.at<Vector3>(V(frame));
        graphError = graph.error(currentEstimate);

        // if (frame % 10 == 0){
        //     cout << "  iSAM2 Smoother Keys: " << endl;
             
        //     cout << smootherISAM2.timestamps().size() << endl;
        //     smootherISAM2.print();
        //     for(const FixedLagSmoother::KeyTimestampMap::value_type& key_timestamp: smootherISAM2.timestamps()) {
        //         cout << setprecision(5) << "    Key: " << key_timestamp.first << "  Time: " << key_timestamp.second << endl;
        //     }
        // }
        

        graph.resize(0);
        initialEstimate.clear();
        newTimestamps.clear();
        accum.resetIntegration();
        frame ++;
        sendTfs(timestep);
        
}


// void StereoISAM2::fastPredictIMU(double t, Eigen::Vector3d linear_acceleration, Eigen::Vector3d angular_velocity)
// {
//     double dt = t - latest_time;
//     latest_time = t;
//     Eigen::Vector3d un_acc_0 = latest_Q * (latest_acc_0 - latest_Ba) - g;
//     Eigen::Vector3d un_gyr = 0.5 * (latest_gyr_0 + angular_velocity) - latest_Bg;
//     latest_Q = latest_Q * Utility::deltaQ(un_gyr * dt);
//     Eigen::Vector3d un_acc_1 = latest_Q * (linear_acceleration - latest_Ba) - g;
//     Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
//     latest_P = latest_P + dt * latest_V + 0.5 * dt * dt * un_acc;
//     latest_V = latest_V + dt * un_acc;
//     latest_acc_0 = linear_acceleration;
//     latest_gyr_0 = angular_velocity;
// }

Point3 StereoISAM2::triangulateFeature(periodic_slam::FeatureMeasurement feature){ 

    //cout << feature.u0  << feature.u1 << feature.v0 << endl;
    double d = feature.u0 - feature.u1;

    //cout << d<< endl;
    double z = fx*baseline/d;
    double x = (feature.u0-cx)*z/fx;
    double y = (feature.v0-cy)*z/fy;
    Point3 camera_point = Point3(x,y,z); 
    Point3 body_point = bodyToSensor.transformFrom(camera_point);
    Point3 world_point = currPose.transformFrom(body_point);
    return world_point;
}
 
 
void StereoISAM2::imuCallback(const sensor_msgs::Imu &imu_msg){
 
    geometry_msgs::Vector3 aV = imu_msg.angular_velocity;
    geometry_msgs::Vector3 lA = imu_msg.linear_acceleration;
    Vector3 measuredAcc(lA.x,lA.y*0,lA.z);
    Vector3 measuredOmega(aV.x,aV.y,aV.z);

    // gtsam::Matrix3 rotOne;
    // double angleOne = 1.57079;
    // rotOne.setZero();
    // rotOne <<  cos(angleOne),   sin(angleOne),    0,
    //            -sin(angleOne),  cos(angleOne),    0, 
    //               0,               0,         1;

    // //Rotate in Y by -pi/2
    // gtsam::Matrix3 rotTwo;
    // double angleTwo = 1.570796;
    // rotTwo.setZero();
    // rotTwo <<  cos(angleTwo),   0,        sin(angleTwo),    
    //         0,                1,                      0,
    //           -sin(angleTwo),  0,          cos(angleTwo);
 
    // Vector3 accCorr = rotTwo*rotOne*measuredAcc;
    // Vector3 gyrCorr = rotTwo*rotOne*measuredOmega;
  
    double timestep = imu_msg.header.stamp.toSec();
    imu_times.push_back(timestep);
    imu_linaccs.push_back(measuredAcc);
    imu_angvel.push_back(measuredOmega); 
    //imu_linaccs.push_back(accCorr);
    //imu_angvel.push_back(gyrCorr);  
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
    ImuFactor imufac(X(frame - 1), V(frame - 1), X(frame), V(frame), B(frame-1), accum); 
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
    t = currPose.translation();
    r = currPose.rotation();
    transform.setOrigin(tf::Vector3(t(0), t(1), t(2)));
    q.setRPY(r.roll(), r.pitch(), r.yaw());
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "world", "Optimized Pose"));

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
    br.sendTransform(tf::StampedTransform(transform, ros::Time(timestep), "True Pose", "Camera"));


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
 
    gtsam::Pose3 pose = currentEstimate.at<Pose3>(X(frame-1)); 
    poseStamped.pose.position.x =  pose.x();
    poseStamped.pose.position.y = pose.y();
    poseStamped.pose.position.z = pose.z();
    pathOPTI.header.frame_id = "world";
    pathOPTI.poses.push_back(poseStamped);
    pathOPTI.header.stamp = poseStamped.header.stamp;
    pathOPTI_pub.publish(pathOPTI); 

    // //Publish SLAM Trajectory
    // gtsam::Pose3 pose;
    // for (int i = 0; i <= frame-1; i ++){
    //     if (!currentEstimate.exists(X(i))) {continue;}
    //     pose = currentEstimate.at<Pose3>(X(i));   
    //     poseStamped.pose.position.x =  pose.x();
    //     poseStamped.pose.position.y = pose.y();
    //     poseStamped.pose.position.z = pose.z();
    //     pathOPTI.header.frame_id = "world";
    //     if ((i == frame-1) && (pathOPTI.poses.size()<frame-1)){
    //         pathOPTI.poses.push_back(poseStamped);
    //     } else {
    //         pathOPTI.poses[i] = poseStamped;
    //     }
    //     pathOPTI.header.stamp = poseStamped.header.stamp;
    //     pathOPTI_pub.publish(pathOPTI); 
    // }
     
    
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