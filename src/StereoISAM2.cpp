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
#include <geometry_msgs/Twist.h>

#include <sensor_msgs/PointCloud2.h>
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
}

StereoISAM2::~StereoISAM2 () {
    delete sync;
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
 
    gazSub = nh.subscribe("/gazebo/link_states", 1000, &StereoISAM2::gazCallback);
    camSub = nh.subscribe("features", 1000, &StereoISAM2::camCallback, this);
 
    debug_pub = it.advertise("/ros_stereo_odo/debug_image", 1);
    point_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("landmark_point_cloud", 10);
    pathOPTI_pub = nh.advertise<nav_msgs::Path>("/vo/pathOPTI", 1);
    pathGT_pub = nh.advertise<nav_msgs::Path>("/vo/pathGT", 1);
    
    //pcl::PointCloud<pcl::PointXYZRGB>::Ptr StereoISAM2::landmark_cloud_msg_ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    
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

 

void StereoISAM2::camCallback(const periodic_slam::CameraMeasurementPtr& camera_msg){
        vector<periodic_slam::FeatureMeasurement> feature_vector = camera_msg->features;
        initialEstimate.insert(X(frame), currPose);
 
        int radius = 2;
        

        noiseModel::Isotropic::shared_ptr prior_landmark_noise = noiseModel::Isotropic::Sigma(3, 0.1);
        noiseModel::Isotropic::shared_ptr pose_landmark_noise = noiseModel::Isotropic::Sigma(3, 10.0); // one pixel in u and v
        gtsam::Cal3_S2Stereo::shared_ptr K{new gtsam::Cal3_S2Stereo(fx, fy, 0.0, cx, cy, baseline)};

        for (int i = 0; i < feature_vector.size(); i++){
            periodic_slam::FeatureMeasurement feature = feature_vector[i];
            int landmark_id = feature.id;
            double uL = feature.u0;
            double uR = feature.u1;
            double v = feature.v0;

            //cv::circle(debug, cvPoint(uL, v), radius*3, CV_RGB(255, 0, 0));
            //addVisualFactor(frame, landmark_id, uL, uR, v);

            double d = uL - uR;
            double z = fx*baseline/d;
            double x = (uL-cx)*z/fx;
            double y = (v-cy)*z/fy;
            
            Point3 camera_point = Point3(x,y,z); 
            Point3 body_point = bodyToSensor.transformFrom(camera_point);
            Point3 world_point = currPose.transformFrom(body_point) ;

            pcl::PointXYZRGB pcl_world_point = pcl::PointXYZRGB(200,0,100);
            pcl_world_point.x = world_point.x();
            pcl_world_point.y = world_point.y();
            pcl_world_point.z = world_point.z(); 
            landmark_cloud_msg_ptr->points.push_back(pcl_world_point);  
            
           
            if (!currentEstimate.exists(L(landmark_id))) {
                initialEstimate.insert(L(landmark_id), world_point);
            }
            
            // Add ISAM2 factor connecting this frame's pose to the landmark
            graph.emplace_shared<
            GenericStereoFactor<Pose3, Point3> >(StereoPoint2(uL, uR, v), 
                pose_landmark_noise, X(frame), L(landmark_id), K,bodyToSensor);
                
            // Removing this causes greater accuracy but earlier gtsam::IndeterminantLinearSystemException)
            //Add prior to the landmark as well    
            //graph.emplace_shared<PriorFactor<Point3> >(L(landmark_id), world_point, prior_landmark_noise);

            
        }

        geometry_msgs::PoseStamped poseStamped;
        poseStamped.header.frame_id="/world";
        poseStamped.header.stamp = ros::Time::now();
    

        poseStamped.pose.position.x =  currPose.x();
        poseStamped.pose.position.y = currPose.y();
        poseStamped.pose.position.z = currPose.z();
        pathOPTI.header.frame_id = "world";
        pathOPTI.poses.push_back(poseStamped);
        pathOPTI.header.stamp = poseStamped.header.stamp;
        pathOPTI_pub.publish(pathOPTI);

        poseStamped.pose.position.x =  gtPose.x();
        poseStamped.pose.position.y = gtPose.y();
        poseStamped.pose.position.z = gtPose.z();
        pathGT.header.frame_id = "world";
        pathGT.poses.push_back(poseStamped);
        pathGT.header.stamp = poseStamped.header.stamp;
        pathGT_pub.publish(pathGT);




        // Publish landmark PointCloud message (in world frame)
        landmark_cloud_msg_ptr->header.frame_id = "world";
        landmark_cloud_msg_ptr->height = 1;
        landmark_cloud_msg_ptr->width = landmark_cloud_msg_ptr->points.size();
        point_pub.publish(landmark_cloud_msg_ptr);
        landmark_cloud_msg_ptr->clear();
        sendTfs();

    
        

        isam.update(graph, initialEstimate);

        // // ofstream os("test.dot");
        // // graph.saveGraph(os, initialEstimate);


        currentEstimate = isam.calculateEstimate();
        currPose = currentEstimate.at<Pose3>(X(frame));

        graph.resize(0);
        initialEstimate.clear();
        frame ++;
}
 
void StereoISAM2::gazCallback(const gazebo_msgs::LinkStates &msgs){
    gtPose = Pose3(Rot3::Quaternion(msgs.pose[7].orientation.w, msgs.pose[7].orientation.x, msgs.pose[7].orientation.y,msgs.pose[7].orientation.z), Point3(msgs.pose[7].position.x, msgs.pose[7].position.y, msgs.pose[7].position.z));
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

 
 