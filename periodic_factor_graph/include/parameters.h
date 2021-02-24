#ifndef PARAM_h
#define PARAM_h

#include <ros/ros.h>
#include <vector>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam_unstable/slam/SmartStereoProjectionPoseFactor.h>
 
#include <opencv2/opencv.hpp>
 
 
extern double fx;
extern double fy;
extern double cx;
extern double cy;
extern double k1;
extern double k2;
extern double p1;
extern double p2;
extern double k3;
extern int width;
extern int height;
extern double baseline;
extern double bf;
extern double gravMag;
extern bool visualizeImages;
extern bool useGrav;

extern double initX;
extern double initY;
extern double initZ;
extern double initRoll;
extern double initPitch;
extern double initYaw;

extern std::vector<int> estimators;

extern cv::Mat projMatrl;
extern cv::Mat projMatrr;
extern cv::Mat cameraMatrix;
extern cv::Mat STEREO_R;
extern cv::Mat STEREO_T;
extern cv::Mat distCoeff;
extern gtsam::Pose3 bodyToSensor;

extern cv::Size IMAGES_SIZE;
extern std::string IMAGE_L_TOPIC;
extern std::string IMAGE_R_TOPIC;

template <typename T>
void readParametersHelper(ros::NodeHandle &nh, std::string name, T &ans);

void readParameters(ros::NodeHandle &nh);

 

#endif