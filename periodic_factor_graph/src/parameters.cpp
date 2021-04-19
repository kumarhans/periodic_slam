#include "parameters.h"


double fx;
double fy;
double cx;
double cy;
double k1;
double k2;
double p1;
double p2;
double k3;
int width;
int height;
double baseline;
double bf;
double gravMag;
bool visualizeImages;
bool useGrav;
std::vector<int> estimators;

cv::Mat projMatrl;
cv::Mat projMatrr;
cv::Mat cameraMatrix;
cv::Mat STEREO_R;
cv::Mat STEREO_T;
cv::Mat distCoeff;
gtsam::Pose3 bodyToSensor;

double initX;
double initY;
double initZ;
double initRoll;
double initPitch;
double initYaw;


cv::Size IMAGES_SIZE;
std::string IMAGE_L_TOPIC;
std::string IMAGE_R_TOPIC;



template <typename T>
void readParametersHelper(ros::NodeHandle &nh, std::string name, T &ans){
    if (nh.getParam(name, ans)){
        //ROS_INFO_STREAM("Loaded " << name << ": " << ans);
    }
    else{
        //ROS_ERROR_STREAM("Failed to load " << name);
        nh.shutdown();
    }
  
}

void readParameters(ros::NodeHandle &nh){
 
    readParametersHelper<double>(nh,"fx",fx);
    readParametersHelper<double>(nh,"fy",fy);
    readParametersHelper<double>(nh,"cx",cx);
    readParametersHelper<double>(nh,"cy",cy);
    readParametersHelper<double>(nh,"k1",k1);
    readParametersHelper<double>(nh,"k2",k2);
    readParametersHelper<double>(nh,"p1",p1);
    readParametersHelper<double>(nh,"p2",p2);
    readParametersHelper<double>(nh,"k3",k3);
    readParametersHelper<double>(nh,"baseline",baseline);
    readParametersHelper<bool>(nh,"visualize",visualizeImages);
    readParametersHelper<bool>(nh,"useGrav",useGrav);
    readParametersHelper<double>(nh,"gravMag",gravMag);
    
    readParametersHelper<int>(nh,"width",width);
    readParametersHelper<int>(nh,"height",height);
    readParametersHelper<std::vector<int>>(nh,"estimators",estimators);

    readParametersHelper<double>(nh,"initX",initX);
    readParametersHelper<double>(nh,"initY",initY);
    readParametersHelper<double>(nh,"initZ",initZ);
    readParametersHelper<double>(nh,"initRoll",initRoll);
    readParametersHelper<double>(nh,"initPitch",initPitch);
    readParametersHelper<double>(nh,"initYaw",initYaw);

    
    readParametersHelper<std::string>(nh,"image0_topic",IMAGE_L_TOPIC);
    readParametersHelper<std::string>(nh,"image1_topic",IMAGE_R_TOPIC);
    bf = baseline*fx;

    projMatrl = (cv::Mat_<float>(3, 4) << fx, 0., cx, 0., 0., fy, cy, 0., 0, 0., 1., 0);
    projMatrr = (cv::Mat_<float>(3, 4) << fx, 0., cx, bf, 0., fy, cy, 0., 0., 0., 1., 0);
    cameraMatrix = (cv::Mat_<float>(3, 3) << fx, 0., cx, 0., fy, cy, 0., 0., 1.0);
    distCoeff = (cv::Mat_<float>(1, 5) << 0., 0., 0., 0., 0.);

    gtsam::Matrix4 roty;
    roty << 0., 0., 1., 0.,
            0., 1., 0., 0.,
            -1., 0., 0., 0.,
            0., 0., 0., 1.;
    gtsam::Matrix4 rotz; //is NegZ
    rotz << 0., 1., 0., 0.,
        -1., 0., 0., 0.,
        0., 0., 1., 0.,
        0., 0., 0., 1.;   

    gtsam::Matrix4 real;
    real <<  1.,   0.,   0.,    -0.01,
             0.,   1.,   0.,        0.,
             0.,   0.,   1.,        0., 
             0.,   0.,   0.,        1.;
 
    //bodyToSensor = gtsam::Pose3(roty*rotz);
    bodyToSensor = gtsam::Pose3(real);

    
    STEREO_R = (cv::Mat_<double>(3, 3) << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
    STEREO_R.convertTo(STEREO_R, CV_64F);
    STEREO_T = (cv::Mat_<double>(3, 1) << bf / fx, 0., 0.);
    STEREO_T.convertTo(STEREO_T, CV_64F);
    


    IMAGES_SIZE = cv::Size_<int>(width,height);

}






