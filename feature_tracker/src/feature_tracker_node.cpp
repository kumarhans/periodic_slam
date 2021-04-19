#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include "visualization.h"
#include <thread>
#include <mutex>
#include "feature_tracker.h"
#include <feature_tracker/CameraMeasurement.h>
#include <feature_tracker/TrackingInfo.h>
#include <feature_tracker/PhaseFrames.h>
#include <gazebo_msgs/LinkStates.h>
#include <tf/transform_datatypes.h>
#include "opencv2/core.hpp"

#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
// #include "opencv2/xfeatures2d.hpp"
#include <assert.h> 

using namespace cv;
//using namespace cv::xfeatures2d;



#define SHOW_UNDISTORTION 0


ros::Publisher pub_image_track1, pub_image_track2, pub_image_track3, feature_pub, pub_image_debug, pub_track;

FeatureTracker featureTracker1(1);
FeatureTracker featureTracker2(2);
FeatureTracker featureTracker3(3);

FeatureTracker featureTracker4(4);
FeatureTracker featureTracker5(5);

double currPitch = 0.0;
double currPhase = 0.0;



Matrix3d ric[2];
Vector3d tic[2];
deque<sensor_msgs::ImageConstPtr> img0_buf;
deque<sensor_msgs::ImageConstPtr> img1_buf;
deque<double> phase_buf;

std::mutex m_buf;

cv::Mat img0_down;
cv::Mat img1_down;

cv::Mat img0_mid;
cv::Mat img1_mid;

cv::Mat img0_up;
cv::Mat img1_up;

double fx = 554.3827128;
double fy = 554.3827128;
double cx = 320.5;
double cy = 240.5;

double baseline = .1;

double currVel = 0.0;


void getStereoPairs(const cv::Mat &imLeftprev, const cv::Mat &imRightprev,
                    const cv::Mat &imLeftcurr, const cv::Mat &imRightcurr,
                                   vector<cv::Point2f> &prevLeftPts, 
                                   vector<cv::Point2f> &prevRightPts,
                                   vector<cv::Point2f> &currLeftPts,
                                   vector<cv::Point2f> &currRightPts){

        if(!prevLeftPts.empty())
        {
            //printf("stereo image; track feature on right image\n");
            vector<uchar> status1, status2;
            vector<float> err;
            // cur left ---- cur right
            cv::calcOpticalFlowPyrLK(imLeftprev, imRightprev, prevLeftPts, prevRightPts, status1, err, cv::Size(21, 21), 3);

            reduceVector(prevLeftPts, status1);
            reduceVector(prevRightPts, status1);
            reduceVector(currLeftPts, status1);

            cv::calcOpticalFlowPyrLK(imLeftcurr, imRightcurr, currLeftPts, currRightPts, status2, err, cv::Size(21, 21), 3);


            reduceVector(prevLeftPts, status2);
            reduceVector(prevRightPts, status2);
            reduceVector(currLeftPts, status2);
            reduceVector(currRightPts, status2);
        }

        assert (prevLeftPts.size() == currRightPts.size());
        
}



void pubTrackImage1(const cv::Mat &imgTrack, const double t, int section)
{
    if (section == -1){
        return;
    }

    std_msgs::Header header;
    header.frame_id = std::to_string(section);
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track1.publish(imgTrackMsg);

}

void pubTrackImage2(const cv::Mat &imgTrack, const double t, int section)
{
    if (section == -1){
        return;
    }

    std_msgs::Header header;
    header.frame_id = std::to_string(section);
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track2.publish(imgTrackMsg);

}

void pubTrackImage3(const cv::Mat &imgTrack, const double t, int section)
{
    if (section == -1){
        return;
    }

    std_msgs::Header header;
    header.frame_id = std::to_string(section);
    header.stamp = ros::Time(t);
    sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", imgTrack).toImageMsg();
    pub_image_track3.publish(imgTrackMsg);

}

void pubStereoFeatures(map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame, const double t, int section){
    
    // Publish features.
    feature_tracker::CameraMeasurementPtr feature_msg_ptr(new feature_tracker::CameraMeasurement);
    feature_msg_ptr->header.stamp = ros::Time(t);
    feature_msg_ptr->section.data = section;
    
    if (section == -1){
        feature_pub.publish(feature_msg_ptr);
        return;
    }

    //xyz_uv_velocity << x, y, z, p_u, p_v, velocity_x, velocity_y;
    int i = 0;
    for (auto const& feature : featureFrame){
        feature_msg_ptr->features.push_back(feature_tracker::FeatureMeasurement());

        // feature_msg_ptr->features[i].id = feature.first;
        // feature_msg_ptr->features[i].u0 = feature.second[0].second[0]*554.3827128 + 640/2 + .5;
        // feature_msg_ptr->features[i].v0 = feature.second[0].second[1]*554.3827128 + 480/2 + .5;
        // feature_msg_ptr->features[i].u1 = feature.second[1].second[0]*554.3827128 + 640/2 + .5;
        // feature_msg_ptr->features[i].v1 = feature.second[1].second[1]*554.3827128 + 480/2 + .5;

        feature_msg_ptr->features[i].id = feature.first;
        feature_msg_ptr->features[i].u0 = feature.second[0].second[0]*385.7544860839844 + 323.1204833984375;
        feature_msg_ptr->features[i].v0 = feature.second[0].second[1]*385.7544860839844+ 236.7432098388672;
        feature_msg_ptr->features[i].u1 = feature.second[1].second[0]*385.7544860839844 + 323.1204833984375;
        feature_msg_ptr->features[i].v1 = feature.second[1].second[1]*385.7544860839844 + 236.7432098388672;

        i++;
    }


    feature_pub.publish(feature_msg_ptr);

}

void pubTrackCount(const int count){
    std_msgs::Int32 msg;
    msg.data = count;
    pub_track.publish(msg); 
}

// void img_match(int section)
// {   
//     cv::Mat img1 = img0_mid.clone();
//     cv::Mat img2;
//     cv::Mat img1R = img1_mid.clone();
//     cv::Mat img2R;

//     if (section == 4){
//         img2 = img0_down;
//         img2R = img1_down;
        
//         cv::Mat pRoi = img1(cv::Rect(0, 0, 640, 240)); 
//         pRoi.setTo(cv::Scalar(0));
//     } else {
//         img2 = img0_up;
//         img2R = img1_up;
        
//         cv::Mat pRoi = img1(cv::Rect(0, 240, 640, 240)); 
//         pRoi.setTo(cv::Scalar(0));

//     }

    

//     //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
//     int minHessian = 6;
//     Ptr<SURF> detector = SURF::create( minHessian );
//     std::vector<KeyPoint> keypoints1, keypoints2;
//     Mat descriptors1, descriptors2;
//     detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
//     detector->detectAndCompute( img2, noArray(), keypoints2, descriptors2 );
//     //-- Step 2: Matching descriptor vectors with a FLANN based matcher
//     // Since SURF is a floating-point descriptor NORM_L2 is used
//     Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);
//     std::vector< std::vector<DMatch> > knn_matches;
//     matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
//     //-- Filter matches using the Lowe's ratio test
//     const float ratio_thresh = 0.7f;
//     std::vector<DMatch> good_matches;
//     for (size_t i = 0; i < knn_matches.size(); i++)
//     {
//         if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
//         {
//             good_matches.push_back(knn_matches[i][0]);
//         }
//     }

//     //-- Draw matches
//     Mat img_matches;
//     drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
//                  Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );


//     std::vector<cv::Point2f> points1prev; 
//     std::vector<cv::Point2f> points2prev; 
//     std::vector<cv::Point2f> points1curr; 
//     std::vector<cv::Point2f> points2curr; 


//     for (auto& match: good_matches){
//         int query = match.queryIdx;
//         int train = match.trainIdx;
//         cv::Point2f p1 = keypoints1[query].pt;
//         points1prev.push_back(p1);
//         cv::Point2f p2 = keypoints2[train].pt;
//         points1curr.push_back(p2);
//     }


//     getStereoPairs( img1, img1R, img2, img2R, points1prev, points2prev, points1curr, points2curr);
//     cout << points1prev.size() << " " << points2prev.size() << " " << points1curr.size() << " " << points2curr.size() << "  " << "FFFFFFFF" << endl;


//     std_msgs::Header header;
//     header.frame_id = "world";
//     header.stamp = ros::Time::now();
//     sensor_msgs::ImagePtr imgTrackMsg = cv_bridge::CvImage(header, "bgr8", img_matches).toImageMsg();
//     pub_image_debug.publish(imgTrackMsg);


//     feature_tracker::CameraMeasurementPtr feature_msg_ptr(new feature_tracker::CameraMeasurement);
//     feature_msg_ptr->header.stamp = ros::Time::now();
//     feature_msg_ptr->section.data = section;
    


//     for (int i = 0; i < points1prev.size(); i ++){
        
//         feature_msg_ptr->features.push_back(feature_tracker::FeatureMeasurement());
//         feature_msg_ptr->features[i*2].id = i*2;
//         feature_msg_ptr->features[i*2].u0 = points1prev[i*2].x;
//         feature_msg_ptr->features[i*2].v0 = points1prev[i*2].y;
//         feature_msg_ptr->features[i*2].u1 = points2prev[i*2].x;
//         feature_msg_ptr->features[i*2].v1 = points2prev[i*2].y;


//         feature_msg_ptr->features.push_back(feature_tracker::FeatureMeasurement());
//         feature_msg_ptr->features[i*2+1].id = i*2+1;
//         feature_msg_ptr->features[i*2+1].u0 = points1curr[i*2+1].x;
//         feature_msg_ptr->features[i*2+1].v0 = points1curr[i*2+1].y;
//         feature_msg_ptr->features[i*2+1].u1 = points2curr[i*2+1].x;
//         feature_msg_ptr->features[i*2+1].v1 = points2curr[i*2+1].y;
        
//     }


//     feature_pub.publish(feature_msg_ptr);


// }



void img0_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img0_buf.push_back(img_msg);
    phase_buf.push_back(currPhase);
    m_buf.unlock();
}


void img1_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    m_buf.lock();
    img1_buf.push_back(img_msg);
    m_buf.unlock();
}


cv::Mat getImageFromMsg(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImageConstPtr ptr;
    if (img_msg->encoding == "8UC1")
    {
        sensor_msgs::Image img;
        img.header = img_msg->header;
        img.height = img_msg->height;
        img.width = img_msg->width;
        img.is_bigendian = img_msg->is_bigendian;
        img.step = img_msg->step;
        img.data = img_msg->data;
        img.encoding = "mono8";
        ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
    }
    else
        ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);

    cv::Mat img = ptr->image.clone();

         
    
    // int kernel_size = abs(currVel)/7.5*51;
    // if (kernel_size % 2 == 0){
    //     kernel_size ++;
    // }
   
  
     
    // if (kernel_size > 2){
    //     cv::blur(img, img, cv::Size(kernel_size,kernel_size));
    // }
    
    

    return img;
}


void feature_frame_push(double t, const cv::Mat &_img, const cv::Mat &_img1, double phase)
{
    
    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> featureFrame;
    TicToc featureTrackerTime;
    double pitch = currPitch;
    
    cv::Mat imgTrack;

 
    cout << "currPhase" << phase << endl;

    if(_img1.empty())
        featureFrame = featureTracker1.trackImage(t, _img);
    else{
        if (phase < -.23){  //magic number is .43  or .23 or .4  //Fast For -.23
            if (img0_down.empty()){
                img0_down = _img;
                img1_down = _img1;
                //img_match(4);
                 
            }
            cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            pubTrackCount(featureTracker1.trackedNum);
            pubStereoFeatures(featureTracker1.trackImage(t, _img, _img1),t, 1);
            pubTrackImage1(featureTracker1.getTrackImage(), t, 1);
            
        } else if (-.11 < phase  && phase < -.07){ //Fast For -.11 to -.07
            if (img0_mid.empty()){
                img0_mid = _img;
                img1_mid = _img1;
                cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            }
            cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            pubStereoFeatures(featureTracker2.trackImage(t, _img, _img1),t, 2);
            pubTrackImage2(featureTracker2.getTrackImage(), t, 2);
            
        } else if (phase > .1){  //Fast For .1
            if (img0_up.empty()){
                img0_up = _img;
                img1_up = _img1;
                cout << "pitch:   " << pitch << "  phase  " << phase << endl;
                //img_match(5);
                
            }
            cout << "pitch:   " << pitch << "  phase  " << phase << endl;
            pubStereoFeatures(featureTracker3.trackImage(t, _img, _img1),t, 3);
            pubTrackImage3(featureTracker3.getTrackImage(), t, 3);
            
        }
        
        //  else if (-.55 < phase  && phase < -.45){
        //     if (img0_up.empty()){
        //         img0_up = _img;
        //         img1_up = _img1;
        //         cout << "pitch:   " << pitch << "  phase  " << phase << endl;
        //         //img_match(5);
                
        //     }
        //     cout << "pitch:   " << pitch << "  phase  " << phase << endl;
        //     pubStereoFeatures(featureTracker4.trackImage(t, _img, _img1),t, 4);
        //     //pubTrackImage(featureTracker4.getTrackImage(), t, 4);
            
        // } else if (.45 < phase  && phase < .55){
        //     if (img0_up.empty()){
        //         img0_up = _img;
        //         img1_up = _img1;
        //         cout << "pitch:   " << pitch << "  phase  " << phase << endl;
        //         //img_match(5);
                
        //     }
        //     cout << "pitch:   " << pitch << "  phase  " << phase << endl;
        //     pubStereoFeatures(featureTracker5.trackImage(t, _img, _img1),t, 5);
        //     //pubTrackImage(featureTracker5.getTrackImage(), t, 5);
            
        // }
         else{
            pubStereoFeatures(featureFrame, t, -1);
            pubTrackImage1(featureTracker1.getTrackImage(), t, -1);
        }
    } 
        

    

}

void sync_process()
{
    while(1)
    {
        if(STEREO)
        {
            cv::Mat image0, image1;
            std_msgs::Header header;
            double time;
            double phase;
            m_buf.lock();
            if (!img0_buf.empty() && !img1_buf.empty())
            {
                double time0 = img0_buf.front()->header.stamp.toSec();
                double time1 = img1_buf.front()->header.stamp.toSec();
                // 0.003s sync tolerance
                //cout << time0 << " " << time1 << endl;
       
                if(time0 < time1 - 0.003)
                {
                    img0_buf.pop_front();
                    printf("throw img0\n");
                }
                else if(time0 > time1 + 0.003)
                {
                    img1_buf.pop_front();
                    printf("throw img1\n");
                }
                else
                {
                    time = img0_buf.front()->header.stamp.toSec();
                    header = img0_buf.front()->header;
                    image0 = getImageFromMsg(img0_buf.front());
                    img0_buf.pop_front();
                    image1 = getImageFromMsg(img1_buf.front());
                    img1_buf.pop_front();
                    phase = phase_buf.front();
                    phase_buf.pop_front();
                    //printf("find img0 and img1\n");
                }
            }
            m_buf.unlock();
            if(!image0.empty())                
                feature_frame_push(time, image0, image1, phase);
         
//         if (STEREO)
//         {
//             cv::Mat image0, image1;
//             std_msgs::Header header;
//             double time;
//             m_buf.lock();
//             if (img0_buf.size() > 2 && img1_buf.size() > 2)
//             {
//                 image0 = 0.0*getImageFromMsg(img0_buf.front());
//                 img0_buf.pop_front();
//                 time = img0_buf.front()->header.stamp.toSec();
//                 header = img0_buf.front()->header;
//                 image0 += 1.0*getImageFromMsg(img0_buf.front()); 
//                 image0 += 0.0*getImageFromMsg(img0_buf[1]);
                 
//                 image1 = 0.0*getImageFromMsg(img1_buf.front());
//                 img1_buf.pop_front();
//                 time = img1_buf.front()->header.stamp.toSec();
//                 header = img1_buf.front()->header;
//                 image1 += 1.0*getImageFromMsg(img1_buf.front()); 
//                 image1 += 0.0*getImageFromMsg(img1_buf[1]);
//                 phase_buf.pop_front();
                
//             }
//             m_buf.unlock();
//             if(!image0.empty())
//                 feature_frame_push(time, image0, image1, phase_buf.front());
        }
        else
        {
            cv::Mat image;
            std_msgs::Header header;
            double time = 0;
            m_buf.lock();
            if(!img0_buf.empty())
            {
                time = img0_buf.front()->header.stamp.toSec();
                header = img0_buf.front()->header;
                image = getImageFromMsg(img0_buf.front());
                img0_buf.pop_front()
;
            }
            m_buf.unlock();
            // if(!image.empty())
            //     estimator.inputImage(time, image);
        }

        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
   }
}



void gazCallback(const gazebo_msgs::LinkStates &msgs){
    tf::Quaternion q(msgs.pose[11].orientation.w, msgs.pose[11].orientation.x, msgs.pose[11].orientation.y,msgs.pose[11].orientation.z);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currPitch = pitch;
}

void gtCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    tf::Quaternion q(msg->pose.orientation.x, msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    currPhase = roll;
    cout << currPitch << endl;
}


void phaseCallback(const std_msgs::Float64 &msgs){
    currPhase = msgs.data;
}

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
    double t = imu_msg->header.stamp.toSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;
    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;
    currVel = ry;
}



int main(int argc, char **argv)
{
    ros::init(argc, argv, "feature_tracker");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);

    if(argc != 2)
    {
        printf("please intput: rosrun vins vins_node [config file] \n"
               "for example: rosrun vins vins_node "
               "~/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 1;
    }

    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);
    readParameters(config_file);
 


    for (int i = 0; i < NUM_OF_CAM; i++)
    {
        tic[i] = TIC[i];
        ric[i] = RIC[i];
        cout << " exitrinsic cam " << i << endl  << ric[i] << endl << tic[i].transpose() << endl;
    }

    featureTracker1.readIntrinsicParameter(CAM_NAMES);
    featureTracker2.readIntrinsicParameter(CAM_NAMES);
    featureTracker3.readIntrinsicParameter(CAM_NAMES);
    featureTracker4.readIntrinsicParameter(CAM_NAMES);
    featureTracker5.readIntrinsicParameter(CAM_NAMES);

   
    ros::Subscriber sub_img0 = n.subscribe(IMAGE0_TOPIC, 100, img0_callback);
    ros::Subscriber sub_img1 = n.subscribe(IMAGE1_TOPIC, 100, img1_callback);
    ros::Subscriber gazSUB = n.subscribe("/gazebo/link_states", 1000, gazCallback);
    ros::Subscriber phaseSUB = n.subscribe("/mobile_robot/phase", 1000, phaseCallback); 
    ros::Subscriber gtSUB = n.subscribe("/mocap_node/Robot_1/pose", 1000, gtCallback);
    
    //ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());


    pub_image_track1 = n.advertise<sensor_msgs::Image>("feature_img1",1000);
    pub_image_track2 = n.advertise<sensor_msgs::Image>("feature_img2",1000);
    pub_image_track3 = n.advertise<sensor_msgs::Image>("feature_img3",1000);
    
    pub_track = n.advertise<std_msgs::Int32>("/track_count1", 1000);
    pub_image_debug = n.advertise<sensor_msgs::Image>("debugging_img",1000);

    feature_pub = n.advertise<feature_tracker::CameraMeasurement>("/features", 3);

    std::thread sync_thread{sync_process};


    ros::spin();
    return 0;
}


