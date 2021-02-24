#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseArray.h"
#include "sensor_msgs/JointState.h"

#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <iostream>
#include <vector>
#include "eigen3/Eigen/Dense"
#include <sstream>

#include <iostream>
#include <fstream>

// std::ofstream myfile;
// myfile.open ("example.csv");
// myfile << "This is the first cell in the first column.\n";
// myfile << "a,b,c,\n";
// myfile << "c,s,v,\n";
// myfile << "1,2,3.456\n";
// myfile << "semi;colon";
// myfile.close();
// return 0;

//timestamp(2) dt(1) acc(3) ang_vel(3) joint_angles(18) feet_positions(6) contact(1)  
class Data_cb
{
public:

  Data_cb()
  {
    t_curr=0;
    t_last=0;
  }

  void callback(const sensor_msgs::ImuConstPtr& imu, 
              const sensor_msgs::JointStateConstPtr& joints_info, 
              const geometry_msgs::PoseArrayConstPtr& feet_info)
  {
    t_curr=imu->header.stamp.sec+1e-9*imu->header.stamp.nsec;
    seq_num=imu->header.seq;
    // Solve all of filtering here..
    if (cb_count>0)
    {
        if (cb_count==1)
        {
            robotData.open("robot_data.csv");
        }
        long double dt=t_curr-t_last;
        robotData<<imu->header.stamp.sec<<","<<imu->header.stamp.nsec<<",";
        robotData<<dt<<",";
        robotData<<imu->linear_acceleration.x<<","<<imu->linear_acceleration.y<<","<<imu->linear_acceleration.z<<",";
        robotData<<imu->angular_velocity.x<<","<<imu->angular_velocity.y<<","<<imu->angular_velocity.z<<",";
        for(int i=0; i<18;++i)
            robotData<<joints_info->position[i]<<",";
        for(int i=0;i<6;++i)
            robotData<<feet_info->poses[i].position.x<<","<<feet_info->poses[i].position.y<<","<<feet_info->poses[i].position.z<<",";
        for(int i=0;i<5;++i)
            robotData<<feet_info->poses[i].orientation.w<<",";
        robotData<<feet_info->poses[5].orientation.w<<std::endl;
    }
    cb_count++;
    // std::cout<<"cb_count= "<<cb_count<<std::endl;
    t_last=t_curr;
  }

private:
  
    int cb_count=0;
    std::ofstream robotData;
    long double t_curr;
    long double t_last;
    long int seq_num;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "save_sensor_data");
    ros::NodeHandle nh;
    Data_cb data_cb;
    message_filters::Subscriber<sensor_msgs::Imu> imu_sub;
    message_filters::Subscriber<sensor_msgs::JointState> joints_sub;
    message_filters::Subscriber<geometry_msgs::PoseArray> feet_sub;

    imu_sub.subscribe(nh, "/titan6_state/fbk/fuse6_imu", 10,ros::TransportHints().tcpNoDelay());
    joints_sub.subscribe(nh, "/titan6_state/fbk/joint_states", 10,ros::TransportHints().tcpNoDelay());
    feet_sub.subscribe(nh,"/titan6_state/cal/ee_status_list",10,ros::TransportHints().tcpNoDelay());
    message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::JointState, geometry_msgs::PoseArray> 
                                                                                                sync(imu_sub, joints_sub,feet_sub,30);
    sync.registerCallback(boost::bind(&Data_cb::callback,&data_cb, _1, _2,_3));

    ros::spin();
    return 0;
}
