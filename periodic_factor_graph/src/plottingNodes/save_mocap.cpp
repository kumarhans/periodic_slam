#include <ros/ros.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <fstream>

int main(int argc, char** argv){
    ros::init(argc, argv, "my_tf_listener");
    ros::NodeHandle node;

    tf::TransformListener listener;

    std::ofstream mocapData;
    mocapData.open("mocap_data.csv");
    bool got=false;
    ros::Rate rate(100.0);
    while (node.ok())
    {
        tf::StampedTransform transform;
        try{
            listener.lookupTransform("/world", "/base_link",  
                                    ros::Time(0), transform);
            got=true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(1.0).sleep();
            got=false;
        }
        if(got==true)
        {
            tf::Vector3 r=transform.getOrigin();
            tf::Quaternion q=transform.getRotation();

            mocapData<<r.x()<<","<<r.y()<<","<<r.z()<<",";
            mocapData<<q.w()<<","<<q.x()<<","<<q.y()<<","<<q.z()<<std::endl;
        }
        
        rate.sleep();
    }
    return 0;
};