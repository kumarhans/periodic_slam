#!/usr/bin/env python  
import roslib
import rospy
import math
import tf
import tf_conversions
import geometry_msgs.msg
import rosbag
import numpy as np
#import matplotlib.pyplot as plt
import time
import math
import csv

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import LinkStates
from std_msgs.msg import Int32




def get6DOF(worldFrame,frameName):
    try:
        (trans,rot) = listener.lookupTransform(frameName, worldFrame,rospy.Time(0))
        return [trans[0],trans[1],trans[2],rot[0],rot[1],rot[2],rot[3]]


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        print('notFound')
        return [0,0,0,0,0,0,0]


def checkDistance():
    try:
        (transReal,rotReal) = listener.lookupTransform('Robottrans', 'world',rospy.Time(0))
        return math.sqrt(transReal[0]**2 + transReal[1]**2)


    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return 0
    

global_gt_pos = [0,0,0,0,0,0,0]
track_num = 100


def gaz_callback(msgs):
    global global_gt_pos
    global_gt_pos = [msgs.pose[11].position.x, msgs.pose[11].position.y, msgs.pose[11].position.z, msgs.pose[11].orientation.w, msgs.pose[11].orientation.x, msgs.pose[11].orientation.y,msgs.pose[11].orientation.z]


def track_callback(msgs):
    global track_num
    track_num = msgs.data
    print(track_num)



if __name__ == '__main__':
    rospy.init_node('error_listener')
    listener = tf.TransformListener()


    rospy.Subscriber("/gazebo/link_states",LinkStates, gaz_callback)
    rospy.Subscriber("/vins_estimator/track_count",Int32, track_callback)

 

    gt = []
    est = []
    t = []
    track_nums = []


    global global_gt_pos
    global track_num
 

    start = 0
    index = 0
 
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        
        #distance = checkDistance()
        #if distance > 0 and distance<1.0:
        if start == 0:
            start = time.time()

        gt.append(global_gt_pos)
        #est.append(get6DOF('body','world'))
        est.append(get6DOF('bodyIMU','world'))
        track_nums.append([track_num])
        #est.append(get6DOF('Optimized Pose','world'))
        # est.append(get6DOF('camera_link','map'))
        t.append([time.time()- start])
        print(global_gt_pos)


        index += 1

        rate.sleep()

    #print(Vicon)

    print(np.shape(np.array(t)))
    print(np.shape(np.array(est)))
    print(np.shape(np.array(gt)))
    print(np.shape(track_nums))

    print(track_nums)


    a = np.concatenate((np.array(t),np.array(est), np.array(gt),np.array(track_nums)),axis=1)

    print(np.shape(a))
    np.savetxt("vinsGait25fh.csv", a, delimiter=",")


    

  

