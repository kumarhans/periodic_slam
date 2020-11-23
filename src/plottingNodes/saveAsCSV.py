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


def gaz_callback(msgs):
    global global_gt_pos
    global_gt_pos = [msgs.pose[7].position.x, msgs.pose[7].position.y, msgs.pose[7].position.z, msgs.pose[7].orientation.w, msgs.pose[7].orientation.x, msgs.pose[7].orientation.y,msgs.pose[7].orientation.z]


if __name__ == '__main__':
    rospy.init_node('error_listener')
    listener = tf.TransformListener()


    rospy.Subscriber("/gazebo/link_states",LinkStates, gaz_callback)

 

    gt = []
    est = []
    t = []


    global global_gt_pos
 

    start = 0
    index = 0
 
    rate = rospy.Rate(100.0)
    while not rospy.is_shutdown():
        
        #distance = checkDistance()
        #if distance > 0 and distance<1.0:
        if start == 0:
            start = time.time()

        gt.append(global_gt_pos)
        # est.append(get6DOF('body','world'))
        est.append(get6DOF('Optimized Pose','world'))
        # est.append(get6DOF('camera_link','map'))
        t.append([time.time()- start])
        print(global_gt_pos)


        index += 1

        rate.sleep()

    #print(Vicon)

    print(np.shape(np.array(t)))
    print(np.shape(np.array(est)))
    print(np.shape(np.array(gt)))


    a = np.concatenate((np.array(t),np.array(est), np.array(gt)),axis=1)

    print(np.shape(a))
    np.savetxt("estBlurryNoIMU.csv", a, delimiter=",")


    

  

