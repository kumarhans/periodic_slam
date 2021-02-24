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


from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

 

 

 

if __name__ == '__main__':
    rospy.init_node('error_listener')
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()
    path = Path()
    path2 = Path()
    path.header.frame_id = "/world"
    path2.header.frame_id = "/world"

    path_pub = rospy.Publisher('/path', Path, queue_size=10)
    path2_pub = rospy.Publisher('/path2', Path, queue_size=10)
 

    gt = []
    est = []
    t = []
    track_nums = []

 
 

    start = 0
    index = 0
    firstTf = False
    mat1 = 0
    direction = -1
    last = 0
    est = []
    rate = rospy.Rate(100.0)
    br = tf.TransformBroadcaster()
    newt = 0
    while not rospy.is_shutdown():
        
        t = rospy.Time(0)
        if not(firstTf):
            try:
                (trans1, rot1) = listener.lookupTransform('world','Robot_2/base_link', t)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            trans1_mat = tf.transformations.translation_matrix(trans1)
            rot1_mat   = tf.transformations.quaternion_matrix(rot1)
            mat1 = np.dot(trans1_mat, rot1_mat)
            R = tf.transformations.euler_matrix(0,.13, 0)
            # RMat = np.eye(4)
            # RMat[0:3,0:3] = R
            mat1 = np.dot(mat1,R)
            firstTf = True
        else:
            (trans2, rot2) = listener.lookupTransform('Robot_2/base_link','world', t)
            trans2_mat = tf.transformations.translation_matrix(trans2)
            rot2_mat = tf.transformations.quaternion_matrix(rot2)
            mat2 = np.dot(trans2_mat, rot2_mat)


            
            mat3 = np.dot(mat2,mat1)
            trans3 = tf.transformations.translation_from_matrix(mat3)
            rot3 = tf.transformations.quaternion_from_matrix(mat3)

            mat4 = tf.transformations.inverse_matrix(mat3)
            trans4 = tf.transformations.translation_from_matrix(mat4)
            rot4 = tf.transformations.quaternion_from_matrix(mat4)
            
            br.sendTransform(
            trans3,
            rot3,
            t,
            "world","target");
 
            est.append([newt,trans4[0],trans4[1],trans4[2],rot4[0],rot4[1],rot4[2],rot4[3]])

            # path.header.stamp = rospy.Time.now()
            # pose = PoseStamped()
            # pose.pose.position.x = trans4[0]
            # pose.pose.position.y = trans4[1]
            # pose.pose.position.z = trans4[2]
            # pose.pose.orientation.x = rot4[0]
            # pose.pose.orientation.y = rot4[1]
            # pose.pose.orientation.z = rot4[2]
            # pose.pose.orientation.w = rot4[3]

            # # al, be, ga = tf.transformations.euler_from_matrix(mat4, 'syxz')
            # # print(al,be,ga)
            # #print(pose.pose.position.z)

           
            # path.poses.append(pose)
            # path_pub.publish(path)
        
 
        # if newt  > 10:
        #     upDownAngle = 14/180.0*math.pi
        #     heaveLength = .03
        #     frequency = 1.625
            
        #     phase = math.sin(frequency*(newt)*(2*math.pi))/(abs(math.sin(frequency*(newt)*(2*math.pi)))**(.5))
            
        #     R = tf.transformations.euler_matrix(0,upDownAngle*phase, 0)
        #     rot5 = tf.transformations.quaternion_from_matrix(R)

        #     path2.header.stamp = t
        #     pose = PoseStamped()
            
        #     pose.pose.position.x = (newt-10)*.2
        #     pose.pose.position.y = 0
        #     pose.pose.position.z = phase*heaveLength +heaveLength
        #     pose.pose.orientation.x = rot5[0]
        #     pose.pose.orientation.y = rot5[1]
        #     pose.pose.orientation.z = rot5[2]
        #     pose.pose.orientation.w = rot5[3]
        #     path2.poses.append(pose)
        #     path2_pub.publish(path2)


        newt += .01

        rate.sleep()

    np.savetxt("realGait.csv" , est, delimiter=" ")
 
    print(np.shape(est))
    


    

  

