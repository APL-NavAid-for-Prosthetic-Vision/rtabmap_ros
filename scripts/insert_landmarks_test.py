#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2021 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#

import numpy as np

# oepncv
import cv2

# ROS imports
import rospy
from cv_bridge import CvBridge

from sensor_msgs.msg import Image
from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from rtabmap_ros.msg import RGBDSemanticDetection
from rtabmap_ros.msg import Landmarks
from rtabmap_ros.msg import Landmark
from rtabmap_ros.srv import LandmarksInsert, LandmarksInsertRequest, LandmarksInsertResponse

###
class LandmarksTest:
    # Initializer
    def __init__(self):
         # Init Node
        rospy.init_node('landmarkTest', anonymous=True)

        # ros params
        self.targetLandmarksSize = rospy.get_param('~target_landmarks_size', 1)
        self.targetLabelKey = rospy.get_param('~target_label_key', 1)

        rospy.loginfo(" targetLandmarksSize: %d ", self.targetLandmarksSize)

        # Subscriber
        rospy.Subscriber("rtabmap/rgbd_semantic_detection_msg", RGBDSemanticDetection, callback=self.landmarkDectectionCallback)


        # Publisher
        rospy.wait_for_service('rtabmap/landmarks_insert')
        self.insertLandmarkSrvClient = rospy.ServiceProxy("rtabmap/landmarks_insert", LandmarksInsert)

        # 
        self.counter = 0

        # spin
        rospy.spin()
        
    def landmarkDectectionCallback(self, msg):
        # send out targetLandmarksSize into Rtabmap node
        #   Unit Test for the landmark functionality
        rospy.loginfo("  Rtabmap registered data receieved")

        if self.targetLandmarksSize > self.counter:
            
            position = [4, 0.5, 0.3]

            self.counter = self.counter + 1

            rospy.loginfo("============ (%d)", self.counter)
            
            landmark = Landmark()
            landmark.landmarkId = 31
            landmark.description = "tags example"
            #timeStamp of the image (type: double) : need to use the rgb timestamp
            landmark.timeStamp = msg.rgb.header.stamp.to_sec()
            
            transform = Transform()
            transform.translation.x = position[0]
            transform.translation.y = position[1]
            transform.translation.z = position[2]
            transform.rotation.x = 0
            transform.rotation.y = 0
            transform.rotation.z = 0
            transform.rotation.w = 1
            landmark.landmarkPose = transform

            transform = Transform()
            transform.translation.x = msg.odom.pose.pose.position.x
            transform.translation.y = msg.odom.pose.pose.position.y
            transform.translation.z = msg.odom.pose.pose.position.z
            transform.rotation.x = msg.odom.pose.pose.orientation.x
            transform.rotation.y = msg.odom.pose.pose.orientation.y
            transform.rotation.z = msg.odom.pose.pose.orientation.z
            transform.rotation.w = msg.odom.pose.pose.orientation.w
            landmark.odometryPose = transform

            # It's requires for the landmark to have convariance with diagonal of non-zero
            convariance = np.zeros((6, 6), dtype=np.double)
            np.fill_diagonal(convariance, convariance.diagonal() + 0.01)
            convarianceArray = np.reshape(convariance, (36))
            landmark.covariance = convarianceArray.tolist()

            # for this test only adding one landmark into list
            reqMsg = LandmarksInsertRequest()
            reqMsg.landmarks.append(landmark)
            print(reqMsg)

            try:
                responseCall = self.insertLandmarkSrvClient(reqMsg)
                print(responseCall)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)        

# main function
if __name__ == '__main__':
    try:
        lt = LandmarksTest()
    except rospy.ROSInterruptException:
        pass