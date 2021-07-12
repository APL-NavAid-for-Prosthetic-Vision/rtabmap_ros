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

from rtabmap_ros.msg import RegisteredData
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
        rospy.Subscriber("rtabmap/registered_data_WM", RegisteredData, callback=self.landmarkDectectionCallback)


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
            # #find landmark pose from labels (base on)
            # index = 0
            # for labelKey in msg.objectsMapKeys:
            #     index += 1 
            #     if labelKey == self.targetLabelKey:
            #         break

            # rospy.loginfo(" signature ID %d; label index: %d", msg.signatureId, index)
            # bridge = CvBridge()

            # cv_data = bridge.imgmsg_to_cv2(msg.semanticOccupancyMapValues[index],  desired_encoding="32FC4")
            # (rows,cols,channels) = cv_data.shape
            # # grab the first element of then index label
            # print("rows= {0}, cols= {1}, channel{2}".format(rows, cols, channels))
            # position = cv_data[0,0,0:3]

            position = [msg.pose.position.x + 4, msg.pose.position.y + 0.5,  msg.pose.position.y + 0.3]

            self.counter = self.counter + 1

            rospy.loginfo("============ (%d)", self.counter)
            
            landmark = Landmark()
            landmark.landmarkId = 31
            landmark.description = "tags example"
            #timeStamp of the image (type: double)
            landmark.timeStamp = msg.rgbImage.header.stamp.to_sec()
            landmark.nodeId = msg.nodeId

            transform = Transform()
            transform.translation.x = position[0]
            transform.translation.y = position[1]
            transform.translation.z = position[2]
            transform.rotation.x = 0
            transform.rotation.y = 0
            transform.rotation.z = 0
            transform.rotation.w = 1
            landmark.transform = transform

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