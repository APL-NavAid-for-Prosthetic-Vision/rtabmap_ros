#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2021 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#

import numpy as np

# ROS imports
import rospy

from geometry_msgs.msg import Transform
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion

from rtabmap_ros.msg import Landmarks
from rtabmap_ros.msg import Landmark
from rtabmap_ros.srv import LandmarksInsert, LandmarksInsertRequest, LandmarksInsertResponse

import yaml

###
class InsertLandmarks:
    # Initializer
    def __init__(self):
        # Init Node
        rospy.init_node('insert_landmarks')

        # ros params
        self.targetFileName = rospy.get_param('~target_file_name', "")

        self.landmarks_msg = None
        # Publisher
        rospy.wait_for_service('rtabmap/landmarks_insert')
        self.insertLandmarkSrvClient = rospy.ServiceProxy("rtabmap/landmarks_insert", LandmarksInsert)

    def loadData(self):
        """
            load data from file into the rtabmap_ros landmarks msg format
        return zero if the data was loaded and formatted successfully.
        """

        # check file path is available
        if not self.targetFileName:
            rospy.logerr(" argument \'target_file_name\' is missing!! ")
            return -1
        
        landmarksDict = None
        with open(self.targetFileName, 'r') as file:
            landmarksDict = yaml.safe_load(file)

        # parse data into rtabmap ros landmark msg format
        self.landmarks_msg = self.format_landmarks_msg_data(landmarksDict["landmarks"])

        # loading data successful 
        return 0

    def format_landmarks_msg_data(self, landmarksDict):
        """
        """
        landmarks_msg = []

        for lm_dict in landmarksDict:
            landmark = Landmark()
            landmark.landmarkId = lm_dict["landmark_id"]
            landmark.description = lm_dict["description"]
            landmark.nodeId = lm_dict["signature_id"]
            # landmark_pose
            landmark.landmarkPose = self.toGeometryMsgTransform(lm_dict["landmark_pose"])
            # odometry_pose
            landmark.odometryPose = self.toGeometryMsgTransform(lm_dict["odometry_pose"])
            # covariance
            landmark.covariance = lm_dict["covariance"]

            landmarks_msg.append(landmark)
            
        return landmarks_msg


    def toGeometryMsgTransform(self, transform_dict):
        """
            formats the dictinary into a ROS message GeometryMsg Transform into a dictonary.
        returns:
            the ROS message GeometryMsg Transform 
        """

        transformMsg = Transform()
        # translation
        transformMsg.translation.x = transform_dict["translation"]["x"]
        transformMsg.translation.y = transform_dict["translation"]["y"]
        transformMsg.translation.z = transform_dict["translation"]["z"]
        # rotation
        transformMsg.rotation.x = transform_dict["rotation"]["x"]
        transformMsg.rotation.y = transform_dict["rotation"]["y"]
        transformMsg.rotation.z = transform_dict["rotation"]["z"]
        transformMsg.rotation.w = transform_dict["rotation"]["w"]

        return transformMsg

    def insert_landmarks_client(self):
        """
        """
        
        reqMsg = LandmarksInsertRequest()
        reqMsg.landmarks = self.landmarks_msg
        # debug print line
        print(f" size of landmarks:  {len(reqMsg.landmarks)}"  )
        print(reqMsg)
        #--- 

        try:
            responseCall = self.insertLandmarkSrvClient(reqMsg)
            print(f" service response : {responseCall}")
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s"%e)   


    def spinOnce(self):
        """
            it runs the class sequence onces, sending the loaded landmarks to rtabmap_ros  
        """

        if self.loadData() != 0:
            rospy.logerr(" FAILED to load data from yaml or structure into msg format!.")
            return

        # ros server client - insertLandmarks
        self.insert_landmarks_client()

        return     

# main function
if __name__ == '__main__':
    """
    """
    try:
        il = InsertLandmarks()
        # main execution of the class
        il.spinOnce()
    except rospy.ROSInterruptException:
        pass


    