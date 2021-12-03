#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2021 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#

import numpy as np

# ROS imports
import rospy

from rtabmap_ros.msg import Landmarks
from rtabmap_ros.msg import Landmark
from rtabmap_ros.srv import LandmarksQuery, LandmarksQueryRequest, LandmarksQueryResponse

from geometry_msgs.msg import Transform, Vector3, Quaternion

import yaml

###
class StoreLandmarksQuery:
    # Initializer
    def __init__(self):

        # Init Node
        rospy.init_node('store_landmarks', anonymous=True)

        # set up desired storing param config
        self.targetFileName = rospy.get_param('~target_save_file_name', "")

        # Client
        rospy.wait_for_service('rtabmap/landmarks_available')
        self.queryLandmarkSrvClient = rospy.ServiceProxy("rtabmap/landmarks_available", LandmarksQuery)

        self.landmarks = None

    def query(self):
        """
            when called, it requests the landmark service for the landmarks in rtabmap.
            The service receives the landmarks and store them in a global variable.
        
        """    
        reqMsg = LandmarksQueryRequest()
        # not supported within rtabmap_ros
        # reqMsg.lookInDB = True
        # reqMsg.maxRange = 100

        try:
            resMsg = self.queryLandmarkSrvClient(reqMsg)
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        landmarksMsg = resMsg.landmarks

        temp_landmarks = []
        for landmarkMsg in landmarksMsg:
            landmark = {}
            landmark["landmark_id"] = landmarkMsg.landmarkId
            landmark["description"] = landmarkMsg.description
            landmark["signature_id"] = landmarkMsg.nodeId
            landmark["landmark_pose"] = self.extractGeometryMsgTransform(landmarkMsg.landmarkPose)
            landmark["odometry_pose"] = self.extractGeometryMsgTransform(landmarkMsg.odometryPose)
            landmark["covariance"] = list(landmarkMsg.covariance)
           
            temp_landmarks.append(landmark)

        self.landmarks = {"landmarks": temp_landmarks}


    def save(self):
        """
            save the stored landmarks into a configuration file for later use within rtabmap for localization mode.
        """
        
        if not self.targetFileName:
            rospy.logerr(" argument \'target_save_file_name\' is missing!! ")
            return

        with open(self.targetFileName, 'w') as yaml_file:
            yaml.dump(self.landmarks, yaml_file, default_flow_style=False)


    def extractGeometryMsgTransform(self, transform_msg):
        """
            extract the ROS message GeometryMsg Transform into a dictonary.
        returns:
            a dictionary with the data of the ROS message GeometryMsg Transform
        """

        transform = {}
        # translation
        translation = {}
        translation["x"] = transform_msg.translation.x
        translation["y"] = transform_msg.translation.y
        translation["z"] = transform_msg.translation.z
        transform["translation"] = translation
        # rotation
        rotation = {}
        rotation["x"] = transform_msg.rotation.x
        rotation["y"] = transform_msg.rotation.y
        rotation["z"] = transform_msg.rotation.z
        rotation["w"] = transform_msg.rotation.w
        transform["rotation"] = rotation

        return transform

# main function
if __name__ == '__main__':
    """
        main executable for store_landmarks node. It creates a ros node to save the landmarks from rtabmap_ros.
        It requires for the ros param 'target_save_file_name' to be set in order to save yaml file with the landmarks.
    """
    
    # ros class for storing landmarks query
    ltq = StoreLandmarksQuery()

    # query landmarks from rtabmap
    ltq.query()

    # store landmarks into a config file for later use
    ltq.save()