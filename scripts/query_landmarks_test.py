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

###
class LandmarksQueryTest:
    # Initializer
    def __init__(self):

         # Init Node
        rospy.init_node('landmarkTest', anonymous=True)

        # Client
        rospy.wait_for_service('rtabmap/landmarks_available')
        queryLandmarkSrvClient = rospy.ServiceProxy("rtabmap/landmarks_available", LandmarksQuery)

        rate = rospy.Rate(1)
        while not rospy.is_shutdown():

            reqMsg = LandmarksQueryRequest()
            reqMsg.lookInDB = True
            reqMsg.maxRange = 100

            try:
                resMsg = queryLandmarkSrvClient(reqMsg)
                print(resMsg)
            except rospy.ServiceException as e:
                print("Service call failed: %s"%e)    

            rate.sleep()

# main function
if __name__ == '__main__':
    try:
        lt = LandmarksQueryTest()
    except rospy.ROSInterruptException:
        pass