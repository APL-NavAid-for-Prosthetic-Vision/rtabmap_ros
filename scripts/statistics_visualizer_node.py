#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2022 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#

# ROS imports
import rospy

# Python Imports
import sys
import os
import threading
import numpy as np

# pyqt modules
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

import StatisticGUI as gui

from rtabmap_ros.msg import MapManagerStats

###
class StatisticsVisualizer:
  """
  """
  # Initializer
  def __init__(self):
    # Init Node
    rospy.init_node('statistics_visualizer', anonymous=True)

    rospy.loginfo("Starting ROS node: statistics_visualizer")

    self.app_display = rospy.get_param('~app_display', True)

    if self.app_display:
      # Q Application
      self.App = QApplication(sys.argv) 
      # Create the GUI
      self.gui = gui.StatisticsGUI()

    # Launch processingThread thread
    # self.stop_thread = False
    # self.threadNH = threading.Thread(target=self.processingThread, args =(lambda : self.stop_thread, ))
    # self.threadNH.daemon = True
    # self.threadNH.start()

    # Subscriber
    rospy.Subscriber("rtabmap/map_manager_stats", MapManagerStats, callback=self.mapManagerStatsCallback)


  def processingThread(self, stop):
    """
    """
    rate = rospy.Rate(20)

    # Publisher loop
    while not rospy.is_shutdown():

      # sleep the remaining 
      rate.sleep()

      # End loop if stop function is called
      if stop():
        break

  def spin(self):
    """
    """

    if self.app_display:
      #
      rospy.loginfo(" app display")
      # Start the app (has internal loop)
      self.App.exec()
    else :
      rospy.loginfo(" no display")
      # spin
      rospy.spin()

    # End thread
    # self.stop_thread = True
    # self.threadNH.join()

  def mapManagerStatsCallback(self, msg):
    """
    """
    msg_header_time = msg.header.stamp.secs + msg.header.stamp.nsecs * 10**-9
    #rospy.loginfo(" time: %f", msg_header_time)

    mm_time = msg.map_manager_update_cache_time
    #rospy.loginfo(" map manager elapsed time: %f", mm_time)

    # if the msg contains octomap stats data
    if msg.is_octomap_data:
      octomap_time = msg.octomap_update_time

# main function
if __name__ == '__main__':
  """
  """
  try:
    statsVisualizerNH = StatisticsVisualizer()
    statsVisualizerNH.spin()
  except rospy.ROSInterruptException:
    pass