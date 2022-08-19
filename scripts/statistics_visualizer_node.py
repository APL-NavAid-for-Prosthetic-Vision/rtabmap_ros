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
import time

import numpy as np

# pyqt modules
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from stats_gui.statistic_gui import StatisticsGUI, RtabmapStruct

from rtabmap_ros.msg import MapManagerStats
from rtabmap_ros.msg import Info

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
      self.gui = StatisticsGUI()

    # Launch processingThread thread
    # self.stop_thread = False
    # self.threadNH = threading.Thread(target=self.processingThread, args =(lambda : self.stop_thread, ))
    # self.threadNH.daemon = True
    # self.threadNH.start()

    self.rtabmap_time_total_sec_buf =  np.array([], dtype=np.float64)
    self.last_response_time = time.time()
    self.rtabmap_stats = RtabmapStruct()

    # Subscriber
    rospy.Subscriber("rtabmap/map_manager_stats", MapManagerStats, callback=self.mapManagerStatsCallback, queue_size=1)
    rospy.Subscriber("rtabmap/info", Info, callback=self.infoStatsCallback, queue_size=1)


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
      grd_height = msg.octomap_grd_height

    # update data in GUI
    if not msg.is_octomap_data:
      # update processing elapsed time with latest values
      self.gui.update_map_manager_graph_2(mm_time, 0.0)
    else:
      # update processing elapsed time with latest values
      self.gui.update_map_manager_graph_2(mm_time, octomap_time)
      # update ground plot with latest value
      self.gui.update_map_manager_ground_graph(grd_height)


  def infoStatsCallback(self, msg):
    """
    """
    statsKeys = msg.statsKeys
    statsValues = msg.statsValues

    data_dictionary = {statsKeys[i]: statsValues[i] for i in range(len(statsKeys))}

    rtabmap_time_total_sec = 0.
    rtabmap_time_total_name = "RtabmapROS/TimeTotal/ms"
    if rtabmap_time_total_name in data_dictionary.keys():
      rtabmap_time_total_sec = data_dictionary[rtabmap_time_total_name]*(1/1000)

    rtabmap_time_sec = 0.
    rtabmap_time_name = "RtabmapROS/TimeRtabmap/ms"
    if rtabmap_time_name in data_dictionary.keys():
      rtabmap_time_sec = data_dictionary[rtabmap_time_name]*(1/1000)

    # update rtabmap processing graph  
    self.gui.update_rtabmap_graph(rtabmap_time_total_sec, rtabmap_time_sec)
    
    # update rtabmap stat data

    #  calculate Hz responce within a 1 second of data
    # in seconds
    current_time = time.time()
    time_elapsed = current_time - self.last_response_time
    self.rtabmap_time_total_sec_buf = np.append(self.rtabmap_time_total_sec_buf, rtabmap_time_total_sec)

    if time_elapsed >= 1:
      # on seconds has passed
      self.last_response_time = current_time
      self.rtabmap_stats.rtabmap_hz = 1. / np.mean(self.rtabmap_time_total_sec_buf)
      self.rtabmap_time_total_sec_buf = np.array([], dtype=np.float64)

    self.gui.update_rtabmap_stat(self.rtabmap_stats)

    
# main function
if __name__ == '__main__':
  """
  """
  try:
    statsVisualizerNH = StatisticsVisualizer()
    statsVisualizerNH.spin()
  except rospy.ROSInterruptException:
    pass