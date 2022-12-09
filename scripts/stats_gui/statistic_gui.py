#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2022 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#


# pyqt modules
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from .matplotlib_qtwidget import MatplotlibWidget


class RtabmapStruct():
  def __init__(self):
    self.rtabmap_hz = 0


class StatisticsGUI(QWidget):

  def __init__(self):
    """
    """
    super().__init__()

    # main_layout
    main_layout =  QGridLayout()
    self.setWindowTitle("Rtabmap Stats")

    # inner box in tab Map manager
    inner_box = QGroupBox()
    inner_box.setStyleSheet("QGroupBox {background-color: darkgrey; margin-top:-1em;}")

    inner_layout = QGridLayout()
    # processing plot
    labels_names = ["total", "octomap"]
    self.mm_plot = MatplotlibWidget(width=5, height=4, dpi=100, title="Time Elapsed for Map Manager", multiple_line=True,
        label_names=labels_names)
    self.mm_plot_grd = MatplotlibWidget(width=5, height=4, dpi=100, title="Ground Height Over Time", y_axis_name="height [m]")

    # data display
    #self.value_octomap_grd_height = QDoubleSpinBox()
    
    # inner box Widget: adding plot processing
    inner_layout.addWidget(self.mm_plot, 0, 0, 1, 2)

    # inner box Widget: adding plot ground over time
    row_pos=1
    inner_layout.addWidget(self.mm_plot_grd, row_pos, 0, 1, 2)

    # row_pos +=1
    # label_octomap_grd_height = QLabel("Octomap ground height:")
    # label_octomap_grd_height.setAlignment(Qt.AlignRight)
    # self.value_octomap_grd_height.setSuffix(" meters")
    # self.value_octomap_grd_height.setDecimals(4)
    # self.value_octomap_grd_height.setButtonSymbols(QAbstractSpinBox.NoButtons)
    # self.value_octomap_grd_height.setMinimum(-50.0)
    # inner_layout.addWidget(label_octomap_grd_height, row_pos, 0, 1, 1)
    # inner_layout.addWidget(self.value_octomap_grd_height, row_pos, 1 , 1, 1)
    # other data infomation TODO: include other data
    # row_pos +=1
    # inner_layout.addWidget(label1, row_pos, 0, 1, 1)
    # inner_layout.addWidget(value1, row_pos, 0, 1, 1)

    # updates layout 
    inner_box.setLayout(inner_layout)

    # --------------------------------------------------------
    
    # second tab
    # inner box in tab rtabmap 
    inner_box_2 = QGroupBox()
    inner_box_2.setStyleSheet("QGroupBox {background-color: darkgrey; margin-top:-1em;}")

    inner_layout_2 = QGridLayout()
    # processing plot
    labels_names = ["total", "rtabmap"]
    self.rtabmap_plot = MatplotlibWidget(width=5, height=4, dpi=100, title="Rtabmap Processing", multiple_line=True,
                                        label_names=labels_names)
    # stats data
    # Rtabmap_stats_hz
    self.rtabmap_stats_hz = QDoubleSpinBox()
    rtabmap_stats_hz_label = QLabel("Rtabmap Average Rate:")
    rtabmap_stats_hz_label.setAlignment(Qt.AlignRight)
    self.rtabmap_stats_hz.setSuffix(" Hz")
    self.rtabmap_stats_hz.setDecimals(2)
    self.rtabmap_stats_hz.setButtonSymbols(QAbstractSpinBox.NoButtons)

    # inner box Widget: adding rtabmap processing plot
    inner_layout_2.addWidget(self.rtabmap_plot, 0, 0, 1, 2)
    #  stats data
    # Rtabmap_stats_hz
    row_pos=1
    inner_layout_2.addWidget(rtabmap_stats_hz_label, row_pos, 0, 1, 1)
    inner_layout_2.addWidget(self.rtabmap_stats_hz, row_pos, 1, 1, 1)

    # updates layout 2
    inner_box_2.setLayout(inner_layout_2)

    # ----------------------------------------------------------

    # third tab
    # inner box in tab rtabmap 
    inner_box_3 = QGroupBox()
    inner_box_3.setStyleSheet("QGroupBox {background-color: darkgrey; margin-top:-1em;}")

    inner_layout_3 = QGridLayout()
    # processing plot
    labels_names = ["total time", "input processing time"]
    self.input_rtabmap_plot = MatplotlibWidget(width=5, height=4, dpi=100, title="Input Data Processing (RTabMap)", multiple_line=True,
                                              label_names=labels_names)

    row_pos=0
    inner_layout_3.addWidget(self.input_rtabmap_plot, row_pos, 0, 1, 2)

    # updates layout 3
    inner_box_3.setLayout(inner_layout_3)


    # -----------------------------------------------------------

    # adding tab to main layout
    tabwidget = QTabWidget()
    tabwidget.addTab(inner_box, "map manager")
    tabwidget.addTab(inner_box_2, "Rtabmap Stats")
    tabwidget.addTab(inner_box_3, "Input Data (RTabMap) Thread Stats")
    main_layout.addWidget(tabwidget, 0, 0)

    self.setLayout(main_layout)

    # show all the widgets 
    self.update() 
    self.show()

  def update_map_manager_graph(self, updateTimedValue):
    """
    """
    self.mm_plot.update_canvas(updateTimedValue, set_y_limit_mode='time')

  def update_map_manager_graph_2(self, mm_update_timed_value, om_update_timed_value):
    """
    """
    self.mm_plot.update_canvas_2(mm_update_timed_value, om_update_timed_value)

  def update_map_manager_ground_graph(self, value_ground_reference):
    """
    """
    self.mm_plot_grd.update_canvas(value_ground_reference, set_y_limit_mode='ground')

  # def update_map_manager_data(self, value_ground_reference):
  #   """
  #   """
  #   self.value_octomap_grd_height.setValue(value_ground_reference)

  def update_rtabmap_graph(self, rtabmap_time_total_sec, rtabmap_time_sec):
    """
    """
    self.rtabmap_plot.update_canvas_2(rtabmap_time_total_sec, rtabmap_time_sec, y_axes_lim_offset=0.1)

  def update_rtabmap_stat(self, stats_data=RtabmapStruct()):
    """
    """
    self.rtabmap_stats_hz.setValue(stats_data.rtabmap_hz)
  
  def update_input_data_thread_stats(self, total_time, processing_time):
    """
    """
    self.input_rtabmap_plot.update_canvas_2(total_time, processing_time, y_axes_lim_offset=0.1)