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
    labels_names = ["total", "octomap"]
    self.mm_plot = MatplotlibWidget(width=5, height=4, dpi=100, title="Time Elapsed for Map Manager", multiple_line=True,
        label_names=labels_names)
    self.value_octomap_grd_height = QDoubleSpinBox()
    
    # adding widgets in inner box 
    inner_layout.addWidget(self.mm_plot, 0, 0, 1, 2)
    row_pos=1
    label_octomap_grd_height = QLabel("Octomap ground height:")
    label_octomap_grd_height.setAlignment(Qt.AlignRight)
    self.value_octomap_grd_height.setSuffix(" meters")
    self.value_octomap_grd_height.setDecimals(4)
    self.value_octomap_grd_height.setButtonSymbols(QAbstractSpinBox.NoButtons)
    self.value_octomap_grd_height.setMinimum(-50.0)
    inner_layout.addWidget(label_octomap_grd_height, row_pos, 0, 1, 1)
    inner_layout.addWidget(self.value_octomap_grd_height, row_pos,1 , 1, 1)
    # other data infomation TODO: include other data
    row_pos +=1
    # inner_layout.addWidget(label1, row_pos, 0, 1, 1)
    # inner_layout.addWidget(value1, row_pos, 0, 1, 1)

    # updates layout 
    inner_box.setLayout(inner_layout)

    # adding tab to main layout
    tabwidget = QTabWidget()
    tabwidget.addTab(inner_box, "map manager")
    main_layout.addWidget(tabwidget, 0, 0)

    self.setLayout(main_layout)

    # show all the widgets 
    self.update() 
    self.show()

  def update_map_manager_graph(self, updateTimedValue):
    """
    """
    self.mm_plot.update_canvas(updateTimedValue)

  def update_map_manager_graph_2(self, mm_update_timed_value, om_update_timed_value):
    """
    """
    self.mm_plot.update_canvas_2(mm_update_timed_value, om_update_timed_value)

  def update_map_manager_data(self, value_ground_reference):
    """
    """
    self.value_octomap_grd_height.setValue(value_ground_reference)