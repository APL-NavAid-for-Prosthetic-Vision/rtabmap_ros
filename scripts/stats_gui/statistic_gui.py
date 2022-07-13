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


    inner_box = QGroupBox()
    inner_box.setStyleSheet("QGroupBox {background-color: darkgrey; margin-top:-1em;}")

    inner_layout = QGridLayout()
    
    self.mm_plot = MatplotlibWidget(width=5, height=4, dpi=100, title="Time Elapsed for Map Manager")

    inner_layout.addWidget(self.mm_plot)
    # 
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