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

class StatisticsGUI(QWidget): 
  def __init__(self): 
    super().__init__()


    # main_layout
    main_layout =  QGridLayout()
    main_layout.addWidget(QLabel("X Offset"), 0, 0, 1, 1)
    self.setLayout(main_layout)

    # show all the widgets 
    self.update() 
    self.show()