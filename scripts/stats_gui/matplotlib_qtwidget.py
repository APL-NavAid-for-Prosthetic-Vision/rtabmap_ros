#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2022 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#

import matplotlib
matplotlib.use('Qt5Agg')

from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtWidgets import *

from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg, NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure

import time

import matplotlib.pyplot as plt
import numpy as np

class MatplotlibWidget(QWidget):
  """
  """
  def __init__(self, parent=None, width=5, height=4, dpi=100, title=""):
    """
    """
    super().__init__()

    layout_canvas = QGridLayout()

    fig = Figure(figsize=(width, height), dpi=dpi)
    dynamic_canvas = FigureCanvasQTAgg(fig)

    layout_canvas.addWidget(dynamic_canvas)
    layout_canvas.addWidget(NavigationToolbar(dynamic_canvas, self))

    self._dynamic_ax = dynamic_canvas.figure.subplots()

    self._dynamic_ax.axes.set_title(title)
    self._dynamic_ax.axes.grid(True, color='b', linestyle='--',alpha=0.3)

    self.time = 0
    self._time_stamp = time.time()
    self.x_axis_data = np.linspace(0, 0.1, 1)
    self.y_axis_data = np.array([0.0 for x in self.x_axis_data])
    # Set up a Line2D.
    self._line, = self._dynamic_ax.plot(self.x_axis_data, self.y_axis_data)

    self._line.axes.set_xlabel("Seconds")
    self._line.axes.set_ylabel("Seconds")

    self.setLayout(layout_canvas)

  def update_canvas(self, y_value):
    """
    """
    timeStop = time.time()
    timeElapsed = timeStop - self._time_stamp
    self.time += timeElapsed
    
    self.x_axis_data = np.append(self.x_axis_data, self.time)
    self.y_axis_data = np.append(self.y_axis_data, y_value)

    y_limit_value = np.amax(self.y_axis_data)
    
    self._line.set_data(self.x_axis_data, self.y_axis_data)
    # update axes 
    self._line.axes.set_xlim(0, self.time + 1)
    self._line.axes.set_ylim(0, y_limit_value + 1)

    self._line.figure.canvas.draw()

    # update time stamp
    self._time_stamp = timeStop 

    