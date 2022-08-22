#! /usr/bin/env python

# This script was written for Python3
#
# Copyright 2022 The Johns Hopkins University
#   Applied Physics Laboratory.  All rights reserved.
#

from cProfile import label
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
  def __init__(self, parent=None, width=5, height=4, dpi=100, title="", multiple_line=False, 
      label_names=[], y_axis_name="Seconds"):
    """
    """
    super().__init__()

    # layout with in widget
    layout_canvas = QGridLayout()

    fig = Figure(figsize=(width, height), dpi=dpi)
    dynamic_canvas = FigureCanvasQTAgg(fig)

    # adding widgets
    layout_canvas.addWidget(dynamic_canvas)
    layout_canvas.addWidget(NavigationToolbar(dynamic_canvas, self))

    # creates a plot figure
    self._dynamic_ax = dynamic_canvas.figure.subplots()

    # config general params figure
    self._dynamic_ax.axes.set_title(title)
    self._dynamic_ax.axes.grid(True, color='b', linestyle='--',alpha=0.3)

    # dummy initial plot 
    self.time = 0
    self._time_stamp = time.time()
    self.x_axis_data = np.linspace(0, 0.1, 1)
    self.y_axis_data_1 = np.array([0.0 for x in self.x_axis_data])
    # Set up a Line2D.
    if not multiple_line:
      self._line1, = self._dynamic_ax.plot(self.x_axis_data, self.y_axis_data_1)
    else:
      # multiple lines
      self.y_axis_data_2 = np.array([0.0 for x in self.x_axis_data])
      self._line1, = self._dynamic_ax.plot(self.x_axis_data, self.y_axis_data_1, label = label_names[0])
      self._line2, = self._dynamic_ax.plot(self.x_axis_data, self.y_axis_data_2, label = label_names[1])
      legend_hn = self._dynamic_ax.legend(loc=2)
      legend_hn.set_draggable(True)

    # setting axes labels after settining initial empty plot, locates labels correctly.
    self._line1.axes.set_xlabel("Seconds")
    self._line1.axes.set_ylabel(y_axis_name)

    # updates layout in the main window.
    self.setLayout(layout_canvas)

  def update_canvas(self, y_value, set_y_limit_mode='time'):
    """
    """
    timeStop = time.time()
    timeElapsed = timeStop - self._time_stamp
    self.time += timeElapsed
    
    self.x_axis_data = np.append(self.x_axis_data, self.time)
    self.y_axis_data_1 = np.append(self.y_axis_data_1, y_value)

    if set_y_limit_mode == 'time':
      y_limit_value = np.amax(self.y_axis_data_1)
    elif set_y_limit_mode == 'ground':
      y_limit_value = np.amin(self.y_axis_data_1)
    else:
      y_limit_value = np.amax(self.y_axis_data_1)
    
    self._line1.set_data(self.x_axis_data, self.y_axis_data_1)
    # update axes 
    self._line1.axes.set_xlim(0, self.time + 1)
    if set_y_limit_mode == 'time':
      self._line1.axes.set_ylim(0, y_limit_value + 1)
    elif set_y_limit_mode == 'ground':
      if y_limit_value < 0:
        # negative 
        self._line1.axes.set_ylim(y_limit_value - 1, 0)
      else:
        # positive 
        self._line1.axes.set_ylim(0, y_limit_value + 1)
    else:
      self._line1.axes.set_ylim(0, y_limit_value)

    self._line1.figure.canvas.draw()

    # update time stamp
    self._time_stamp = timeStop 

  def update_canvas_2(self, mm_value, om_value, y_axes_lim_offset=1):
    """
    """
    timeStop = time.time()
    timeElapsed = timeStop - self._time_stamp
    self.time += timeElapsed
    
    self.x_axis_data = np.append(self.x_axis_data, self.time)
    # first line
    self.y_axis_data_1 = np.append(self.y_axis_data_1, mm_value)

    y_limit_value = np.amax(self.y_axis_data_1)
    
    self._line1.set_data(self.x_axis_data, self.y_axis_data_1)
    # update axes 
    self._line1.axes.set_xlim(0, self.time + 1)
    self._line1.axes.set_ylim(0, y_limit_value + y_axes_lim_offset)

    # update line 1
    self._line1.figure.canvas.draw()

    # second line
    self.y_axis_data_2 = np.append(self.y_axis_data_2, om_value)
    self._line2.set_data(self.x_axis_data, self.y_axis_data_2)

    # update line 2
    self._line2.figure.canvas.draw()

    # update time stamp
    self._time_stamp = timeStop 
    