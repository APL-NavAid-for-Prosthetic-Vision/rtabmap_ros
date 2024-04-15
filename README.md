# rtabmap_ros by JHUAPL
This is a wrapper of the original RTABMap_ros package based on the RTABMap_apl library.

Original RTABMap_ros : https://github.com/introlab/rtabmap_ros.git; branch: noetic-devel.
For Ubuntu 20.04

This modified wrapper to RTABMap_ros has been developed by JHUAPL.  
Developers:
Nicolas Norena-Acosta {nicolas.norena@jhuapl.edu}


### Dependencies

ROS packages :    
Octomap packages need to be from source:
- octomap_msgs: https://github.com/OctoMap/octomap_msgs.git (branch: melodic-devel) 
- octomap_ros: https://github.com/OctoMap/octomap_ros.git (branch: melodic-devel)
- octomap_rviz_plugins (if using RVIZ): https://github.com/OctoMap/octomap_rviz_plugins.git (branch: kinetic-devel)  
  
common packages:
ros-noetic-perception-pcl, ros-noetic-geometry2

```bash
sudo apt install ros-noetic-perception-pcl ros-noetic-geometry2 ros-noetic-move-base-msgs
```

system libraries:  
opencv 4.5.5 (build from source to enable cuda support)  
libfmt-dev 

```bash
sudo apt install libfmt-dev
```

## Installation

```bash
#clone repo with submodules
git clone https://gitlab.jhuapl.edu/slamr01/rtabmap_ros.git
git submodule init
git submodule update
```

- From your workspace initialize and configure the catkin project (make sure to have source the ROS environment 'source /opt/ros/melodic/setup.bash')

```bash

# First, set build configuration

# If building without semantic segmentation:
# catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# If building with semantic segmentation (with CUDA and yaml support)
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DOPENCV_CUDA=ON -DWITH_YAMLCPP=ON

# Or to specify python executable as well
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DOPENCV_CUDA=ON -DWITH_YAMLCPP=ON -DPYTHON_EXECUTABLE=/usr/bin/python3

# Second, run build

# If building without RTABMap's custom graphics (default):
catkin build rtabmap_ros 

# If building with RTABMap's custom graphics:
catkin build rtabmap_ros --cmake-args -DRTABMAP_GUI=ON 
 
```


## ROS Settings for Storing RTABMap Database to File on Shutdown

When mapping large maps, it may require a substantial period of time to write the rtabmap database to file on shutdown. Due to this extended write time, ROS may interrupt this process by sending the SIGKILL signal, causing the program to terminate immediately before the database file can be fully written.

When shutting down a ROS program, the roscore initally sends the SIGINT signal to all nodes to signal them to shutdown. If these nodes don't shutdown within a predefined period of time, roscore escalates this to the SIGTERM signal. If the nodes still do not all shutdown within another predefined period, roscore finally sends the SIGKILL signal which terminates the process immediately.

While it is possible to catch the SIGINT and SIGTERM signals with custom handlers, it is not possible to handle the SIGKILL signal. So the only way to prevent ROS from shutting down the process before the database write is complete is to prevent the SIGKILL signal from being issued. This can be done by increasing the timeouts for the SIGINT and SIGTERM signals. While there is no run-time configuration for changing these timeouts, the timeouts can be modified in ROS code. For ROS Noetic, the timeouts are specified here:

File: /opt/ros/noetic/lib/python3/dist-packages/roslaunch/nodeprocess.py

  DEFAULT_TIMEOUT_SIGINT  = 15.0 #seconds
  DEFAULT_TIMEOUT_SIGTERM = 2.0 #seconds

Increasing the DEFAULT_TIMEOUT_SIGINT timeout to a sufficiently large value will allow the database write to complete without being interrupted by the ROS shutdown.
