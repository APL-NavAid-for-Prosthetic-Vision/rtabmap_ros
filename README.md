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
git clone https://bitbucket.xrcs.jhuapl.edu/scm/slamr01/rtabmap_ros.git
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

# Second, run build

# If building without RTABMap's custom graphics (default):
catkin build rtabmap_ros 

# If building with RTABMap's custom graphics:
catkin build rtabmap_ros --cmake-args -DRTABMAP_GUI=ON 
 
```
