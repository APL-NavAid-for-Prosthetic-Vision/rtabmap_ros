# rtabmap_ros by JHUAPL
This is a wrapper of the original RTABMap_ros package based on the RTABMap_apl library.

Original RTABMap_ros : https://github.com/introlab/rtabmap_ros.git; branch: melodic-devel.
For Ubuntu 18.04

This modified wrapper to RTABMap_ros has been developed by JHUAPL.  
Developers:
Nicolas Norena-Acosta {nicolas.norena@jhuapl.edu}


### Dependencies

ROS packages :  
octomap_msgs, octomap_ros, octomap_rviz_plugins 

system libraries:  
opencv 3.4.17 (Tested, newer might work 3.4.17)  
libfmt-dev 

## Installation

```bash
#clone repo with submodules
git clone https://bitbucket.xrcs.jhuapl.edu/scm/slamr01/rtabmap_ros.git
git submodule init
git submodule update
```

- From your workspace initialize and configure the catkin project (make sure to have source the ROS environment 'source /opt/ros/melodic/setup.bash')

```bash
# system dependecies
$ sudo apt-get install -y libfmt-dev

$ catkin init
#if building python3
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
#otherwise
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release

# Building with CUDA support
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DOPENCV_CUDA=ON -DWITH_YAMLCPP=ON
catkin build rtabmap_ros 

# ** Optional flags:
# WITH_YAMLCPP : Needs to be enable for semantic segmentation mode.

# Building with graphic
catkin build rtabmap_ros --cmake-args -DRTABMAP_GUI=ON 
 
```
