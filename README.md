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
opencv 3.4.10 (Tested, newer might work 3.4.x)  
libfmt-dev 

## Installation

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
catkin build rtabmap_ros --cmake-args -DOPENCV_CUDA=ON -DWITH_YAMLCPP=ON 

# Building with graphic
catkin build rtabmap_ros --cmake-args -DWITH_YAMLCPP=ON -DRTABMAP_GUI=ON 
 
```