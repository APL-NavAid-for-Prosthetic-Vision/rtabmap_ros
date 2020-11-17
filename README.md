# rtabmap_ros 
This is a wrapper to the RTABMap_ros package with linking to the RTABMap_apl library. Furthermore, it was tested to build on ROS Melodic with python3. The ROS Melodic was built from source. 

Original RTABMap_ros : https://github.com/introlab/rtabmap_ros.git ; branch: melodic-devel

This modified wrapper to RTABMap_ros has been developed by JHUAPL.  
Developers:
Nicolas Norena-Acosta {nicolas.norena@jhuapl.edu}


### Dependencies
This depends on a number of packages which could installed directly from the binaries in the ubuntu repository.
In the case of have built from source to support python3, see the documenation for building ros melodic from source which
includes all the dependecies for this package.

## Installation
This wrapper links with the RTABMap_apl library, which it's required.

- From your workspace init and configure the catkin project (make sure to have source the ROS environment 'source /opt/ros/melodic/setup.bash')

```bash
$ catkin init
#if building python3
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3
#otherwise
$ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build rtabmap_apl
```