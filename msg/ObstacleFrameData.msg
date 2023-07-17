std_msgs/Header header

#
# This message holds obstacle data corresponding to one camera frame
# 
#  obstacleIds : list of each obstacle id (class id) which corresponds to set of points in the same index position
#                 of the variable pointClouds
#  pointClouds : list of point Clouds (set of points) belonging to class id. each point cloud is contained 
#                 in a sensor_msgs::Image so it can be used with cv_bridge.
#                 
#

# list of points depending on the encapsulation used in Rtabmap.
#   refer to function "laserScanFromPointCloud" in util3d.cpp  for details and 
#   function publishObstacleData in CoreWrapper.cpp cv_bridge conversion. 
#
#  depending on the image encapsulation used, a single point can consist of 3 to 7 dimensions.
#   the varable "encoding" can identify this encapsulation
#   for example)
#       pcl::PointXYZRGB uses 4 dimensions as follow:
#               [0] = x, [1] = y, [2] = z, [3] = b | g  << 8 | r << 16 
#       encoding = "32FC4"
#   
#     Note: sensor_msgs/Image.data stores the data as bytes, a 32FC4 would use 16 bytes for one encapsulated point.

int32 signatureId   # camera frame ID (for debug)
uint32[] obstacleIds
sensor_msgs/Image[] pointClouds

# transformation from base frame to camera frame
geometry_msgs/Pose T_cam_pts
