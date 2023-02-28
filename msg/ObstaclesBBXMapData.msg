std_msgs/Header header

#
#  occupancyTypesId:
#
#  
#  occupancyTypesData: list of OccupancyData
#                 
#

uint32[] occupancyTypesId
OccupancyData[] occupancyTypesData

# transform from pose to map coords
geometry_msgs/Pose T_map_pose
