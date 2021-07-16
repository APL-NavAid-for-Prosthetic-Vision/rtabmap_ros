Header header

# lastNodeId is the last added node id (gets merged into when very similar)
# nodeId is the same as the signature id (actual id used the map)
#        
# rgbImage - visual image in RGB format (RAW format)
# depthImage - depth image in meters [m] 
# semanticMaskImage - semantic mask image corresponding to rgb image
# pose - pose of the current node id - includes map corrections
#
int32  lastNodeId
int32  nodeId
sensor_msgs/Image rgbImage
sensor_msgs/Image depthImage
sensor_msgs/Image semanticMaskImage
geometry_msgs/Pose pose

#
# occupancyMapkeys is the list of occupancy types {ground, ceiling, static, dynamic}
# occupancyMapObjectValue is the object classification id corresponding to in the used detection model 
#
string[] occupancyMapkeys
uint32[] occupancyMapObjectValue

#
#  objectsMapKeys is the object classification id corresponding to in the used detection model (keys)
#  objectsMapCloudValues is the list of poses  / pose correspondingto the object with respect to the current node
#
uint32[] objectsMapKeys
sensor_msgs/Image[] objectsMapCloudValues
