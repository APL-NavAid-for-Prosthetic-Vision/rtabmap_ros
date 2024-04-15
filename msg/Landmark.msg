# landmark message 
#
#   id [int] of the landmark (unique identifier)
#   std::string description of the landmark
#   timeStamp [double] of the corresponding nodeId {signatureId}
#   nodeId is same as signatureId
#   landmarkPose : pose of landmark in camera frame
#       c++ type : rtabmap::Transform transform;
#   odometryPose : transform from camera frame to world frame
#       c++ type : rtabmap::Transform transform;
#   covariance :
#       c++ type : cv::Mat(6,6,CV_64FC1) covariance;
#
#   landmarkMapId is only need available when quering landmark
#
#   -------------------------------------------
#
#   landmarkPose_WorldCoords : pose of landmark in world frame (only used for service response)
#

int32 landmarkId
string description
float64 timeStamp
int32 nodeId
geometry_msgs/Transform landmarkPose
geometry_msgs/Transform odometryPose
float64[36] covariance
int32 landmarkMapId
float64 landmarkSize

geometry_msgs/Transform landmarkPose_WorldCoords
