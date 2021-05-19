# landmark message 
#
#   int id of the landmark (unique identifier)
#   std::string description of the landmark
#   double timeStamp of the corresponding poseId {signatureId}
#   int poseId same as signatureId
#   transform : from landmark to camera pose
#       c++ type : rtabmap::Transform transform;
#   covariance :
#       c++ type : cv::Mat(6,6,CV_64FC1) covariance;
#
#   landmarkMapId is only need available when quering landmark

int32 landmarkId
string description
float64 timeStamp
int32 signatureId
geometry_msgs/Transform transform
float64[36] covariance
int32 landmarkMapId
