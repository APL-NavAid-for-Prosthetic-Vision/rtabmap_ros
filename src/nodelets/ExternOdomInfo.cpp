/*
	Johns Hopkins University

*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Transform.h>

#include <nav_msgs/Odometry.h>
#include <std_msgs/Header.h>

#include <rtabmap_ros/OdomInfo.h>
#include <rtabmap_ros/MsgConversion.h>


namespace rtabmap_ros
{

class ExternOdomInfo : public nodelet::Nodelet
{
public:
	//Constructor
	ExternOdomInfo():
		lastOdomStamp_(0)
	{}

	virtual ~ExternOdomInfo()
	{}

private:
	virtual void onInit()
	{
		ros::NodeHandle& nh = getNodeHandle(); //relative node handle (for setting parameters across multiple nodes)
		ros::NodeHandle& private_nh = getPrivateNodeHandle(); //private node handle

		// Subscribe
		odometry_sub_ = nh.subscribe("odom_in", 1, &ExternOdomInfo::callback, this);

		// Publish
		odomPub_ = private_nh.advertise<nav_msgs::Odometry>("odom", 1);
		odomInfoPub_ = private_nh.advertise<rtabmap_ros::OdomInfo>("odomInfo", 1);
	};

	void callback(const nav_msgs::OdometryConstPtr & odom)
	{
		if(odomPub_.getNumSubscribers())
		{
			//create a OdomInfo message
			if(odomInfoPub_.getNumSubscribers())
			{
				rtabmap_ros::OdomInfo odomInfoMsg;

				odomInfoMsg.header = odom->header;
				odomInfoMsg.stamp = rtabmap_ros::timestampFromROS(odom->header.stamp);
				if(lastOdomStamp_ == 0)
				{
					lastOdomStamp_ = rtabmap_ros::timestampFromROS(odom->header.stamp);
					lastOdomTF_ = rtabmap_ros::transformFromPoseMsg(odom->pose.pose);
				}
				odomInfoMsg.interval =  rtabmap_ros::timestampFromROS(odom->header.stamp) - lastOdomStamp_;
				odomInfoMsg.timeEstimation = odomInfoMsg.interval;
				memcpy(odomInfoMsg.covariance.data(), odom->twist.covariance.data(), 36*sizeof(double));
				// transformation of odom with respect to last odom pose
				rtabmap::Transform odomDiff = lastOdomTF_.inverse()*rtabmap_ros::transformFromPoseMsg(odom->pose.pose);
				NODELET_DEBUG("odomInfoMsg.transform: %s", odomDiff.prettyPrint().c_str());
				
				rtabmap_ros::transformToGeometryMsg(odomDiff, odomInfoMsg.transform);
				odomInfoPub_.publish(odomInfoMsg);
			}
			
			odomPub_.publish(odom);
			lastOdomStamp_ = rtabmap_ros::timestampFromROS(odom->header.stamp);
			lastOdomTF_ = rtabmap_ros::transformFromPoseMsg(odom->pose.pose);
		}
	}

	ros::Subscriber odometry_sub_;
	ros::Publisher odomPub_;
	ros::Publisher odomInfoPub_;
	double lastOdomStamp_;
	rtabmap::Transform lastOdomTF_;
};


PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ExternOdomInfo, nodelet::Nodelet);
}
