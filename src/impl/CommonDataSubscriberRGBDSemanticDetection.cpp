/*
*   Johns Hopkins University Applied Physics Laboratory
*   
*   This is based on the work from CommonDataSubscribedRGBD.cpp by Mathieu Labbe - IntRoLab - Universite de Sherbrooke
*   All rights reserved.
*/

#include <rtabmap_ros/CommonDataSubscriber.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap_ros/MsgConversion.h>
#include <cv_bridge/cv_bridge.h>

namespace rtabmap_ros {

void CommonDataSubscriber::setupRGBDSemanticDetectionCallbacks(
    ros::NodeHandle & nh,
    ros::NodeHandle & pnh,
    bool subscribeOdom,
    bool subscribeUserData,
    bool subscribeScan2d,
    bool subscribeScan3d,
    bool subscribeScanDesc,
    bool subscribeOdomInfo,
    int queueSize,
    bool approxSync)
{
    ROS_INFO("Setup rgbd callback");

	if(subscribeOdom || subscribeUserData || subscribeScan2d || subscribeScan3d || subscribeOdomInfo)
	{
		rgbdSemanticDetectionSubs_.resize(1);
		rgbdSemanticDetectionSubs_[0] = new message_filters::Subscriber<rtabmap_ros::RGBDSemanticDetectionImage>;
		rgbdSemanticDetectionSubs_[0]->subscribe(nh, "rgbd_semantic_detection_image", 1);
#ifdef RTABMAP_SYNC_USER_DATA
		if(subscribeOdom && subscribeUserData)
		{
			// TODO in future version supporting UserData
			
		}
		else
#endif
        if(subscribeOdom)
		{
            odomSub_.subscribe(nh, "odom", 1);
            if(subscribeOdomInfo)
			{
				subscribedToOdomInfo_ = true;
				odomInfoSub_.subscribe(nh, "odom_info", 1);
				SYNC_DECL3(rgbdSemanticDetectionOdomInfo, approxSync, queueSize, odomSub_, (*rgbdSemanticDetectionSubs_[0]), odomInfoSub_);
			}
			else
			{
				SYNC_DECL2(rgbdSemanticDetectionOdom, approxSync, queueSize, odomSub_, (*rgbdSemanticDetectionSubs_[0]));
			}
        }
        else
        {
            ROS_FATAL("Not supposed to be here!");
        }
        
    }
    else
    {
        //TODO: when supports it as a callback when odom comes from rtabmap 
        /* 
        rgbdSemanticDetectionSub_ = nh.subscribe("rgbd_semantic_detection_image", 1, &CommonDataSubscriber::rgbdCallback, this);

		subscribedTopicsMsg_ =
				uFormat("\n%s subscribed to:\n   %s",
				ros::this_node::getName().c_str(),
				rgbdSemanticDetectionSub_.getTopic().c_str());

        */
    }
    
}

// 1 RGBDSemanticDetection + Odom  
void CommonDataSubscriber::rgbdSemanticDetectionOdomCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDSemanticDetectionImageConstPtr& image1Msg)
{
    cv_bridge::CvImageConstPtr rgb, depth, semantic_mask;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth, semantic_mask);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null
	rtabmap_ros::OdomInfoConstPtr odomInfoMsg; // null

	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth,
			semantic_mask, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}

void CommonDataSubscriber::rgbdSemanticDetectionOdomInfoCallback(
		const nav_msgs::OdometryConstPtr & odomMsg,
		const rtabmap_ros::RGBDSemanticDetectionImageConstPtr& image1Msg,
		const rtabmap_ros::OdomInfoConstPtr& odomInfoMsg)
{
    cv_bridge::CvImageConstPtr rgb, depth, semantic_mask;
	rtabmap_ros::toCvShare(image1Msg, rgb, depth, semantic_mask);

	rtabmap_ros::UserDataConstPtr userDataMsg; // Null
	sensor_msgs::LaserScan scanMsg; // Null
	sensor_msgs::PointCloud2 scan3dMsg; // Null

	std::vector<rtabmap_ros::GlobalDescriptor> globalDescriptorMsgs;
	if(!image1Msg->global_descriptor.data.empty())
	{
		globalDescriptorMsgs.push_back(image1Msg->global_descriptor);
	}

	commonSingleDepthCallback(odomMsg, userDataMsg, rgb, depth,
			semantic_mask, image1Msg->rgb_camera_info, image1Msg->depth_camera_info,
			scanMsg, scan3dMsg, odomInfoMsg,
			globalDescriptorMsgs, image1Msg->key_points, image1Msg->points,
			rtabmap::uncompressData(image1Msg->descriptors));
}

}  /* namespace rtabmap_ros */