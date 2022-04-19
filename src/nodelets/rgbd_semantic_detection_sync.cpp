/*
* 	Copyright 2021 The Johns Hopkins University
*   Applied Physics Laboratory.  All rights reserved.
*
*/

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/subscriber.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>

#include <boost/thread.hpp>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Compression.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/utilite/UConversion.h>

#include <rtabmap_ros/RGBDSemanticDetection.h>
#include <rtabmap_ros/MsgConversion.h>

#include <objectrecognition/SegmentImage.h>

#include <opencv2/imgcodecs.hpp>

namespace rtabmap_ros
{

class RGBDSemanticDetectionSync : public nodelet::Nodelet
{
public:
	RGBDSemanticDetectionSync() :
		//compressedRate_(0),
		warningThread_(0),
		callbackCalled_(false),
		approxSyncDepth_(0),
		exactSyncDepth_(0)
	{}

	virtual ~RGBDSemanticDetectionSync()
	{
		if(approxSyncDepth_)
			delete approxSyncDepth_;
		if(exactSyncDepth_)
			delete exactSyncDepth_;

		if(warningThread_)
		{
			callbackCalled_=true;
			warningThread_->join();
			delete warningThread_;
		}
	}

private:
	virtual void onInit()
	{
		ros::NodeHandle & nh = getNodeHandle();
		ros::NodeHandle & pnh = getPrivateNodeHandle();

		int queueSize = 10;
		bool approxSync = true;
		computeOdomInfo_ = true;
		pnh.param("approx_sync", approxSync, approxSync);
		pnh.param("queue_size", queueSize, queueSize);
		pnh.param("compute_odom_info", computeOdomInfo_, computeOdomInfo_);
		//pnh.param("compressed_rate", compressedRate_, compressedRate_);

		NODELET_INFO("%s: approx_sync = %s", getName().c_str(), approxSync?"true":"false");
		NODELET_INFO("%s: queue_size  = %d", getName().c_str(), queueSize);
		NODELET_INFO("%s: compute_odom_info  = %s", getName().c_str(), computeOdomInfo_?"true":"false");
		//NODELET_INFO("%s: compressed_rate = %f", getName().c_str(), compressedRate_);

		rgbdSemanticDetectionPub_ = nh.advertise<rtabmap_ros::RGBDSemanticDetection>("rgbd_semantic_detection_msg", 1);

		if(approxSync)
		{
			approxSyncDepth_ = new message_filters::Synchronizer<MyApproxSyncDepthPolicy>(MyApproxSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_, odomSub_);
			approxSyncDepth_->registerCallback(boost::bind(&RGBDSemanticDetectionSync::callback, this, _1, _2, _3, _4));
		}
		else
		{
			exactSyncDepth_ = new message_filters::Synchronizer<MyExactSyncDepthPolicy>(MyExactSyncDepthPolicy(queueSize), imageSub_, imageDepthSub_, cameraInfoSub_, odomSub_);
			exactSyncDepth_->registerCallback(boost::bind(&RGBDSemanticDetectionSync::callback, this, _1, _2, _3, _4));
		}

		ros::NodeHandle rgb_nh(nh, "rgb");
		ros::NodeHandle depth_nh(nh, "depth");
		ros::NodeHandle rgb_pnh(pnh, "rgb");
		ros::NodeHandle depth_pnh(pnh, "depth");
		image_transport::ImageTransport rgb_it(rgb_nh);
		image_transport::ImageTransport depth_it(depth_nh);
		image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
		image_transport::TransportHints hintsDepth("raw", ros::TransportHints(), depth_pnh);

		imageSub_.subscribe(rgb_it, rgb_nh.resolveName("image"), 1, hintsRgb);
		imageDepthSub_.subscribe(depth_it, depth_nh.resolveName("image"), 1, hintsDepth);
		cameraInfoSub_.subscribe(rgb_nh, "camera_info", 1);
		odomSub_.subscribe(nh, "odom", 10);

		std::string subscribedTopicsMsg = uFormat("\n%s subscribed to (%s sync):\n   %s,\n   %s,\n   %s,\n	%s",
							getName().c_str(),
							approxSync?"approx":"exact",
							imageSub_.getTopic().c_str(),
							imageDepthSub_.getTopic().c_str(),
							cameraInfoSub_.getTopic().c_str(),
							odomSub_.getTopic().c_str());

		objRecognitionSemanticDetectionClient_ = nh.serviceClient<objectrecognition::SegmentImage>("/objectrecognition/segmentation");

		warningThread_ = new boost::thread(boost::bind(&RGBDSemanticDetectionSync::warningLoop, this, subscribedTopicsMsg, approxSync));
		NODELET_INFO("%s", subscribedTopicsMsg.c_str());
	}

	void warningLoop(const std::string & subscribedTopicsMsg, bool approxSync)
	{
		ros::Duration r(5.0);
		while(!callbackCalled_)
		{
			r.sleep();
			if(!callbackCalled_)
			{
				ROS_WARN("%s: Did not receive data since 5 seconds! Make sure the input topics are "
						"published (\"$ rostopic hz my_topic\") and the timestamps in their "
						"header are set. %s%s",
						getName().c_str(),
						approxSync?"":"Parameter \"approx_sync\" is false, which means that input "
							"topics should have all the exact timestamp for the callback to be called.",
						subscribedTopicsMsg.c_str());
			}
		}
	}

	void computeOdomInfo(const nav_msgs::OdometryConstPtr& odom, rtabmap_ros::OdomInfo & odomInfoMsg)
	{
		odomInfoMsg.header = odom->header;
		odomInfoMsg.interval = rtabmap_ros::timestampFromROS(odom->header.stamp) - lastOdomStamp_;
		odomInfoMsg.timeEstimation = (float) rtabmap_ros::timestampFromROS(odom->header.stamp);

		memcpy(odomInfoMsg.covariance.data(), odom->pose.covariance.data(), 36*sizeof(double));

		rtabmap::Transform odomTransform = rtabmap_ros::transformFromPoseMsg(odom->pose.pose);
		rtabmap_ros::transformToGeometryMsg(odomTransform, odomInfoMsg.transform);
	}

	void callback(
			  const sensor_msgs::ImageConstPtr& image,
			  const sensor_msgs::ImageConstPtr& depth,
			  const sensor_msgs::CameraInfoConstPtr& cameraInfo,
			  const nav_msgs::OdometryConstPtr& odom)
	{
		callbackCalled_ = true;
		if(timeLastPublished_.toSec() == 0.0)
		{
			timeLastPublished_ = ros::Time::now();
		}

		if(lastOdomStamp_ == 0.0)
		{
			lastOdomStamp_ = rtabmap_ros::timestampFromROS(odom->header.stamp);
		}

		if(rgbdSemanticDetectionPub_.getNumSubscribers())
		{
			double rgbStamp = image->header.stamp.toSec();
			double depthStamp = depth->header.stamp.toSec();
			double infoStamp = cameraInfo->header.stamp.toSec();
			double odomStamp = odom->header.stamp.toSec();

			rtabmap_ros::RGBDSemanticDetection msg;
			msg.header.frame_id = cameraInfo->header.frame_id;

			// Down the line, RTabMap ultimately uses the rgb image timestamp
			// for the consolidated data frame, so use the same timestamp here
			// for the whole message
			msg.header.stamp = image->header.stamp;
			//msg.header.stamp = image->header.stamp > depth->header.stamp?image->header.stamp:depth->header.stamp;
						
			msg.rgb_camera_info = *cameraInfo;
			msg.depth_camera_info = *cameraInfo;
			
			cv::Mat rgbMat;
			cv::Mat depthMat;
			cv_bridge::CvImageConstPtr imagePtr = cv_bridge::toCvShare(image);
			cv_bridge::CvImageConstPtr imageDepthPtr = cv_bridge::toCvShare(depth);
			rgbMat = imagePtr->image;
			depthMat = imageDepthPtr->image;

			if(rgbdSemanticDetectionPub_.getNumSubscribers())
			{
				// rgb image
				cv_bridge::CvImage cvImg;
				cvImg.header = image->header;
				cvImg.image = rgbMat;
				cvImg.encoding = image->encoding;
				cvImg.toImageMsg(msg.rgb);

				// depth image
				cv_bridge::CvImage cvDepth;
				cvDepth.header = depth->header;
				// set timestamp to the same as rgb image to avoid a local transformation update 
				// later on (in MsgConversion.cpp::convertRGBDMsgs()) due to a timestamp difference				
				cvDepth.header.stamp = image->header.stamp;		
				cvDepth.image = depthMat;
				cvDepth.encoding = depth->encoding;
				cvDepth.toImageMsg(msg.depth);

				// object recognition client server call - blocks until it response 
				//  the RGB image is encoded in rgb - server needs it in rgb
				objectrecognition::SegmentImage objRecognSegImgMsg;
				cvImg.toImageMsg(objRecognSegImgMsg.request.image);
				cvDepth.toImageMsg(objRecognSegImgMsg.request.depth);
				objRecognSegImgMsg.request.odom = *odom;
				objRecognSegImgMsg.request.cameraInfo = *cameraInfo;
				ros::Time start_time = ros::Time::now();
				// request server in object recognition node a corresponding the semantic segmentation mask
				objRecognitionSemanticDetectionClient_.call(objRecognSegImgMsg);
				
				double elapse_time = ros::Time::now().toSec() - start_time.toSec();
				//NODELET_INFO("object recognition client call: elapse_time= %f", elapse_time);
				
				if(objRecognSegImgMsg.response.output.data.empty())
				{
					NODELET_WARN(" rgbd_semantic_detection-sync.cpp :: Failed to recieve the semantic segmentation from object recognition node");
				}
				else
				{
					// the semantic segmentation mask was received 
					cv::Mat semanticMaskMat;
					cv_bridge::CvImageConstPtr cvSemanticMaskPtr = cv_bridge::toCvCopy(objRecognSegImgMsg.response.output);
					semanticMaskMat = cvSemanticMaskPtr->image;				

					cv_bridge::CvImage cvSemanticImg;
					cvSemanticImg.header.stamp = depth->header.stamp;
					cvSemanticImg.header.seq = image->header.seq;
					cvSemanticImg.header.frame_id = image->header.frame_id;
					cvSemanticImg.image = semanticMaskMat;
					cvSemanticImg.encoding = objRecognSegImgMsg.response.output.encoding;
					cvSemanticImg.toImageMsg(msg.semantic_mask);

					// copy odom msg with msg out
					msg.odom = *odom;
					
					// compute odomInfo
					if(computeOdomInfo_)
					{
						computeOdomInfo(odom, msg.odomInfo);
					}
										 
					// publish RGBDSemanticDetection
					rgbdSemanticDetectionPub_.publish(msg);
					// statistics of the node
					elapse_time = ros::Time::now().toSec() - timeLastPublished_.toSec();
					NODELET_INFO("RGBD semantic Msg: elapse_time= %f (frq:%.2f Hz)", elapse_time, 1/elapse_time);
				}	
			}

			if(rgbStamp != image->header.stamp.toSec() || depthStamp != depth->header.stamp.toSec())
			{
				NODELET_ERROR("Input stamps changed between the beginning and the end of the callback! Make "
						"sure the node publishing the topics doesn't override the same data after publishing them. A "
						"solution is to use this node within another nodelet manager. Stamps: "
						"rgb=%f->%f depth=%f->%f",
						rgbStamp, image->header.stamp.toSec(),
						depthStamp, depth->header.stamp.toSec());
			}

			lastOdomStamp_ = rtabmap_ros::timestampFromROS(odom->header.stamp);
			timeLastPublished_ = ros::Time::now();
		}
	}

private:
	boost::thread * warningThread_;
	bool callbackCalled_;
	bool computeOdomInfo_;

	ros::Time timeLastPublished_;
	double lastOdomStamp_;
	
	ros::Publisher rgbdSemanticDetectionPub_;
	ros::ServiceClient objRecognitionSemanticDetectionClient_;

	image_transport::SubscriberFilter imageSub_;
	image_transport::SubscriberFilter imageDepthSub_;
	message_filters::Subscriber<sensor_msgs::CameraInfo> cameraInfoSub_;
	message_filters::Subscriber<nav_msgs::Odometry> odomSub_;

	typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MyApproxSyncDepthPolicy;
	message_filters::Synchronizer<MyApproxSyncDepthPolicy> * approxSyncDepth_;

	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo, nav_msgs::Odometry> MyExactSyncDepthPolicy;
	message_filters::Synchronizer<MyExactSyncDepthPolicy> * exactSyncDepth_;
};

PLUGINLIB_EXPORT_CLASS(rtabmap_ros::RGBDSemanticDetectionSync, nodelet::Nodelet);
}

