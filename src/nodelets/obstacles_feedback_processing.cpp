/*
 * 	Copyright 2023 The Johns Hopkins University
 *   Applied Physics Laboratory.  All rights reserved.
 *
 */

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include <rtabmap/utilite/UTimer.h>

#include <rtabmap/core/Transform.h>
#include <rtabmap/core/util3d_transforms.h>

// it is assumed the semantic octomap was compile as a dependency
#include <rtabmap/core/SemanticColorOcTree.h>
#include <rtabmap/core/SemanticOctoMap.h>

#include <rtabmap_ros/MsgConversion.h>
#include <rtabmap_ros/ObstaclesBBXMapData.h>
#include <rtabmap_ros/OccupancyData.h>
#include <rtabmap_ros/ObstacleData.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

// boost
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// pcl library
#include <pcl/point_cloud.h>
#include <pcl/pcl_base.h>
#include <pcl/point_types.h>

// cv2 library
#include <opencv2/core/types.hpp>

// system
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>


namespace rtabmap_ros
{

  class ObstaclesFeedbackProcessing : public nodelet::Nodelet
  {
  public:
    ObstaclesFeedbackProcessing() : t_mapToPose_(rtabmap::Transform::getIdentity()) {}

    virtual ~ObstaclesFeedbackProcessing() {}

  private:
    virtual void onInit()
    {
      ros::NodeHandle &nh = getNodeHandle();
      ros::NodeHandle &pnh = getPrivateNodeHandle();

      double rate = 2; // hz
      pnh.param("rate", rate, rate);

      // Subscriber
      semanticObstaclesProcessingSub_ = nh.subscribe("octomap_bbx_obstacles", 1, &ObstaclesFeedbackProcessing::semanticObstacleFeedbackProcessingCallback, this);
      poseSub_ = nh.subscribe("localization_pose", 1, &ObstaclesFeedbackProcessing::poseUpdateCallback, this);

      // publisher
      obstaclesFeedbackPub_ = nh.advertise<rtabmap_ros::ObstaclesBBXMapData>("obstacles_map_feedback", 1);

      processingThread_ = boost::thread(boost::bind(&ObstaclesFeedbackProcessing::feedbackProcessingThread, this, rate));
    }

    void semanticObstacleFeedbackProcessingCallback(const octomap_msgs::OctomapConstPtr &msg)
    {
      NODELET_DEBUG("Received Semantic Octomap message (size: %d bytes)", (int)msg->data.size());

      if (msg->binary)
      {
        NODELET_ERROR("Received Semantic Octomap message does not have the full map");
        return;
      }

      pose_mtx_.lock();
      if (t_mapToPose_.isNull() || t_mapToPose_.isIdentity())
      {
        NODELET_WARN("base_link pose has not updated, transform has null or it's an idntity");
        return;
      }
      rtabmap::Transform t_mapToPose = t_mapToPose_;
      pose_mtx_.unlock();

      // creating octree
      rtabmap::SemanticColorOcTree *octomapPtr = NULL;
      UTimer timer;
      timer.start();

      octomapPtr = new rtabmap::SemanticColorOcTree(msg->resolution);

      // deserializing message payload
      if (octomapPtr)
      {
        std::stringstream datastream;
        if (msg->data.size() > 0)
        {
          datastream.write((const char *)&(msg->data[0]), msg->data.size());
          octomapPtr->readData(datastream);
        }
      }

      obstacles_data_mtx_.lock();
      rtabmap::Transform t_poseToMap = t_mapToPose.inverse();
      // converting semantic octomap to data structure
      for (auto octreeNodeIter = octomapPtr->begin(), end = octomapPtr->end(); octreeNodeIter != end; ++octreeNodeIter)
      {
        if (octomapPtr->isNodeOccupied(*octreeNodeIter))
        {
          // octomap::OcTreeKey key = octreeNodeIter.getKey();

          // transform point w.r.t base_link pose
          cv::Point3f pt_from_map(octreeNodeIter.getX(), octreeNodeIter.getY(), octreeNodeIter.getZ());
          cv::Point3f pt_from_pose = rtabmap::util3d::transformPoint(pt_from_map, t_poseToMap);

          rtabmap::SemanticColorOcTreeNode::Color pt_color = octreeNodeIter->getColor();
          int occupancyType = octreeNodeIter->getOccupancyType();
          int referenceNodeId = octreeNodeIter->getNodeRefId();
          unsigned int classId = octreeNodeIter->getClassLabel();
          // cv::Point3f semanticObjColor = octreeNodeIter->getMaskColor();

          pcl::PointXYZRGB ptTransformed(pt_from_pose.x,
                                         pt_from_pose.y,
                                         pt_from_pose.z);
          ptTransformed.r = pt_color.r;
          ptTransformed.g = pt_color.g;
          ptTransformed.b = pt_color.b;

          // add obstacles to data structure w.r.t to the base_link pose
          auto occupancy_iter = obstacles_data_.find(occupancyType);
          if (occupancy_iter == obstacles_data_.end())
          {
            // add new occupancy type group
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
            pointCloudPtr->push_back(ptTransformed);

            std::vector<int> nodeIds;
            nodeIds.push_back(referenceNodeId);
            // all data can be added
            std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>>> value;
            value.insert({classId, std::make_tuple(pointCloudPtr, nodeIds)});
            obstacles_data_.insert({occupancyType, value});
          }
          else
          {
            // occupancy type was already added.
            auto classIdIter = occupancy_iter->second.find(classId);
            if (classIdIter == occupancy_iter->second.end())
            {
              // add new class id group
              pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr(new pcl::PointCloud<pcl::PointXYZRGB>);
              pointCloudPtr->push_back(ptTransformed);

              std::vector<int> nodeIds;
              nodeIds.push_back(referenceNodeId);

              occupancy_iter->second.insert({classId, std::make_tuple(pointCloudPtr, nodeIds)});
            }
            else
            {
              // class id was already added, the point can be just added
              auto pointCloudPtr = std::get<0>(classIdIter->second);
              auto nodeIds = std::get<1>(classIdIter->second);

              pointCloudPtr->push_back(ptTransformed);
              nodeIds.push_back(referenceNodeId);
            }
          }
        }
      }
      // update orientation vector
      poseOrientationVector_ = rtabmap::util3d::transformPoint(cv::Point3f(1, 0, 0), t_poseToMap);
      obstacles_data_mtx_.unlock();

      delete octomapPtr;

      NODELET_DEBUG("(map conversion) Semantic Octomap post-processing time = %0.4f sec", timer.ticks());
    }

    void poseUpdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
    {
      pose_mtx_.lock();
      t_mapToPose_ = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
      pose_mtx_.unlock();
    }

    void feedbackProcessingThread(const double &updateRateHz)
    {
      ros::Rate rate(updateRateHz);

      while (ros::ok()) 
      {
        obstacles_data_mtx_.lock();

        UTimer total_timer;
        total_timer.start();

        //NODELET_WARN_STREAM("poseOrientationVector : " << poseOrientationVector_);
        rtabmap_ros::ObstaclesBBXMapData data_msg;

        // loop over all occupancy type groups
        for (auto occupancyIter = obstacles_data_.begin(); occupancyIter != obstacles_data_.end(); ++occupancyIter)
        {
          int occupancyType = occupancyIter->first;
          // add data to msg
          data_msg.occupancyTypesId.push_back(occupancyType);
          
          rtabmap_ros::OccupancyData occupancy_msg;
          // loop over all obstacles classes
          for (auto obstaclesIter = occupancyIter->second.begin(); obstaclesIter != occupancyIter->second.end(); ++obstaclesIter)
          {
            unsigned int classId = obstaclesIter->first;

            //occupancy_msg.obstaclesName.push_back();   /// TODO: load id name and look it up
            occupancy_msg.obstaclesId.push_back(classId);
            
            // obstacle message data
            rtabmap_ros::ObstacleData obstacleDataMsg;

            auto pointCloudPtr = std::get<0>(obstaclesIter->second);
            auto nodeIds = std::get<1>(obstaclesIter->second);

            // identify the pose of the object
            // point cloud is w.r.t the base_link pose



            // obstacle msg filling in 
            //obstacleDataMsg.pose
          
            occupancy_msg.obstaclesData.push_back(obstacleDataMsg);
          }
          // insert the occupancy object to the msg
          data_msg.occupancyTypesData.push_back(occupancy_msg);
        }
        obstacles_data_mtx_.unlock();

        // publish message
        data_msg.header.stamp = ros::Time::now();
        data_msg.header.frame_id = "base_link";
        obstaclesFeedbackPub_.publish(data_msg);

        NODELET_DEBUG("(obstacles processing) Semantic Octomap post-processing time = %0.4f sec", total_timer.ticks());

        // sleep for the rest of the time
        rate.sleep();
      }
    }

  private:
    // Publisher
    ros::Publisher obstaclesFeedbackPub_;

    // Subscriber
    ros::Subscriber semanticObstaclesProcessingSub_;
    ros::Subscriber poseSub_;

    rtabmap::Transform t_mapToPose_;
    boost::mutex pose_mtx_;
    cv::Point3f poseOrientationVector_;
    boost::mutex obstacles_data_mtx_;

    boost::thread processingThread_;

    std::map<int, std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>> > > obstacles_data_; // {occupancyType : {classId : (point cloud, list of nodeId)} }

  }; /* class ObstaclesFeedbackProcessing */

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesFeedbackProcessing, nodelet::Nodelet);

} /* namespace rtabmap_ros */