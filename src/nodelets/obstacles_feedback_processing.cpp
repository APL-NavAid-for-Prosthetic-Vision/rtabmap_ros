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
#include <pcl/ModelCoefficients.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/search/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
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
    ObstaclesFeedbackProcessing() : g_map_base_(rtabmap::Transform::getIdentity()) {}

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

      g_map2base_mtx_.lock();
      if (g_map_base_.isNull() || g_map_base_.isIdentity())
      {
        NODELET_WARN("base_link pose has not updated, transform has null or it's an idntity");
        return;
      }
      // g_map_base : Transforms points from base frame to map frame
      rtabmap::Transform g_map_base = g_map_base_;
      g_map2base_mtx_.unlock();

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


      // g_base_map : transform pts from map frame to base frame
      rtabmap::Transform g_base_map = g_map_base.inverse();
      // converting semantic octomap to data structure
      for (auto octreeNodeIter = octomapPtr->begin(), end = octomapPtr->end(); octreeNodeIter != end; ++octreeNodeIter)
      {
        if (octomapPtr->isNodeOccupied(*octreeNodeIter))
        {
          // octomap::OcTreeKey key = octreeNodeIter.getKey();

          // transform point from map to the base frame 
          cv::Point3f pt_map(octreeNodeIter.getX(), octreeNodeIter.getY(), octreeNodeIter.getZ());
          cv::Point3f pt_base = rtabmap::util3d::transformPoint(pt_map, g_base_map);

          rtabmap::SemanticColorOcTreeNode::Color pt_color = octreeNodeIter->getColor();
          int occupancyType = octreeNodeIter->getOccupancyType();
          int referenceNodeId = octreeNodeIter->getNodeRefId();
          unsigned int classId = octreeNodeIter->getClassLabel();
          // cv::Point3f semanticObjColor = octreeNodeIter->getMaskColor();
          
          // pt of the bbx map w.r.t base
          pcl::PointXYZRGB ptTransformed;
          ptTransformed.x = pt_base.x;
          ptTransformed.y = pt_base.y;
          ptTransformed.z = pt_base.z;
          ptTransformed.r = pt_color.r;
          ptTransformed.g = pt_color.g;
          ptTransformed.b = pt_color.b;
      
          // add obstacles to data structure w.r.t to the base frame
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
      poseOrientationVector_ = rtabmap::util3d::transformPoint(cv::Point3f(1, 0, 0), g_base_map);
      obstacles_data_mtx_.unlock();

      delete octomapPtr;

      NODELET_DEBUG("(map conversion) Semantic Octomap post-processing time = %0.4f sec", timer.ticks());
    }

    void poseUpdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
    {
      g_map2base_mtx_.lock();
      // transforms points from base frame to map frame
      g_map_base_ = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
      g_map2base_mtx_.unlock();
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
         
          // for now only looking at static objects
          if (occupancyType != rtabmap::SemanticColorOcTreeNode::OccupancyType::kTypeStatic)
            continue;
        
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

            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointCloudPtr
            auto pointCloudPtr = std::get<0>(obstaclesIter->second);
            
            //NODELET_ERROR("classId: %d", classId);

            // std::vector<int> nodeIds
            auto nodeIds = std::get<1>(obstaclesIter->second);

            //NODELET_ERROR("pointcloud size: %d", (unsigned int)pointCloudPtr->size());
            // for (auto pointIter = pointCloudPtr->begin(); pointIter != pointCloudPtr->end(); ++pointIter){
            //   NODELET_ERROR("x: %f, y: %f, z: %f", pointIter->x, pointIter->y, pointIter->z);
            // }
            

            // identify the pose of the object
            // point cloud is w.r.t the base_link pose
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f(new pcl::PointCloud<pcl::PointXYZRGB>);
            // pcl::VoxelGrid<pcl::PointXYZRGB> vg;
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
            // vg.setInputCloud(pointCloudPtr);
            // vg.setLeafSize(0.01f, 0.01f, 0.01f);
            // vg.filter(*cloud_filtered);
            

            // std::cout << "PointCloud after filtering has: " << cloud_filtered->size() << "data points." << std::endl;
 
            // pcl::SACSegmentation<pcl::PointXYZRGB> seg;
            // pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            // pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
            // seg.setOptimizeCoefficients(true);
            // seg.setModelType(pcl::SACMODEL_SPHERE);
            // seg.setMethodType(pcl::SAC_RANSAC);
            // seg.setMaxIterations(100);
            // seg.setDistanceThreshold(0.02);

            // NODELET_ERROR("before while(cloud_filtered) loop!!!!");
            // int nr_points = (int) cloud_filtered->size();
            // while(cloud_filtered->size() > 0.3 * nr_points)
            // {
            //   seg.setInputCloud(pointCloudPtr);
            //   seg.segment(*inliers, *coefficients);
            //   if(inliers->indices.size() == 0)
            //   {
            //     std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            //     break;
            //   }
              
            //   pcl::ExtractIndices<pcl::PointXYZRGB> extract;
            //   extract.setInputCloud(cloud_filtered);
            //   extract.setIndices(inliers);
            //   extract.setNegative(false);

            //   extract.filter(*cloud_plane);
            //   std::cout << "PointCloud representing the planar component: " << cloud_plane->size() << " data points." << std::endl;

            //   extract.setNegative(true);
            //   extract.filter(*cloud_f);
            //   *cloud_filtered = *cloud_f;
            // }
            // NODELET_ERROR("after while(cloud_filtered) loop!!!!");
            // break;

            // pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
            // tree->setInputCloud(pointCloudPtr);

            // std::vector<pcl::PointIndices> cluster_indices;
            // pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
            // ec.setClusterTolerance(0.02);
            // ec.setMinClusterSize(3);
            // // ec.setMaxClusterSize(25000);
            // ec.setSearchMethod(tree);
            // ec.setInputCloud(pointCloudPtr);
            // ec.extract(cluster_indices);
            // NODELET_ERROR("Cluster indices size: %d", (unsigned int)cluster_indices.size());
           
            // for (const auto& cluster : cluster_indices)
            // {
            //   pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
            //   for (const auto& idx : cluster.indices)
            //   {
            //     cloud_cluster->push_back((*pointCloudPtr)[idx]);
            //   }
            //   cloud_cluster->width = cloud_cluster->size();
            //   cloud_cluster->height = 1;
            //   cloud_cluster->is_dense = true;

            //   std::cout << "PointCloud representing the cluster: " << cloud_cluster->size() << " datapoints." << std::endl;
            // }
            
            // obstacle msg filling in 
            
            //occupancy_msg.obstaclesData.push_back(obstacleDataMsg);
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

    rtabmap::Transform g_map_base_;
    boost::mutex g_map2base_mtx_;
    cv::Point3f poseOrientationVector_;
    boost::mutex obstacles_data_mtx_;

    boost::thread processingThread_;

    std::map<int, std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>> > > obstacles_data_; // {occupancyType : {classId : (point cloud, list of nodeId)} }

  }; /* class ObstaclesFeedbackProcessing */

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesFeedbackProcessing, nodelet::Nodelet);

} /* namespace rtabmap_ros */