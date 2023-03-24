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
#include <rtabmap_ros/SelectedPointCloud.h>
#include <rtabmap_ros/OctomapWithBBox.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point.h>

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
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

// system
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <math.h>


namespace rtabmap_ros
{

  class ObstaclesFeedbackProcessing : public nodelet::Nodelet
  {
  public:
    // pcl::visualization::PCLVisualizer viewer;

    ObstaclesFeedbackProcessing() : 
      g_map_base_(rtabmap::Transform::getIdentity()){}

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
      pointCloudPub_ = nh.advertise<rtabmap_ros::SelectedPointCloud>("obstacle_points_selected", 1);
      processingThread_ = boost::thread(boost::bind(&ObstaclesFeedbackProcessing::feedbackProcessingThread, this, rate));
    }

    void semanticObstacleFeedbackProcessingCallback(const rtabmap_ros::OctomapWithBBox &msg)
    {
      NODELET_DEBUG("Received Semantic Octomap message (size: %d bytes)", (int)msg.octomap.data.size());
      if (msg.octomap.binary)
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
      //rtabmap::SemanticColorOcTree *octomapPtr = NULL;
      UTimer timer;
      timer.start();

      boost::shared_ptr<rtabmap::SemanticColorOcTree> octreePtr(new rtabmap::SemanticColorOcTree(msg.octomap.resolution));
      //octomapPtr = new rtabmap::SemanticColorOcTree(msg.octomap.resolution);

      // deserializing message payload
      if (octreePtr)
      {
        std::stringstream datastream;
        if (msg.octomap.data.size() > 0)
        {
          datastream.write((const char *)&(msg.octomap.data[0]), msg.octomap.data.size());
          octreePtr->readData(datastream);
        }
      }

      obstacles_data_mtx_.lock();

      // smart pointer takes care of destroying the object
      octreePtr_ = octreePtr;
      min_bbox_range_map_.x = msg.min_bbox_range_map.x;
      min_bbox_range_map_.y = msg.min_bbox_range_map.y;
      min_bbox_range_map_.z = msg.min_bbox_range_map.z;
      
      max_bbox_range_map_.x = msg.max_bbox_range_map.x;
      max_bbox_range_map_.y = msg.max_bbox_range_map.y;
      max_bbox_range_map_.z = msg.max_bbox_range_map.z;

      bbox_bounds_sizes_.x = max_bbox_range_map_.x - min_bbox_range_map_.x;
      bbox_bounds_sizes_.y = max_bbox_range_map_.y - min_bbox_range_map_.y;
      bbox_bounds_sizes_.z = max_bbox_range_map_.z - min_bbox_range_map_.z;
      obstacles_data_mtx_.unlock();
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
        //  boost::shared_ptr<rtabmap::SemanticColorOcTree> octreePtr
        auto octreePtr = octreePtr_;
        cv::Point3f min_bbox_range_map = min_bbox_range_map_;
        cv::Point3f max_bbox_range_map = max_bbox_range_map_;
        cv::Point3f bbox_bounds_sizes = bbox_bounds_sizes_;
        obstacles_data_mtx_.unlock();

        UTimer total_timer;
        total_timer.start();

        int grid_size = 40;
        float grid_res_x = grid_size/bbox_bounds_sizes_.x;
        float grid_res_y = grid_size/bbox_bounds_sizes_.y;

        // int c = (grid_size/2)/grid_res;

        cv::Mat grid = cv::Mat::zeros(grid_size, grid_size, CV_32F);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_points(new pcl::PointCloud<pcl::PointXYZRGB>);
      
        //NODELET_WARN_STREAM("poseOrientationVector : " << poseOrientationVector_);
  
        // loop over all occupancy type groups
        if (octreePtr == nullptr)
        {
          rate.sleep();
          continue;
        }
        // NODELET_ERROR("START_LOOP");
        for (auto octreeNodeIter = octreePtr->begin(); octreeNodeIter != octreePtr->end(); ++octreeNodeIter)
        {
          if (octreePtr->isNodeOccupied(*octreeNodeIter))
          {
            cv::Point3f pt_map(octreeNodeIter.getX(), octreeNodeIter.getY(), octreeNodeIter.getZ());
            float x_map_grid = pt_map.x - min_bbox_range_map.x;
            float y_map_grid = pt_map.y - min_bbox_range_map.y;
            if (x_map_grid >= bbox_bounds_sizes.x || y_map_grid >= bbox_bounds_sizes.y ||
                x_map_grid < 0 || y_map_grid < 0)
            {
              continue;
            }
            int pixel_x = floor((pt_map.x - min_bbox_range_map.x) * grid_res_x);
            int pixel_y = floor((pt_map.y - min_bbox_range_map.y) * grid_res_y);
            // NODELET_ERROR("pt_map X: %f, Y: %f", pt_map.x, pt_map.y);
            // NODELET_ERROR("bbx_min X: %f, Y: %f", min_bbox_range_map.x, min_bbox_range_map.y);
            // NODELET_ERROR("pt minus bbx X: %f, Y: %f", pt_map.x - min_bbox_range_map.x, pt_map.y - min_bbox_range_map.y);
            // NODELET_ERROR("bbx_range X: %f, Y: %f", bbox_bounds_sizes.x, bbox_bounds_sizes.y);
            // NODELET_ERROR("grid_resolution: X: %f, Y: %f", grid_res_x, grid_res_y);
            // NODELET_ERROR("pixel X: %d, pixel Y: %d", pixel_x, pixel_y);

            if (grid.at<int>(pixel_x, pixel_y) == 0)
            {

              rtabmap::SemanticColorOcTreeNode::Color pt_color = octreeNodeIter->getColor();
              pcl::PointXYZRGB selected_point;

              selected_point.x = pt_map.x;
              selected_point.y = pt_map.y;
              selected_point.z = 0;

              selected_point.r = pt_color.r;
              selected_point.g = pt_color.g;
              selected_point.b = pt_color.b;

            
              selected_points->push_back(selected_point);

              grid.at<unsigned int>(pixel_x, pixel_y) = 1;

            }

          }

        }

        for (int i = grid_size-1; i >= 0; --i)
        {
          for (int j = 0; j < grid_size; ++j)
          {
            std::cout << grid.at<int>(i, j) << " ";
          }
          std::cout << std::endl;
        }

 
        sensor_msgs::Image pointImage;
        pcl::toROSMsg(*selected_points, pointImage);
        rtabmap_ros::SelectedPointCloud pointCloud_msg;
        pointCloud_msg.pointCloud = pointImage;
        pointCloudPub_.publish(pointCloud_msg);
        // // publish message
        // data_msg.header.stamp = ros::Time::now();
        // data_msg.header.frame_id = "base_link";
        // obstaclesFeedbackPub_.publish(data_msg);

        NODELET_ERROR("(obstacles processing) Semantic Octomap post-processing time = %0.4f sec", total_timer.ticks());

        // sleep for the rest of the time
        rate.sleep();
      }
    }

  private:
    // Publisher
    ros::Publisher obstaclesFeedbackPub_;
    ros::Publisher pointCloudPub_;

    // Subscriber
    ros::Subscriber semanticObstaclesProcessingSub_;
    ros::Subscriber poseSub_;

    rtabmap::Transform g_map_base_;
    boost::mutex g_map2base_mtx_;
    cv::Point3f poseOrientationVector_;
    cv::Point3f min_bbox_range_map_;
    cv::Point3f max_bbox_range_map_;
    cv::Point3f bbox_bounds_sizes_;
    boost::mutex obstacles_data_mtx_;

    boost::thread processingThread_;

    boost::shared_ptr<rtabmap::SemanticColorOcTree> octreePtr_;
    //std::map<int, std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>> > > obstacles_data_; // {occupancyType : {classId : (point cloud, list of nodeId)} }

  }; /* class ObstaclesFeedbackProcessing */

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesFeedbackProcessing, nodelet::Nodelet);

} /* namespace rtabmap_ros */