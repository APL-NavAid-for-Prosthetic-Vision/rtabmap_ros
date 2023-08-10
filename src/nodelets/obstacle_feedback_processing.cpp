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
#include <rtabmap_ros/ObstacleMapData.h>
#include <rtabmap_ros/ObstaclesLocalMap.h>
#include <rtabmap_ros/OctomapWithBBox.h>

#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>

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
#include <cv_bridge/cv_bridge.h>

#include<opencv2/opencv.hpp>

// system
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <math.h>
#include <string.h>


namespace rtabmap_ros
{

  class ObstacleFeedbackProcessing : public nodelet::Nodelet
  {

  private:
    // Publisher
    //ros::Publisher obstaclesFeedbackPub_;
    ros::Publisher connectedPointCloudPub_;

    // Subscriber
    ros::Subscriber semanticObstaclesProcessingSub_;
    ros::Subscriber poseSub_;

    // rtabmap::Transform g_map_base_;
    // boost::mutex g_map2base_mtx_;

    boost::mutex obstacles_data_mtx_;

    cv::Point3f min_bbox_range_map_;
    cv::Point3f max_bbox_range_map_;
    cv::Point3f bbox_bounds_sizes_;
    
    double grid_resolution_;
    boost::thread processingThread_;

    boost::shared_ptr<rtabmap::SemanticColorOcTree> octreePtr_;
    //std::map<int, std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>> > > obstacles_data_; // {occupancyType : {classId : (point cloud, list of nodeId)} }
    int min_connected_component_size_;


  public:
    // pcl::visualization::PCLVisualizer viewer;

    ObstacleFeedbackProcessing() {}
      //: g_map_base_(rtabmap::Transform::getIdentity()){}

    virtual ~ObstacleFeedbackProcessing() {}


  private:
    virtual void onInit()
    {
      ros::NodeHandle &nh = getNodeHandle();
      ros::NodeHandle &pnh = getPrivateNodeHandle();

      double rate = 2; // hz
      pnh.param("rate", rate, rate);
      pnh.param("obstacle_feedback_grid_resolution", grid_resolution_, 10.0);
      pnh.param("obstacle_feedback_min_connected_component_size", min_connected_component_size_, 5);

      // subscribers
      semanticObstaclesProcessingSub_ = nh.subscribe("octomap_with_bbx_obstacles", 1, &ObstacleFeedbackProcessing::semanticObstacleFeedbackProcessingCallback, this);
      //poseSub_ = nh.subscribe("localization_pose", 1, &ObstacleFeedbackProcessing::poseUpdateCallback, this);

      // publishers
      //obstaclesFeedbackPub_ = nh.advertise<rtabmap_ros::ObstaclesBBXMapData>("obstacles_map_feedback", 1);
      connectedPointCloudPub_ = nh.advertise<rtabmap_ros::ObstaclesLocalMap>("obstacles_local_map", 1);

      // child threads
      processingThread_ = boost::thread(boost::bind(&ObstacleFeedbackProcessing::feedbackProcessingThread, this, rate));
    }

    void semanticObstacleFeedbackProcessingCallback(const rtabmap_ros::OctomapWithBBox &msg)
    {
      //
      // This callback deserializes the octomap and stores it to a local file.
      // It is decoupled from the octomap processing thread so that deserialization and processing
      // of the obstacle data may occur in parallel to improve throughput and latency.
      //

      NODELET_DEBUG("Received Semantic Octomap message (size: %d bytes)", (int)msg.octomap.data.size());
      if (msg.octomap.binary)
      {
        NODELET_ERROR("Received Semantic Octomap message does not have the full map");
        return;
      }

      // g_map2base_mtx_.lock();
      // if (g_map_base_.isNull() || g_map_base_.isIdentity())
      // {
      //   NODELET_WARN("base_link pose has not updated, transform has null or it's an idntity");
      //   return;
      // }
      // // g_map_base : Transforms points from base frame to map frame
      // rtabmap::Transform g_map_base = g_map_base_;
      // g_map2base_mtx_.unlock();

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

    // void poseUpdateCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
    // {
    //   g_map2base_mtx_.lock();
    //   // transforms points from base frame to map frame
    //   g_map_base_ = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
    //   g_map2base_mtx_.unlock();
    // }


    void feedbackProcessingThread(const double &updateRateHz)
    {
      //
      // This routine flattens the obstacle data to 2D and applies connected
      // component filtering on the 2D obstacle grid
      //

      //
      // Note: the obstacle grid created in this routine uses standard image
      //       coordinates (X - right, Y - down) and the map coordinate points are
      //       mapped directly to these axis; therefore, the generated grid does not 
      //       show a top-down view of the map it ends up being a bottom-up view of the map.
      //

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

        int grid_cols = floor(grid_resolution_ * bbox_bounds_sizes_.x);
        int grid_rows = floor(grid_resolution_ * bbox_bounds_sizes_.y);
        
        cv::Mat grid = cv::Mat::zeros(grid_rows, grid_cols, CV_8U);
        cv::Mat connected_grid;
        cv::Mat stats;
        cv::Mat centroids;
        std::map<int, cv::Point3f> map_coord2point; // flattened index to point
  
        if (octreePtr == nullptr)
        {
          rate.sleep();
          continue;
        }

        // for each point in the octree
        for (auto octreeNodeIter = octreePtr->begin(); octreeNodeIter != octreePtr->end(); ++octreeNodeIter)
        {
          if (octreePtr->isNodeOccupied(*octreeNodeIter))
          {
            // get map coordinate of this point
            cv::Point3f pt_map(octreeNodeIter.getX(), octreeNodeIter.getY(), octreeNodeIter.getZ());

            // compute local grid coordinate of this point
            float pt_grid_x = pt_map.x - min_bbox_range_map.x; // meters
            float pt_grid_y = pt_map.y - min_bbox_range_map.y; // meters
            int pixel_x = floor(pt_grid_x * grid_resolution_); // pixels
            int pixel_y = floor(pt_grid_y * grid_resolution_); // pixels
            if (pixel_x >= grid_cols || pixel_y >= grid_rows || pixel_x < 0 || pixel_y < 0)
            {
              continue;
            }

            // set occupancy for this grid location if not already set
            if (grid.at<uint8_t>(pixel_y, pixel_x) == 0)
            {
              // get map location for the center of the grid pixel
              cv::Point3f pt_map_for_grid;
              pt_map_for_grid.x = ((pixel_x + 0.5) / grid_resolution_) + min_bbox_range_map.x; // pt_map.x;
              pt_map_for_grid.y = ((pixel_y + 0.5) / grid_resolution_) + min_bbox_range_map.y; // pt_map.y;
              pt_map_for_grid.z = 0.;

              // store map point for this grid location
              int flattened_coordinate = pixel_y * grid_cols + pixel_x;
              map_coord2point[flattened_coordinate] = pt_map_for_grid;
              
              // set grid occupancy
              grid.at<uint8_t>(pixel_y, pixel_x) = 1;
            }
          }
        }

        // this sets the cv::Mat parameter types to:
        //   connected_grid:  CV_32S (int32)
        //   stats:           CV_32S (int32)
        int num_components = cv::connectedComponentsWithStats(grid, connected_grid, stats, centroids, 8, CV_32S);

        // remove connected components that are too small
        //   Note: label 0 is the background label
        //
        // identify connected component to be removed
        std::set<int> labels_to_remove;
        for (int label = 1; label < num_components; ++label)
        { 
          if (stats.at<int>(label, cv::CC_STAT_AREA) < min_connected_component_size_)
          {
            labels_to_remove.insert(label);
          }
        }
        // remove the identified components by creating a new filtered grid
        cv::Mat filtered_grid = connected_grid.clone();
        // for each column and row of the grid
        for(int i = 0; i < filtered_grid.rows; i++)
        {
            int32_t* filtered_row = filtered_grid.ptr<int32_t>(i);
            for(int j = 0; j < filtered_grid.cols; j++)
            {
              // if the component at this grid position is flagged for removal
              // then set grid pixel to background and remove the stored map point
              if (labels_to_remove.count(filtered_row[j]))
              {
                filtered_row[j] = 0;
                int flattened_coordinate = j * grid_cols + i;
                map_coord2point.erase(flattened_coordinate);
              }
            }
        }

        if (map_coord2point.size() > 0)
        {
          rtabmap_ros::ObstaclesLocalMap connected_point_cloud_msg;
          for (auto iterator : map_coord2point)
          {
            int flattened_coord = iterator.first;
            cv::Point3f point = iterator.second;
            geometry_msgs::Point32 point_msg;
            point_msg.x = point.x;
            point_msg.y = point.y;
            point_msg.z = point.z;

            int point_row = floor(flattened_coord / grid_cols);
            int point_col = flattened_coord % grid_cols;
            int component_label = connected_grid.at<int>(point_row, point_col);
            connected_point_cloud_msg.point_cloud.points.push_back(point_msg);
            connected_point_cloud_msg.point_connected_component.push_back(component_label);
            connected_point_cloud_msg.point_row.push_back(point_row);
            connected_point_cloud_msg.point_col.push_back(point_col);
          }
          connected_point_cloud_msg.grid_resolution_per_meter = grid_resolution_;
          connected_point_cloud_msg.grid_cols = grid_cols;
          connected_point_cloud_msg.grid_rows = grid_rows;
          connected_point_cloud_msg.grid_origin_x = min_bbox_range_map.x;
          connected_point_cloud_msg.grid_origin_y = min_bbox_range_map.y;

          connectedPointCloudPub_.publish(connected_point_cloud_msg);
        }

        // debug print
        // std::cout << "filtered grid:\n" << filtered_grid << std::endl;

        // debug image
        // // display grid as OpenCV Image
        // std::string window_name = "Filtered Grid";
        // cv::namedWindow(window_name);
        // cv::Mat filtered_grid_8U;
        // filtered_grid.convertTo(filtered_grid_8U, CV_8U, 255.);
        // cv::resize(filtered_grid_8U, filtered_grid_8U, cv::Size(filtered_grid_8U.cols*4,filtered_grid_8U.rows*4),0,0,cv::INTER_NEAREST);
        // cv::imshow(window_name, filtered_grid_8U);
        // cv::waitKey(1);

        // sleep for the rest of the time
        rate.sleep();
      }
    }

    std::string type2str(int type)
    {
      std::string r;

      uchar depth = type & CV_MAT_DEPTH_MASK;
      uchar chans = 1 + (type >> CV_CN_SHIFT);

      switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
      }

      r += "C";
      r += (chans+'0');

      return r;
    }

  }; /* class ObstacleFeedbackProcessing */

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstacleFeedbackProcessing, nodelet::Nodelet);

} /* namespace rtabmap_ros */

