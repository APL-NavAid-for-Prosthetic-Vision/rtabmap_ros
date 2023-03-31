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

// system
#include <cstdint>
#include <map>
#include <tuple>
#include <utility>
#include <math.h>
#include <string.h>


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
      nh.param("obstacles_feedback/grid_res_x", grid_res_x_, 10.0);
      nh.param("obstacles_feedback/grid_res_y", grid_res_y_, 10.0);
      nh.param("obstacles_feedback/print_grid", print_grid_, false);
      nh.param("obstacles_feedback/min_connected_component_size", min_connected_component_size_, 2);
      

      // Subscriber
      semanticObstaclesProcessingSub_ = nh.subscribe("octomap_with_bbx_obstacles", 1, &ObstaclesFeedbackProcessing::semanticObstacleFeedbackProcessingCallback, this);
      poseSub_ = nh.subscribe("localization_pose", 1, &ObstaclesFeedbackProcessing::poseUpdateCallback, this);

      // publisher
      obstaclesFeedbackPub_ = nh.advertise<rtabmap_ros::ObstaclesBBXMapData>("obstacles_map_feedback", 1);
      // pointCloudPub_ = nh.advertise<rtabmap_ros::SelectedPointCloud>("obstacle_points_selected", 1);
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

        // float grid_res_x = grid_size/bbox_bounds_sizes_.x;
        // float grid_res_y_ = grid_size/bbox_bounds_sizes_.y;
        int grid_size_x = grid_res_x_ * bbox_bounds_sizes_.x;
        int grid_size_y = grid_res_y_ * bbox_bounds_sizes_.y;

        // int c = (grid_size/2)/grid_res;
        
        
        cv::Mat grid = cv::Mat::zeros(grid_size_x, grid_size_y, CV_8S);
        cv::Mat connected_grid;
        cv::Mat stats;
        cv::Mat centroids;
        std::vector<int> single_components;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_points(new pcl::PointCloud<pcl::PointXYZRGB>);
       
        // std::map<int, pcl::PointXYZRGB> point_map; //flattened coordinates to pointxyzrgb
        std::map<int, cv::Point3f> point_map; //flattened coordinates to pointxyzrgb
        //NODELET_WARN_STREAM("poseOrientationVector : " << poseOrientationVector_);
  
        if (octreePtr == nullptr)
        {
          rate.sleep();
          continue;
        }

        for (auto octreeNodeIter = octreePtr->begin(); octreeNodeIter != octreePtr->end(); ++octreeNodeIter)
        {
          if (octreePtr->isNodeOccupied(*octreeNodeIter))
          {
            
            cv::Point3f pt_map(octreeNodeIter.getX(), octreeNodeIter.getY(), octreeNodeIter.getZ());
            float pt_grid_x = pt_map.x - min_bbox_range_map.x; //in meters
            float pt_grid_y = pt_map.y - min_bbox_range_map.y; //in meters
            if (pt_grid_x >= bbox_bounds_sizes.x || pt_grid_y >= bbox_bounds_sizes.y ||
                pt_grid_x < 0 || pt_grid_y < 0)
            {
              continue;
            }   

            int pixel_x = floor(pt_grid_x * grid_res_x_); //in pixels
            int pixel_y = floor(pt_grid_y * grid_res_y_); //in pixels
            float pt_map_centered_x = ((pixel_x + 0.5) / grid_res_x_) + min_bbox_range_map.x; //meters
            float pt_map_centered_y = ((pixel_y + 0.5) / grid_res_y_) + min_bbox_range_map.y; //meters

      
            // NODELET_ERROR("pt_map X: %f, Y: %f", pt_map.x, pt_map.y);
            // NODELET_ERROR("pt_map_centered X: %f, Y: %f", pt_map_centered_x, pt_map_centered_y);
            // NODELET_ERROR("bbx_min X: %f, Y: %f", min_bbox_range_map.x, min_bbox_range_map.y);
            // NODELET_ERROR("pt minus bbx X: %f, Y: %f", pt_map.x - min_bbox_range_map.x, pt_map.y - min_bbox_range_map.y);
            // NODELET_ERROR("bbx_range X: %f, Y: %f", bbox_bounds_sizes.x, bbox_bounds_sizes.y);
            // NODELET_ERROR("grid_resolution: X: %f, Y: %f", grid_res_x_, grid_res_y_);
            
            // std::cout << "rows " << grid.rows << std::endl; 
            // std::cout << "cols " << grid.cols << std::endl; 
            if (grid.at<int8_t>(pixel_x, pixel_y) == 0)
            {

              rtabmap::SemanticColorOcTreeNode::Color pt_color = octreeNodeIter->getColor();
              // pcl::PointXYZRGB selected_point;
              cv::Point3f selected_point;
              selected_point.x = pt_map.x;
              selected_point.y = pt_map.y;
              selected_point.z = 0;

              // selected_point.r = pt_color.r;
              // selected_point.g = pt_color.g;
              // selected_point.b = pt_color.b;

              int flattened_coordinate = pixel_x * grid_size_x + pixel_y;
              point_map[flattened_coordinate] = selected_point;

              // selected_points->push_back(selected_point);
              
            
              grid.at<int8_t>(pixel_x, pixel_y) = 1;
              // NODELET_ERROR("pixel X: %d, pixel Y: %d", pixel_x, pixel_y);

            }

          }

        }
 
        grid.convertTo(grid, CV_8U);
        int num_components = cv::connectedComponentsWithStats(grid, connected_grid, stats, centroids, 8);
        // NODELET_WARN("GOT COMPONENTS");
        // NODELET_WARN("num components %d", num_components);
        // NODELET_WARN("rows %d", connected_grid.rows);
        // NODELET_WARN("cols %d", connected_grid.cols);

        // std::cout << type2str(connected_grid.type()) << std::endl;

        for (int i = 0; i < stats.rows; ++i)
        { 
          if (stats.at<int>(i, 4) < min_connected_component_size_)
          {
            // std::cout << "component " << i << " has " << stats.at<int>(i, 4) << " element" << std::endl;
            // std::cout << "value at x: " << stats.at<int>(i, 0) << " y: " << stats.at<int>(i, 1);
            // std::cout << "= " << connected_grid.at<int>(stats.at<int>(i, 1), stats.at<int>(i, 0)) << std::endl;
            // std::cout << connected_grid << std::endl;
            int flattened_coordinate = stats.at<int>(i, 1) * grid_size_x + stats.at<int>(i, 0);
            if (point_map.find(flattened_coordinate) != point_map.end())
            {
              point_map.erase(flattened_coordinate);
              connected_grid.at<int>(stats.at<int>(i, 1), stats.at<int>(i, 0)) = 0;
            }
            // single_components.push_back(flattened_coordinate);
          }
        } 

        // for (int component : single_components)
        // {
        //   pcl::PointXYZRGB current_point = point_map[component];
        //   for (auto it = selected_points->begin(); it != selected_points->end(); ++it)
        //   {
        //     if (it->x == current_point.x && it->y == current_point.y)
        //     {
        //       selected_points->erase(it);
        //     }
        //   }
        // }
        

        if (print_grid_)
        {
          std::cout << connected_grid << std::endl;
        }

        if (point_map.size() > 0)
        {
          rtabmap_ros::SelectedPointCloud selected_point_cloud_msg;
          for (auto iterator : point_map)
          {
            cv::Point3f point = iterator.second;
            geometry_msgs::Point32 point_msg;
            point_msg.x = point.x;
            point_msg.y = point.y;
            point_msg.z = point.z;

            int component_label = connected_grid.at<int>(iterator.first / grid_size_x, iterator.first % grid_size_y);
            selected_point_cloud_msg.point_cloud.points.push_back(point_msg);
            selected_point_cloud_msg.point_connected_component.push_back(component_label);
          }

          // std_msgs::Header header;
          // header.seq = 0;
          // header.stamp = ros::Time::now();
          // cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::TYPE_32SC1, connected_grid);
          // sensor_msgs::Image img_msg;
          // img_bridge.toImageMsg(img_msg);
          // selected_point_cloud_msg.component_grid = img_msg;

          pointCloudPub_.publish(selected_point_cloud_msg);
        }
        // if (selected_points->size() > 0)
        // {
        //   sensor_msgs::Image pointImage;
        //   pcl::toROSMsg(*selected_points, pointImage);
        //   rtabmap_ros::SelectedPointCloud pointCloud_msg;
        //   pointCloud_msg.pointCloud = pointImage;
        //   pointCloudPub_.publish(pointCloud_msg);
        //   // // publish message
        // }
        // data_msg.header.stamp = ros::Time::now();
        // data_msg.header.frame_id = "base_link";
        // obstaclesFeedbackPub_.publish(data_msg);

        // NODELET_ERROR("(obstacles processing) Semantic Octomap post-processing time = %0.4f sec", total_timer.ticks());

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
    boost::mutex obstacles_data_mtx_;

    cv::Point3f poseOrientationVector_;
    cv::Point3f min_bbox_range_map_;
    cv::Point3f max_bbox_range_map_;
    cv::Point3f bbox_bounds_sizes_;
    
    
    double grid_res_x_;
    double grid_res_y_;
    bool print_grid_;
    boost::thread processingThread_;

    boost::shared_ptr<rtabmap::SemanticColorOcTree> octreePtr_;
    //std::map<int, std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>> > > obstacles_data_; // {occupancyType : {classId : (point cloud, list of nodeId)} }
    int min_connected_component_size_;

    std::string type2str(int type) {
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

}; /* class ObstaclesFeedbackProcessing */

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesFeedbackProcessing, nodelet::Nodelet);

} /* namespace rtabmap_ros */