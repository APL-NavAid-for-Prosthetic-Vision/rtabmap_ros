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

  class ObstaclesFeedbackProcessing : public nodelet::Nodelet
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
    bool print_grid_;
    boost::thread processingThread_;

    boost::shared_ptr<rtabmap::SemanticColorOcTree> octreePtr_;
    //std::map<int, std::map<unsigned int, std::tuple<pcl::PointCloud<pcl::PointXYZRGB>::Ptr, std::vector<int>> > > obstacles_data_; // {occupancyType : {classId : (point cloud, list of nodeId)} }
    int min_connected_component_size_;


  public:
    // pcl::visualization::PCLVisualizer viewer;

    ObstaclesFeedbackProcessing() {}
      //: g_map_base_(rtabmap::Transform::getIdentity()){}

    virtual ~ObstaclesFeedbackProcessing() {}


  private:
    virtual void onInit()
    {
      ros::NodeHandle &nh = getNodeHandle();
      ros::NodeHandle &pnh = getPrivateNodeHandle();

      double rate = 2; // hz
      pnh.param("rate", rate, rate);
      pnh.param("obstacle_feedback_grid_resolution", grid_resolution_, 10.0);
      pnh.param("obstacle_feedback_min_connected_component_size", min_connected_component_size_, 5);
      pnh.param("obstacle_feedback_print_grid", print_grid_, false);
      
      NODELET_ERROR("grid_resolution: %f", grid_resolution_);
      NODELET_ERROR("print_grid_: %u", print_grid_);

      // subscribers
      semanticObstaclesProcessingSub_ = nh.subscribe("octomap_with_bbx_obstacles", 1, &ObstaclesFeedbackProcessing::semanticObstacleFeedbackProcessingCallback, this);
      //poseSub_ = nh.subscribe("localization_pose", 1, &ObstaclesFeedbackProcessing::poseUpdateCallback, this);

      // publishers
      //obstaclesFeedbackPub_ = nh.advertise<rtabmap_ros::ObstaclesBBXMapData>("obstacles_map_feedback", 1);
      connectedPointCloudPub_ = nh.advertise<rtabmap_ros::ObstaclesLocalMap>("obstacles_local_map", 1);

      // child threads
      processingThread_ = boost::thread(boost::bind(&ObstaclesFeedbackProcessing::feedbackProcessingThread, this, rate));
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
      //       show a top-down view of the map wrt the standard point of view for viewing 
      //       an image grid. (It ends up being a turned bottom-up view of the map)
      //

      ros::Rate rate(updateRateHz);

      std::string window_name = "Filtered Grid";
      cv::namedWindow(window_name);

      std::string window_name2 = "Filtered Grid 2";
      cv::namedWindow(window_name2);

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
        // float grid_res_y = grid_size/bbox_bounds_sizes_.y;
        int grid_cols = floor(grid_resolution_ * bbox_bounds_sizes_.x);
        int grid_rows = floor(grid_resolution_ * bbox_bounds_sizes_.y);
        // int c = (grid_size/2)/grid_res;
        
        cv::Mat grid = cv::Mat::zeros(grid_rows, grid_cols, CV_8U);
        cv::Mat connected_grid;
        cv::Mat stats;
        cv::Mat centroids;
        //std::vector<int> single_components;

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr selected_points(new pcl::PointCloud<pcl::PointXYZRGB>);
        // std::map<int, pcl::PointXYZRGB> map_coord2point; //flattened coordinates to pointxyzrgb
        std::map<int, cv::Point3f> map_coord2point; //flattened coordinates to pointxyzrgb
  
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
            // if (pt_grid_x >= bbox_bounds_sizes.x || pt_grid_y >= bbox_bounds_sizes.y ||
            //     pt_grid_x < 0 || pt_grid_y < 0)
            // {
            //   continue;
            // }

            // keep this point if a point for this grid position is not already found
            if (grid.at<uint8_t>(pixel_y, pixel_x) == 0)
            {
              // use the map position of the center of the grid pixel rather than the original map point
              cv::Point3f pt_map_for_grid;
              pt_map_for_grid.x = ((pixel_x + 0.5) / grid_resolution_) + min_bbox_range_map.x; // pt_map.x;
              pt_map_for_grid.y = ((pixel_y + 0.5) / grid_resolution_) + min_bbox_range_map.y; // pt_map.y;
              pt_map_for_grid.z = 0.;

              // NODELET_ERROR("pt_map X: %f, Y: %f", pt_map.x, pt_map.y);
              // NODELET_ERROR("pt_map for grid X: %f, Y: %f", pt_map_for_grid.x, pt_map_for_grid.y);
              // NODELET_ERROR("bbx_min X: %f, Y: %f", min_bbox_range_map.x, min_bbox_range_map.y);
              // NODELET_ERROR("pt minus bbx X: %f, Y: %f", pt_map.x - min_bbox_range_map.x, pt_map.y - min_bbox_range_map.y);
              // NODELET_ERROR("bbx_range X: %f, Y: %f", bbox_bounds_sizes.x, bbox_bounds_sizes.y);
              // NODELET_ERROR("grid_resolution: X: %f, Y: %f", grid_resolution_, grid_resolution_);

              //rtabmap::SemanticColorOcTreeNode::Color pt_color = octreeNodeIter->getColor();
              // grid_point.r = pt_color.r;
              // grid_point.g = pt_color.g;
              // grid_point.b = pt_color.b;

              int flattened_coordinate = pixel_y * grid_cols + pixel_x;
              map_coord2point[flattened_coordinate] = pt_map_for_grid;

              // selected_points->push_back(pt_map_for_grid);
              
              grid.at<uint8_t>(pixel_y, pixel_x) = 1;
              // NODELET_ERROR("pixel X: %d, pixel Y: %d", pixel_x, pixel_y);
            }
          }
        }

        // this sets the cv::Mat parameter types to:
        //   connected_grid:  CV_32S (int32)
        //   stats:           CV_32S (int32)
        int num_components = cv::connectedComponentsWithStats(grid, connected_grid, stats, centroids, 8, CV_32S);
        // NODELET_WARN("GOT COMPONENTS");
        // NODELET_WARN("num components %d", num_components);
        // NODELET_WARN("rows %d", connected_grid.rows);
        // NODELET_WARN("cols %d", connected_grid.cols);
        // NODELET_WARN("stats type is %d (%s)", stats.type(), type2str(stats.type()).c_str());
        // NODELET_WARN("connected_grid type is %d (%s)", connected_grid.type(), type2str(connected_grid.type()).c_str());

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

        // for (int i = 0; i < stats.rows; ++i)
        // { 
        //   if (stats.at<int>(i, cv::CC_STAT_AREA) < min_connected_component_size_)
        //   {
        //     // std::cout << "component " << i << " has " << stats.at<int>(i, 4) << " element" << std::endl;
        //     // std::cout << "value at x: " << stats.at<int>(i, 0) << " y: " << stats.at<int>(i, 1);
        //     // std::cout << "= " << connected_grid.at<int>(stats.at<int>(i, 1), stats.at<int>(i, 0)) << std::endl;
        //     // std::cout << connected_grid << std::endl;
        //     int flattened_coordinate = stats.at<int>(i, cv::CC_STAT_TOP) * grid_cols + stats.at<int>(i, cv::CC_STAT_LEFT);
        //     if (map_coord2point.find(flattened_coordinate) != map_coord2point.end())
        //     {
        //       map_coord2point.erase(flattened_coordinate);
        //       connected_grid.at<int>(stats.at<int>(i, 1), stats.at<int>(i, 0)) = 0;
        //     }
        //     // single_components.push_back(flattened_coordinate);
        //   }
        // } 

        // for (int component : single_components)
        // {
        //   pcl::PointXYZRGB current_point = map_coord2point[component];
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
          std::cout << "filtered grid:\n" << filtered_grid << std::endl;
        }

        // display grid as OpenCV Image
        cv::Mat filtered_grid_8U;
        filtered_grid.convertTo(filtered_grid_8U, CV_8U, 255.);
        cv::resize(filtered_grid_8U, filtered_grid_8U, cv::Size(filtered_grid_8U.cols*4,filtered_grid_8U.rows*4),0,0,cv::INTER_NEAREST);
        cv::imshow(window_name, filtered_grid_8U);
        //cv::waitKey(1);

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

          // std_msgs::Header header;
          // header.seq = 0;
          // header.stamp = ros::Time::now();
          // cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::TYPE_32SC1, connected_grid);
          // sensor_msgs::Image img_msg;
          // img_bridge.toImageMsg(img_msg);
          // connected_point_cloud_msg.component_grid = img_msg;

          connectedPointCloudPub_.publish(connected_point_cloud_msg);



          // convert back to grid and display as OpenCV Image
          // load the x,y components of the map points into a numpy array
          std::vector<cv::Point2f> points_map_2d, points_bodycentered_2d;
          cv::Point2f bodyOrigin_map_2d;
          cv::Mat obstacleImg = cv::Mat::zeros(connected_point_cloud_msg.grid_rows, connected_point_cloud_msg.grid_cols, CV_8U);
          bodyOrigin_map_2d.x = connected_point_cloud_msg.grid_origin_x + connected_point_cloud_msg.grid_cols / connected_point_cloud_msg.grid_resolution_per_meter / 2;
          bodyOrigin_map_2d.y = connected_point_cloud_msg.grid_origin_y + connected_point_cloud_msg.grid_rows / connected_point_cloud_msg.grid_resolution_per_meter / 2;
          for (auto iter = connected_point_cloud_msg.point_cloud.points.begin(); iter != connected_point_cloud_msg.point_cloud.points.end(); ++iter)
          {
            cv::Point2f point, point_bodycentered_2d, point_img_2d;
            cv::Point2i point_pixel, img_center;
            point.x = iter->x;
            point.y = iter->y;
            //points_map_2d.push_back(point);

            // first recenter on the body origin, then yaw to body heading keeping level with the horizon (i.e., no pitch/roll)
            // rospy.logwarn(f"bodyOrigin_map_2d: \n{bodyOrigin_map_2d}")
            // rospy.logwarn(f"points_map_2d: \n{points_map_2d}")
            //cv::Mat Rz_2d = cv::eye(2);
            point_bodycentered_2d = point - bodyOrigin_map_2d;

            // transform to image-based pixel coordinates
            // image coordinates follow the convention:
            //    x = right, y = down
            // we want the body frame axes to map into the image as follows:
            //    body_x => img_up
            //    body_y => img_left
            // therefore, apply a further body to image transformation:
            //   x_body => -y_img
            //   y_body => -x_img
            // point_img_2d.x = -point_bodycentered_2d.y;
            // point_img_2d.y = -point_bodycentered_2d.x;
            point_img_2d = point_bodycentered_2d;

            // now convert from image (metric) to pixel coordinates
            int img_width = connected_point_cloud_msg.grid_cols;
            int img_height = connected_point_cloud_msg.grid_rows;
            img_center.x = floor(img_width/2);
            img_center.y = floor(img_height/2);
            point_pixel.x = point_img_2d.x * connected_point_cloud_msg.grid_resolution_per_meter + img_center.x;  // shift the body origin to the image center
            point_pixel.y = point_img_2d.y * connected_point_cloud_msg.grid_resolution_per_meter + img_center.y;  // shift the body origin to the image center

            if (point_pixel.x >= 0 && point_pixel.x < img_width && point_pixel.y >= 0 && point_pixel.y < img_height)
            {
              obstacleImg.at<unsigned char>(point_pixel.y, point_pixel.x) = 255;
            }
            else
            {
              std::cout << "Invalid Pixel (row,col): (" << point_pixel.y << "," << point_pixel.x << ")" << std::endl;
            }
          }

          // display grid as OpenCV Image
          cv::resize(obstacleImg, obstacleImg, cv::Size(obstacleImg.cols*4, obstacleImg.rows*4), 0, 0, cv::INTER_NEAREST);
          cv::imshow(window_name2, obstacleImg);
          cv::waitKey(1);
        }

        // if (selected_points->size() > 0)
        // {
        //   sensor_msgs::Image pointImage;
        //   pcl::toROSMsg(*selected_points, pointImage);
        //   rtabmap_ros::ObstaclesLocalMap pointCloud_msg;
        //   pointCloud_msg.pointCloud = pointImage;
        //   connectedPointCloudPub_.publish(pointCloud_msg);
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

  }; /* class ObstaclesFeedbackProcessing */

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::ObstaclesFeedbackProcessing, nodelet::Nodelet);

} /* namespace rtabmap_ros */

