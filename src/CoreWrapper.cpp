/*
Modification by:
  The Johns Hopkins University Applied Physics Laboratory.

Original by:
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Universite de Sherbrooke nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "rtabmap_ros/CoreWrapper.h"

#include <stdio.h>
#include <ros/ros.h>
#include "pluginlib/class_list_macros.h"

#include <nav_msgs/Path.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/io.h>

#include <visualization_msgs/MarkerArray.h>

#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UDirectory.h>
#include <rtabmap/utilite/UFile.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UMath.h>

#include <rtabmap/core/util2d.h>
#include <rtabmap/core/util3d.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util3d_surface.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/OdometryEvent.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/Registration.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Optimizer.h>
#include <rtabmap/core/Landmark.h>

#include <image_transport/image_transport.h>
#include <opencv2/imgproc/types_c.h>
#include <opencv2/imgproc.hpp>

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
#include <octomap_msgs/conversions.h>
#include <rtabmap/core/OctoMap.h>
#include <rtabmap/core/SemanticOctoMap.h>
#endif
#endif

#define BAD_COVARIANCE 9999

#ifdef OPENCV_CUDA
#include <opencv2/core/cuda.hpp>
#include <opencv2/cudawarping.hpp>
#endif

// msgs
#include "rtabmap_ros/Info.h"
#include "rtabmap_ros/MapData.h"
#include "rtabmap_ros/MapGraph.h"
#include "rtabmap_ros/GetMap.h"
#include "rtabmap_ros/PublishMap.h"
#include "rtabmap_ros/Path.h"

#include "rtabmap_ros/MsgConversion.h"

/// JHUAPL section

#include "rtabmap_ros/utils_mapping.h"

#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Transform.h>

#include <rtabmap_ros/Landmark.h>
#include <rtabmap_ros/Landmarks.h>
#include <visualization_msgs/MarkerArray.h>

#include <rtabmap_ros/ObstaclesData.h>
#include <rtabmap_ros/MapManagerStats.h>
#include <rtabmap_ros/InputDataStats.h>

// boost
#include <boost/chrono.hpp>

/// JHUAPL section end

using namespace rtabmap;

namespace rtabmap_ros
{

  void signature_complete_callback(void *user_data)
  {
    CoreWrapper *core_wrapper = static_cast<CoreWrapper *>(user_data);

    // this is the same as the "data" sent to rtabmap except that it now
    // contains local occupancy grid information and post-processing
    // (e.g., decimation) of the sensor data
    rtabmap::SensorData sd = core_wrapper->get_rtabmap()->getMemory()->getLastAddedData();

    // publish obstacle data
    core_wrapper->publishObstaclesData(sd);
  }

  CoreWrapper::CoreWrapper() : CommonDataSubscriber(false),
                               paused_(false),
                               lastPose_(Transform::getIdentity()),
                               lastPoseIntermediate_(false),
                               latestNodeWasReached_(false),
                               frameId_("base_link"),
                               odomFrameId_(""),
                               mapFrameId_("map"),
                               groundTruthFrameId_(""),			// e.g., "world"
                               groundTruthBaseFrameId_(""), // e.g., "base_link_gt"
                               configPath_(""),
                               odomDefaultAngVariance_(0.001),
                               odomDefaultLinVariance_(0.001),
                               landmarkDefaultAngVariance_(0.001),
                               landmarkDefaultLinVariance_(0.001),
                               waitForTransform_(true),
                               waitForTransformDuration_(0.2), // 200 ms
                               useActionForGoal_(false),
                               useSavedMap_(true),
                               genScan_(false),
                               genScanMaxDepth_(4.0),
                               genScanMinDepth_(0.0),
                               genDepth_(false),
                               genDepthDecimation_(1),
                               genDepthFillHolesSize_(0),
                               genDepthFillIterations_(1),
                               genDepthFillHolesError_(0.1),
                               scanCloudMaxPoints_(0),
                               mapToOdom_(rtabmap::Transform::getIdentity()),
                               transformThread_(0),
                               tfThreadRunning_(false),
                               stereoToDepth_(false),
                               interOdomSync_(0),
                               odomSensorSync_(false),
                               rate_(Parameters::defaultRtabmapDetectionRate()),
                               createIntermediateNodes_(Parameters::defaultRtabmapCreateIntermediateNodes()),
                               mappingMaxNodes_(Parameters::defaultGridGlobalMaxNodes()),
                               mappingAltitudeDelta_(Parameters::defaultGridGlobalAltitudeDelta()),
                               alreadyRectifiedImages_(Parameters::defaultRtabmapImagesAlreadyRectified()),
                               twoDMapping_(Parameters::defaultRegForce3DoF()),
                               previousStamp_(0),
                               mbClient_(0),
                               maxNodesRepublished_(2),
                               rtabmapProcessThread_(0),
                               rtabmapProcessThreadRunning_(false),
                               mapManagerUpdateThread_(0),
                               mapManagerUpdateThreadRunning_(false),
                               publishMapThreadRunning_(false),
                               last_baseToMap_(rtabmap::Transform::getIdentity()),
                               mapToOdomPrev_(rtabmap::Transform::getIdentity()),
                               mapManagerInitialized_(false),
                               threadedMode_(false),
                               timeInputLastProcess_(0),
                               camTobaseTransform_(rtabmap::Transform::getIdentity()),
                               eventHandlerThread_()
  {
    char *rosHomePath = getenv("ROS_HOME");
    std::string workingDir = rosHomePath ? rosHomePath : UDirectory::homeDir() + "/.ros";
    databasePath_ = workingDir + "/" + rtabmap::Parameters::getDefaultDatabaseName();
    globalPose_.header.stamp = ros::Time(0);
  }

  void CoreWrapper::onInit()
  {

    // JHUAPL section
    // allowing for nodelet to be use in multi thread mode.
    int num_threads_param = -1;
    getPrivateNodeHandle().getParam("num_worker_threads", num_threads_param);

    NODELET_INFO("number of thread allocated for NODELET: %d", num_threads_param);

    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &pnh = getMTPrivateNodeHandle();

    // JHUAPL section

    //ros::NodeHandle &nh = getNodeHandle();
    //ros::NodeHandle &pnh = getPrivateNodeHandle();

    mapsManager_.init(nh, pnh, getName(), true);

    bool publishTf = true;
    double tfDelay = 0.05;		// 20 Hz
    double tfTolerance = 0.1; // 100 ms
    std::string odomFrameIdInit;

    pnh.param("config_path", configPath_, configPath_);
    pnh.param("database_path", databasePath_, databasePath_);

    pnh.param("frame_id", frameId_, frameId_);
    pnh.param("odom_frame_id", odomFrameId_, odomFrameId_);						 // set to use odom from TF
    pnh.param("odom_frame_id_init", odomFrameIdInit, odomFrameIdInit); // set to publish map->odom TF before receiving odom topic
    pnh.param("map_frame_id", mapFrameId_, mapFrameId_);
    pnh.param("ground_truth_frame_id", groundTruthFrameId_, groundTruthFrameId_);
    pnh.param("ground_truth_base_frame_id", groundTruthBaseFrameId_, frameId_);
    if (pnh.hasParam("depth_cameras") && !pnh.hasParam("depth_cameras"))
    {
      NODELET_ERROR("\"depth_cameras\" parameter doesn't exist "
                    "anymore! It is replaced by \"rgbd_cameras\" parameter "
                    "used when \"subscribe_rgbd\" is true");
    }
    if (!odomFrameIdInit.empty())
    {
      if (odomFrameId_.empty())
      {
        ROS_INFO("rtabmap: odom_frame_id_init = %s", odomFrameIdInit.c_str());
        odomFrameId_ = odomFrameIdInit;
      }
      else
      {
        ROS_WARN("odom_frame_id_init (%s) is ignored if odom_frame_id (%s) is set.", odomFrameIdInit.c_str(), odomFrameId_.c_str());
      }
    }

    pnh.param("publish_tf", publishTf, publishTf);
    pnh.param("tf_delay", tfDelay, tfDelay);
    if (pnh.hasParam("tf_prefix"))
    {
      ROS_ERROR("tf_prefix parameter has been removed, use directly map_frame_id, odom_frame_id and frame_id parameters.");
    }
    pnh.param("tf_tolerance", tfTolerance, tfTolerance);
    pnh.param("odom_tf_angular_variance", odomDefaultAngVariance_, odomDefaultAngVariance_);
    pnh.param("odom_tf_linear_variance", odomDefaultLinVariance_, odomDefaultLinVariance_);
    pnh.param("landmark_angular_variance", landmarkDefaultAngVariance_, landmarkDefaultAngVariance_);
    pnh.param("landmark_linear_variance", landmarkDefaultLinVariance_, landmarkDefaultLinVariance_);
    pnh.param("wait_for_transform", waitForTransform_, waitForTransform_);
    pnh.param("wait_for_transform_duration", waitForTransformDuration_, waitForTransformDuration_);
    pnh.param("use_action_for_goal", useActionForGoal_, useActionForGoal_);
    pnh.param("use_saved_map", useSavedMap_, useSavedMap_);
    pnh.param("max_nodes_republished", maxNodesRepublished_, maxNodesRepublished_);
    pnh.param("gen_scan", genScan_, genScan_);
    pnh.param("gen_scan_max_depth", genScanMaxDepth_, genScanMaxDepth_);
    pnh.param("gen_scan_min_depth", genScanMinDepth_, genScanMinDepth_);
    pnh.param("gen_depth", genDepth_, genDepth_);
    pnh.param("gen_depth_decimation", genDepthDecimation_, genDepthDecimation_);
    pnh.param("gen_depth_fill_holes_size", genDepthFillHolesSize_, genDepthFillHolesSize_);
    pnh.param("gen_depth_fill_iterations", genDepthFillIterations_, genDepthFillIterations_);
    pnh.param("gen_depth_fill_holes_error", genDepthFillHolesError_, genDepthFillHolesError_);
    pnh.param("scan_cloud_max_points", scanCloudMaxPoints_, scanCloudMaxPoints_);
    if (pnh.hasParam("scan_cloud_normal_k"))
    {
      ROS_WARN("rtabmap: Parameter \"scan_cloud_normal_k\" has been removed. RTAB-Map's parameter \"%s\" should be used instead. "
               "The value is copied. Use \"%s\" to avoid this warning.",
               Parameters::kMemLaserScanNormalK().c_str(),
               Parameters::kMemLaserScanNormalK().c_str());
      double value;
      pnh.getParam("scan_cloud_normal_k", value);
      uInsert(parameters_, ParametersPair(Parameters::kMemLaserScanNormalK(), uNumber2Str(value)));
    }
    pnh.param("stereo_to_depth", stereoToDepth_, stereoToDepth_);
    pnh.param("odom_sensor_sync", odomSensorSync_, odomSensorSync_);
    if (pnh.hasParam("flip_scan"))
    {
      NODELET_WARN("Parameter \"flip_scan\" doesn't exist anymore. Rtabmap now "
                   "detects automatically if the laser is upside down with /tf, then if so, it "
                   "switches scan values.");
    }

    NODELET_INFO("rtabmap: frame_id      = %s", frameId_.c_str());
    if (!odomFrameId_.empty())
    {
      NODELET_INFO("rtabmap: odom_frame_id = %s", odomFrameId_.c_str());
    }
    if (!groundTruthFrameId_.empty())
    {
      NODELET_INFO("rtabmap: ground_truth_frame_id = %s -> ground_truth_base_frame_id = %s",
                   groundTruthFrameId_.c_str(),
                   groundTruthBaseFrameId_.c_str());
    }
    NODELET_INFO("rtabmap: map_frame_id  = %s", mapFrameId_.c_str());
    NODELET_INFO("rtabmap: use_action_for_goal  = %s", useActionForGoal_ ? "true" : "false");
    NODELET_INFO("rtabmap: tf_delay      = %f", tfDelay);
    NODELET_INFO("rtabmap: tf_tolerance  = %f", tfTolerance);
    NODELET_INFO("rtabmap: odom_sensor_sync   = %s", odomSensorSync_ ? "true" : "false");
    bool subscribeStereo = false;
    pnh.param("subscribe_stereo", subscribeStereo, subscribeStereo);
    if (subscribeStereo)
    {
      NODELET_INFO("rtabmap: stereo_to_depth = %s", stereoToDepth_ ? "true" : "false");
    }

    NODELET_INFO("rtabmap: gen_scan  = %s", genScan_ ? "true" : "false");
    if (genScan_)
    {
      NODELET_INFO("rtabmap: gen_scan_max_depth  = %f", genScanMaxDepth_);
      NODELET_INFO("rtabmap: gen_scan_min_depth  = %f", genScanMinDepth_);
    }

    NODELET_INFO("rtabmap: gen_depth  = %s", genDepth_ ? "true" : "false");
    if (genDepth_)
    {
      NODELET_INFO("rtabmap: gen_depth_decimation        = %d", genDepthDecimation_);
      NODELET_INFO("rtabmap: gen_depth_fill_holes_size   = %d", genDepthFillHolesSize_);
      NODELET_INFO("rtabmap: gen_depth_fill_iterations   = %d", genDepthFillIterations_);
      NODELET_INFO("rtabmap: gen_depth_fill_holes_error  = %f", genDepthFillHolesError_);
    }
    bool subscribeScanCloud = false;
    pnh.param("subscribe_scan_cloud", subscribeScanCloud, subscribeScanCloud);
    if (subscribeScanCloud)
    {
      NODELET_INFO("rtabmap: scan_cloud_max_points = %d", scanCloudMaxPoints_);
    }

    // JHUAPL section

#ifdef OPENCV_CUDA
    NODELET_INFO("rtabmap: OPENCV_CUDA: Enabled");
#else
    NODELET_INFO("rtabmap: OPENCV_CUDA: Disabled");
#endif

    pnh.param("depth_Filters", depthFilters_, depthFilters_);
    pnh.param("threaded_mode", threadedMode_, threadedMode_);
    NODELET_INFO("RTABMAP Thread Processing Mode: %s", threadedMode_ ? "ENABLED" : "DISABLED");

    // JHUAPL section end

    obstacleDataSeq = 0;

    infoPub_ = nh.advertise<rtabmap_ros::Info>("info", 1);
    mapDataPub_ = nh.advertise<rtabmap_ros::MapData>("mapData", 1);
    mapGraphPub_ = nh.advertise<rtabmap_ros::MapGraph>("mapGraph", 1);
    odomCachePub_ = nh.advertise<rtabmap_ros::MapGraph>("mapOdomCache", 1);
    landmarksPub_ = nh.advertise<geometry_msgs::PoseArray>("landmarks", 1);
    labelsPub_ = nh.advertise<visualization_msgs::MarkerArray>("labels", 1);
    mapPathPub_ = nh.advertise<nav_msgs::Path>("mapPath", 1);
    localGridObstacle_ = nh.advertise<sensor_msgs::PointCloud2>("local_grid_obstacle", 1);
    localGridEmpty_ = nh.advertise<sensor_msgs::PointCloud2>("local_grid_empty", 1);
    localGridGround_ = nh.advertise<sensor_msgs::PointCloud2>("local_grid_ground", 1);
    localizationPosePub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("localization_pose", 1);
    initialPoseSub_ = nh.subscribe("initialpose", 1, &CoreWrapper::initialPoseCallback, this);

    // planning topics
    goalSub_ = nh.subscribe("goal", 1, &CoreWrapper::goalCallback, this);
    goalNodeSub_ = nh.subscribe("goal_node", 1, &CoreWrapper::goalNodeCallback, this);
    nextMetricGoalPub_ = nh.advertise<geometry_msgs::PoseStamped>("goal_out", 1);
    goalReachedPub_ = nh.advertise<std_msgs::Bool>("goal_reached", 1);
    globalPathPub_ = nh.advertise<nav_msgs::Path>("global_path", 1);
    localPathPub_ = nh.advertise<nav_msgs::Path>("local_path", 1);
    globalPathNodesPub_ = nh.advertise<rtabmap_ros::Path>("global_path_nodes", 1);
    localPathNodesPub_ = nh.advertise<rtabmap_ros::Path>("local_path_nodes", 1);

    ros::Publisher nextMetricGoal_;
    ros::Publisher goalReached_;
    ros::Publisher path_;

    configPath_ = uReplaceChar(configPath_, '~', UDirectory::homeDir());
    databasePath_ = uReplaceChar(databasePath_, '~', UDirectory::homeDir());
#ifndef _WIN32
    if (configPath_.size() && configPath_.at(0) != '/')
    {
      configPath_ = UDirectory::currentDir(true) + configPath_;
    }
    if (databasePath_.size() && databasePath_.at(0) != '/')
    {
      databasePath_ = UDirectory::currentDir(true) + databasePath_;
    }
#endif

    ParametersMap allParameters = Parameters::getDefaultParameters();
    // remove Odom parameters
    for (ParametersMap::iterator iter = allParameters.begin(); iter != allParameters.end();)
    {
      if (iter->first.find("Odom") == 0)
      {
        allParameters.erase(iter++);
      }
      else
      {
        ++iter;
      }
    }
    uInsert(allParameters, ParametersPair(Parameters::kRGBDCreateOccupancyGrid(), "true")); // default true in ROS
    char *rosHomePath = getenv("ROS_HOME");
    std::string workingDir = rosHomePath ? rosHomePath : UDirectory::homeDir() + "/.ros";
    uInsert(allParameters, ParametersPair(Parameters::kRtabmapWorkingDirectory(), workingDir)); // change default to ~/.ros

    // load parameters
    loadParameters(configPath_, parameters_);
    for (ParametersMap::iterator iter = parameters_.begin(); iter != parameters_.end();)
    {
      if (iter->first.find("Odom") == 0)
      {
        parameters_.erase(iter++);
      }
      else
      {
        ++iter;
      }
    }

    // update parameters with user input parameters (private)
    for (ParametersMap::iterator iter = allParameters.begin(); iter != allParameters.end(); ++iter)
    {
      std::string vStr;
      bool vBool;
      int vInt;
      double vDouble;
      if (pnh.getParam(iter->first, vStr))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());

        if (iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
        {
          vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
        }
        else if (iter->first.compare(Parameters::kKpDictionaryPath()) == 0)
        {
          vStr = uReplaceChar(vStr, '~', UDirectory::homeDir());
        }
        uInsert(parameters_, ParametersPair(iter->first, vStr));
      }
      else if (pnh.getParam(iter->first, vBool))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
        uInsert(parameters_, ParametersPair(iter->first, uBool2Str(vBool)));
      }
      else if (pnh.getParam(iter->first, vDouble))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
        uInsert(parameters_, ParametersPair(iter->first, uNumber2Str(vDouble)));
      }
      else if (pnh.getParam(iter->first, vInt))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
        uInsert(parameters_, ParametersPair(iter->first, uNumber2Str(vInt)));
      }
    }

    // parse input arguments
    std::vector<std::string> argList = getMyArgv();
    char **argv = new char *[argList.size()];
    bool deleteDbOnStart = false;
    for (unsigned int i = 0; i < argList.size(); ++i)
    {
      argv[i] = &argList[i].at(0);
      if (strcmp(argv[i], "--delete_db_on_start") == 0 || strcmp(argv[i], "-d") == 0)
      {
        deleteDbOnStart = true;
      }
    }
    rtabmap::ParametersMap parameters = rtabmap::Parameters::parseArguments(argList.size(), argv);
    delete[] argv;
    for (ParametersMap::const_iterator iter = parameters.begin(); iter != parameters.end(); ++iter)
    {
      uInsert(parameters_, ParametersPair(iter->first, iter->second));
      NODELET_INFO("Update RTAB-Map parameter \"%s\"=\"%s\" from arguments", iter->first.c_str(), iter->second.c_str());
    }

    // Backward compatibility
    for (std::map<std::string, std::pair<bool, std::string>>::const_iterator iter = Parameters::getRemovedParameters().begin();
         iter != Parameters::getRemovedParameters().end();
         ++iter)
    {
      std::string vStr;
      bool vBool;
      int vInt;
      double vDouble;
      std::string paramValue;
      if (pnh.getParam(iter->first, vStr))
      {
        paramValue = vStr;
      }
      else if (pnh.getParam(iter->first, vBool))
      {
        paramValue = uBool2Str(vBool);
      }
      else if (pnh.getParam(iter->first, vDouble))
      {
        paramValue = uNumber2Str(vDouble);
      }
      else if (pnh.getParam(iter->first, vInt))
      {
        paramValue = uNumber2Str(vInt);
      }
      if (!paramValue.empty())
      {
        if (!iter->second.second.empty() && parameters_.find(iter->second.second) != parameters_.end())
        {
          NODELET_WARN("Rtabmap: Parameter name changed: \"%s\" -> \"%s\". The new parameter is already used with value \"%s\", ignoring the old one with value \"%s\".",
                       iter->first.c_str(), iter->second.second.c_str(), parameters_.find(iter->second.second)->second.c_str(), paramValue.c_str());
        }
        else if (iter->second.first)
        {
          // can be migrated
          uInsert(parameters_, ParametersPair(iter->second.second, paramValue));
          NODELET_WARN("Rtabmap: Parameter name changed: \"%s\" -> \"%s\". Please update your launch file accordingly. Value \"%s\" is still set to the new parameter name.",
                       iter->first.c_str(), iter->second.second.c_str(), paramValue.c_str());
        }
        else
        {
          if (iter->second.second.empty())
          {
            NODELET_ERROR("Rtabmap: Parameter \"%s\" doesn't exist anymore!",
                          iter->first.c_str());
          }
          else
          {
            NODELET_ERROR("Rtabmap: Parameter \"%s\" doesn't exist anymore! You may look at this similar parameter: \"%s\"",
                          iter->first.c_str(), iter->second.second.c_str());
          }
        }
      }
    }

    // Backward compatibility (MapsManager)
    mapsManager_.backwardCompatibilityParameters(pnh, parameters_);

    bool subscribeScan2d = false;
    bool subscribeScan3d = false;
    pnh.param("subscribe_scan", subscribeScan2d, subscribeScan2d);
    pnh.param("subscribe_scan_cloud", subscribeScan3d, subscribeScan3d);
    int gridSensor = Parameters::defaultGridSensor();
    if ((subscribeScan2d || subscribeScan3d || genScan_) && parameters_.find(Parameters::kGridSensor()) == parameters_.end())
    {
      NODELET_WARN("Setting \"%s\" parameter to 0 (default 1) as \"subscribe_scan\", \"subscribe_scan_cloud\" or \"gen_scan\" is "
                   "true. The occupancy grid map will be constructed from "
                   "laser scans. To get occupancy grid map from camera's cloud projection, set \"%s\" "
                   "to 1. To suppress this warning, "
                   "add <param name=\"%s\" type=\"string\" value=\"0\"/>",
                   Parameters::kGridSensor().c_str(),
                   Parameters::kGridSensor().c_str(),
                   Parameters::kGridSensor().c_str());
      parameters_.insert(ParametersPair(Parameters::kGridSensor(), "0"));
    }
    Parameters::parse(parameters_, Parameters::kGridSensor(), gridSensor);
    if ((subscribeScan2d || subscribeScan3d || genScan_) && parameters_.find(Parameters::kGridRangeMax()) == parameters_.end() && gridSensor == 0)
    {
      NODELET_INFO("Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan\", \"subscribe_scan_cloud\" or \"gen_scan\" is true and %s is 0.",
                   Parameters::kGridRangeMax().c_str(),
                   Parameters::defaultGridRangeMax(),
                   Parameters::kGridSensor().c_str());
      parameters_.insert(ParametersPair(Parameters::kGridRangeMax(), "0"));
    }
    if (subscribeScan3d && parameters_.find(Parameters::kIcpPointToPlaneRadius()) == parameters_.end())
    {
      NODELET_INFO("Setting \"%s\" parameter to 0 (default %f) as \"subscribe_scan_cloud\" is true.",
                   Parameters::kIcpPointToPlaneRadius().c_str(),
                   Parameters::defaultIcpPointToPlaneRadius());
      parameters_.insert(ParametersPair(Parameters::kIcpPointToPlaneRadius(), "0"));
    }
    int regStrategy = Parameters::defaultRegStrategy();
    Parameters::parse(parameters_, Parameters::kRegStrategy(), regStrategy);
    if (parameters_.find(Parameters::kRGBDProximityPathMaxNeighbors()) == parameters_.end() &&
        (regStrategy == Registration::kTypeIcp || regStrategy == Registration::kTypeVisIcp))
    {
      if (subscribeScan2d)
      {
        NODELET_WARN("Setting \"%s\" parameter to 10 (default 0) as \"subscribe_scan\" is "
                     "true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
                     "scans. To disable, set \"%s\" to 0. To suppress this warning, "
                     "add <param name=\"%s\" type=\"string\" value=\"10\"/>",
                     Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                     Parameters::kRegStrategy().c_str(),
                     Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                     Parameters::kRGBDProximityPathMaxNeighbors().c_str());
        parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
      }
      else if (subscribeScan3d)
      {
        NODELET_WARN("Setting \"%s\" parameter to 1 (default 0) as \"subscribe_scan_cloud\" is "
                     "true and \"%s\" uses ICP. To disable, set \"%s\" to 0. To suppress this warning, "
                     "add <param name=\"%s\" type=\"string\" value=\"1\"/>",
                     Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                     Parameters::kRegStrategy().c_str(),
                     Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                     Parameters::kRGBDProximityPathMaxNeighbors().c_str());
        parameters_.insert(ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
      }
    }

    int estimationType = Parameters::defaultVisEstimationType();
    Parameters::parse(parameters_, Parameters::kVisEstimationType(), estimationType);
    int cameras = 0;
    bool subscribeRGBD = false;
    pnh.param("rgbd_cameras", cameras, cameras);
    pnh.param("subscribe_rgbd", subscribeRGBD, subscribeRGBD);
    if (subscribeRGBD && cameras > 1 && estimationType > 0)
    {
      NODELET_WARN("Setting \"%s\" parameter to 0 (%d is not supported "
                   "for multi-cameras) as \"subscribe_rgbd\" is "
                   "true and \"rgbd_cameras\">1. Set \"%s\" to 0 to suppress this warning.",
                   Parameters::kVisEstimationType().c_str(),
                   estimationType,
                   Parameters::kVisEstimationType().c_str());
      uInsert(parameters_, ParametersPair(Parameters::kVisEstimationType(), "0"));
    }

    // modify default parameters with those in the database
    if (!deleteDbOnStart)
    {
      ParametersMap dbParameters;
      rtabmap::DBDriver *driver = rtabmap::DBDriver::create();
      if (driver->openConnection(databasePath_))
      {
        dbParameters = driver->getLastParameters(); // parameter migration is already done
      }
      delete driver;
      for (ParametersMap::iterator iter = dbParameters.begin(); iter != dbParameters.end(); ++iter)
      {
        if (iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
        {
          // ignore working directory
          continue;
        }
        if (parameters_.find(iter->first) == parameters_.end() &&
            allParameters.find(iter->first) != allParameters.end() &&
            allParameters.find(iter->first)->second.compare(iter->second) != 0)
        {
          NODELET_INFO("Update RTAB-Map parameter \"%s\"=\"%s\" from database", iter->first.c_str(), iter->second.c_str());
          parameters_.insert(*iter);
        }
      }
    }
    ParametersMap modifiedParameters = parameters_;
    // Add all other parameters (not copied if already exists)
    parameters_.insert(allParameters.begin(), allParameters.end());

    if (parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kRtabmapDetectionRate(), rate_);
      NODELET_INFO("RTAB-Map detection rate = %f Hz", rate_);
    }
    if (parameters_.find(Parameters::kRtabmapCreateIntermediateNodes()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kRtabmapCreateIntermediateNodes(), createIntermediateNodes_);
      if (createIntermediateNodes_)
      {
        NODELET_INFO("Create intermediate nodes");
        if (rate_ == 0.0f)
        {
          bool interOdomInfo = false;
          pnh.getParam("subscribe_inter_odom_info", interOdomInfo);
          if (interOdomInfo)
          {
            NODELET_INFO("Subscribe to inter odom + info messages");
            interOdomSync_ = new message_filters::Synchronizer<MyExactInterOdomSyncPolicy>(MyExactInterOdomSyncPolicy(100), interOdomSyncSub_, interOdomInfoSyncSub_);
            interOdomSync_->registerCallback(boost::bind(&CoreWrapper::interOdomInfoCallback, this, _1, _2));
            interOdomSyncSub_.subscribe(nh, "inter_odom", 100);
            interOdomInfoSyncSub_.subscribe(nh, "inter_odom_info", 100);
          }
          else
          {
            NODELET_INFO("Subscribe to inter odom messages");
            interOdomSub_ = nh.subscribe("inter_odom", 100, &CoreWrapper::interOdomCallback, this);
          }
        }
      }
    }
    if (parameters_.find(Parameters::kGridGlobalMaxNodes()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kGridGlobalMaxNodes(), mappingMaxNodes_);
      if (mappingMaxNodes_ > 0)
      {
        NODELET_INFO("Max mapping nodes = %d", mappingMaxNodes_);
      }
    }
    if (parameters_.find(Parameters::kGridGlobalAltitudeDelta()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kGridGlobalAltitudeDelta(), mappingAltitudeDelta_);
      if (mappingAltitudeDelta_ > 0.0)
      {
        NODELET_INFO("Mapping altitude delta = %f", mappingAltitudeDelta_);
      }
    }
    if (parameters_.find(Parameters::kRtabmapImagesAlreadyRectified()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kRtabmapImagesAlreadyRectified(), alreadyRectifiedImages_);
    }
    if (parameters_.find(Parameters::kRegForce3DoF()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kRegForce3DoF(), twoDMapping_);
    }

    paused_ = pnh.param("is_rtabmap_paused", paused_);
    if (paused_)
    {
      NODELET_WARN("Node paused... don't forget to call service \"resume\" to start rtabmap.");
    }

    if (deleteDbOnStart)
    {
      if (UFile::erase(databasePath_) == 0)
      {
        NODELET_INFO("rtabmap: Deleted database \"%s\" (--delete_db_on_start or -d are set).", databasePath_.c_str());
      }
    }

    if (databasePath_.size())
    {
      NODELET_INFO("rtabmap: Using database from \"%s\" (%ld MB).", databasePath_.c_str(), UFile::length(databasePath_) / (1024 * 1024));
    }
    else
    {
      NODELET_INFO("rtabmap: database_path parameter not set, the map will not be saved.");
    }

    // JHUAPL section
    bool enableSemanticSegmentation = false;
    if (parameters_.find(Parameters::kGridEnableSemanticSegmentation()) != parameters_.end())
    {
      Parameters::parse(parameters_, Parameters::kGridEnableSemanticSegmentation(), enableSemanticSegmentation);
      NODELET_INFO("rtabmap: parameter EnableSemanticSegmentation: %s", enableSemanticSegmentation ? "true" : "false");
    }

    std::string semanticSegmentationModelFilePath;
    pnh.param("model_classes_file_path", semanticSegmentationModelFilePath, semanticSegmentationModelFilePath);
    NODELET_INFO(" model_classes_file_path = %s", semanticSegmentationModelFilePath.empty() ? "NOT_PATH" : semanticSegmentationModelFilePath.c_str());

    // set the model class map if available
    std::map<std::string, std::map<unsigned int, std::string>> networkModelMap;
#ifdef WITH_YAMLCPP
    std::map<unsigned int, cv::Point3f> modelMaskIdColorMap;
    if (enableSemanticSegmentation && !semanticSegmentationModelFilePath.empty())
    {
      if (!utils::parseModelConfig(semanticSegmentationModelFilePath, networkModelMap, modelMaskIdColorMap))
      {
        ROS_WARN("parseModelConfig FAILED to parse the semantic segmentation ocupacy to label id to name association file");
      }
    }
#endif

    bool publishVisualImage = false;
    pnh.param("publish_visual_image", publishVisualImage, publishVisualImage);

    NODELET_INFO("rtabmap: parameter PublishVisualImage: %s", publishVisualImage ? "true" : "false");

    bool publishDepthImage = false;
    pnh.param("publish_depth_image", publishDepthImage, publishDepthImage);

    NODELET_INFO("rtabmap: parameter PublishDethImage: %s", publishDepthImage ? "true" : "false");

    double mapManagerUpdateRate = 0.5;
    pnh.param("map_manager_update_rate", mapManagerUpdateRate, mapManagerUpdateRate);

    LandmarkToMultiSignatureId_ = false;
    pnh.param("landmark_to_multi_signature_id", LandmarkToMultiSignatureId_, LandmarkToMultiSignatureId_);

    // JHUAPL section end

    mapsManager_.setParameters(parameters_);

    // Init RTAB-Map
    rtabmap_.init(parameters_, databasePath_);

    if (rtabmap_.getMemory())
    {
      if (useSavedMap_)
      {
        float xMin, yMin, gridCellSize;
        cv::Mat map = rtabmap_.getMemory()->load2DMap(xMin, yMin, gridCellSize);
        if (!map.empty())
        {
          NODELET_INFO("rtabmap: 2D occupancy grid map loaded (%dx%d).", map.cols, map.rows);
          mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());
        }
      }

      if (rtabmap_.getMemory()->getWorkingMem().size() > 1)
      {
        NODELET_INFO("rtabmap: Working Memory = %d, Local map = %d.",
                     (int)rtabmap_.getMemory()->getWorkingMem().size() - 1,
                     (int)rtabmap_.getLocalOptimizedPoses().size());
      }

      if (databasePath_.size())
      {
        NODELET_INFO("rtabmap: Database version = \"%s\".", rtabmap_.getMemory()->getDatabaseVersion().c_str());
      }

      if (rtabmap_.getMemory()->isIncremental())
      {
        NODELET_INFO("rtabmap: SLAM mode (%s=true)", Parameters::kMemIncrementalMemory().c_str());
      }
      else
      {
        NODELET_INFO("rtabmap: Localization mode (%s=false)", Parameters::kMemIncrementalMemory().c_str());
      }

      // JHUAPL section
      // In Semantic Segmentation mode this needs to be configured.
      if (enableSemanticSegmentation)
      {
        std::map<unsigned int, int> classId2octreeId = mapsManager_.getSemanticOctomap()->getClassIdToOctreeId();
        rtabmap_.setOccupancyObjectAssociation(networkModelMap, classId2octreeId);
      }

      // JHUAPL section end
    }

    // setup services
    updateSrv_ = nh.advertiseService("update_parameters", &CoreWrapper::updateRtabmapCallback, this);
    resetSrv_ = nh.advertiseService("reset", &CoreWrapper::resetRtabmapCallback, this);
    pauseSrv_ = nh.advertiseService("pause", &CoreWrapper::pauseRtabmapCallback, this);
    resumeSrv_ = nh.advertiseService("resume", &CoreWrapper::resumeRtabmapCallback, this);
    loadDatabaseSrv_ = nh.advertiseService("load_database", &CoreWrapper::loadDatabaseCallback, this);
    triggerNewMapSrv_ = nh.advertiseService("trigger_new_map", &CoreWrapper::triggerNewMapCallback, this);
    backupDatabase_ = nh.advertiseService("backup", &CoreWrapper::backupDatabaseCallback, this);
    detectMoreLoopClosuresSrv_ = nh.advertiseService("detect_more_loop_closures", &CoreWrapper::detectMoreLoopClosuresCallback, this);
    globalBundleAdjustmentSrv_ = nh.advertiseService("global_bundle_adjustment", &CoreWrapper::globalBundleAdjustmentCallback, this);
    cleanupLocalGridsSrv_ = nh.advertiseService("cleanup_local_grids", &CoreWrapper::cleanupLocalGridsCallback, this);
    setModeLocalizationSrv_ = nh.advertiseService("set_mode_localization", &CoreWrapper::setModeLocalizationCallback, this);
    setModeMappingSrv_ = nh.advertiseService("set_mode_mapping", &CoreWrapper::setModeMappingCallback, this);
    getNodeDataSrv_ = nh.advertiseService("get_node_data", &CoreWrapper::getNodeDataCallback, this);
    getMapDataSrv_ = nh.advertiseService("get_map_data", &CoreWrapper::getMapDataCallback, this);
    getMapData2Srv_ = nh.advertiseService("get_map_data2", &CoreWrapper::getMapData2Callback, this);
    //getMapSrv_ = nh.advertiseService("get_map", &CoreWrapper::getMapCallback, this);
    //getProbMapSrv_ = nh.advertiseService("get_prob_map", &CoreWrapper::getProbMapCallback, this);
    //getGridMapSrv_ = nh.advertiseService("get_grid_map", &CoreWrapper::getGridMapCallback, this);
    //getProjMapSrv_ = nh.advertiseService("get_proj_map", &CoreWrapper::getProjMapCallback, this);
    publishMapDataSrv_ = nh.advertiseService("publish_map", &CoreWrapper::publishMapCallback, this);
    getPlanSrv_ = nh.advertiseService("get_plan", &CoreWrapper::getPlanCallback, this);
    getPlanNodesSrv_ = nh.advertiseService("get_plan_nodes", &CoreWrapper::getPlanNodesCallback, this);
    setGoalSrv_ = nh.advertiseService("set_goal", &CoreWrapper::setGoalCallback, this);
    cancelGoalSrv_ = nh.advertiseService("cancel_goal", &CoreWrapper::cancelGoalCallback, this);
    setLabelSrv_ = nh.advertiseService("set_label", &CoreWrapper::setLabelCallback, this);
    listLabelsSrv_ = nh.advertiseService("list_labels", &CoreWrapper::listLabelsCallback, this);
    removeLabelSrv_ = nh.advertiseService("remove_label", &CoreWrapper::removeLabelCallback, this);
    addLinkSrv_ = nh.advertiseService("add_link", &CoreWrapper::addLinkCallback, this);
    getNodesInRadiusSrv_ = nh.advertiseService("get_nodes_in_radius", &CoreWrapper::getNodesInRadiusCallback, this);
#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
    octomapBinarySrv_ = nh.advertiseService("octomap_binary", &CoreWrapper::octomapBinaryCallback, this);
    octomapFullSrv_ = nh.advertiseService("octomap_full", &CoreWrapper::octomapFullCallback, this);
#endif
#endif
    // private services
    setLogDebugSrv_ = pnh.advertiseService("log_debug", &CoreWrapper::setLogDebug, this);
    setLogInfoSrv_ = pnh.advertiseService("log_info", &CoreWrapper::setLogInfo, this);
    setLogWarnSrv_ = pnh.advertiseService("log_warning", &CoreWrapper::setLogWarn, this);
    setLogErrorSrv_ = pnh.advertiseService("log_error", &CoreWrapper::setLogError, this);

    int optimizeIterations = 0;
    Parameters::parse(parameters_, Parameters::kOptimizerIterations(), optimizeIterations);
    if (publishTf && optimizeIterations != 0)
    {
      tfThreadRunning_ = true;
      transformThread_ = new boost::thread(boost::bind(&CoreWrapper::publishLoop, this, tfDelay, tfTolerance));
    }
    else if (publishTf)
    {
      NODELET_WARN("Graph optimization is disabled (%s=0), the tf between frame \"%s\" and odometry frame will not be published. You can safely ignore this warning if you are using map_optimizer node.",
                   Parameters::kOptimizerIterations().c_str(), mapFrameId_.c_str());
    }

    setupCallbacks(nh, pnh, getName()); // do it at the end
    if (!this->isDataSubscribed())
    {
      bool isRGBD = uStr2Bool(parameters_.at(Parameters::kRGBDEnabled()).c_str());
      if (isRGBD)
      {
        NODELET_WARN("ROS param subscribe_depth, subscribe_stereo and subscribe_rgbd are false, but RTAB-Map "
                     "parameter \"%s\" is true! Please set subscribe_depth, subscribe_stereo or subscribe_rgbd "
                     "to true to use rtabmap node for RGB-D SLAM, set \"%s\" to false for loop closure "
                     "detection on images-only or set subscribe_rgb to true to localize a single RGB camera against pre-built 3D map.",
                     Parameters::kRGBDEnabled().c_str(),
                     Parameters::kRGBDEnabled().c_str());
      }
      ros::NodeHandle rgb_nh(nh, "rgb");
      ros::NodeHandle rgb_pnh(pnh, "rgb");
      image_transport::ImageTransport rgb_it(rgb_nh);
      image_transport::TransportHints hintsRgb("raw", ros::TransportHints(), rgb_pnh);
      defaultSub_ = rgb_it.subscribe("image", 1, &CoreWrapper::defaultCallback, this);

      NODELET_INFO("\n%s subscribed to:\n   %s", getName().c_str(), defaultSub_.getTopic().c_str());
    }
    else if (!this->isSubscribedToDepth() &&
             !this->isSubscribedToStereo() &&
             !this->isSubscribedToRGBD() &&
             !this->isSubscribedToRGB() &&
             (this->isSubscribedToScan2d() || this->isSubscribedToScan3d() || this->isSubscribedToOdom()))
    {
      NODELET_WARN("There is no image subscription, bag-of-words loop closure detection will be disabled...");
      int kpMaxFeatures = Parameters::defaultKpMaxFeatures();
      int registrationStrategy = Parameters::defaultRegStrategy();
      Parameters::parse(parameters_, Parameters::kKpMaxFeatures(), kpMaxFeatures);
      Parameters::parse(parameters_, Parameters::kRegStrategy(), registrationStrategy);
      bool updateParams = false;
      if (kpMaxFeatures != -1)
      {
        uInsert(parameters_, ParametersPair(Parameters::kKpMaxFeatures(), "-1"));
        NODELET_WARN("Setting %s=-1 (bag-of-words disabled)", Parameters::kKpMaxFeatures().c_str());
        updateParams = true;
      }
      if ((this->isSubscribedToScan2d() || this->isSubscribedToScan3d()) && registrationStrategy != 1)
      {
        uInsert(parameters_, ParametersPair(Parameters::kRegStrategy(), "1"));
        NODELET_WARN("Setting %s=1 (ICP)", Parameters::kRegStrategy().c_str());
        updateParams = true;

        if (modifiedParameters.find(Parameters::kRGBDProximityPathMaxNeighbors()) == modifiedParameters.end())
        {
          if (this->isSubscribedToScan2d())
          {
            NODELET_WARN("Setting \"%s\" parameter to 10 (default 0) as \"subscribe_scan\" is "
                         "true and \"%s\" uses ICP. Proximity detection by space will be also done by merging close "
                         "scans. To disable, set \"%s\" to 0. To suppress this warning, "
                         "add <param name=\"%s\" type=\"string\" value=\"10\"/>",
                         Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                         Parameters::kRegStrategy().c_str(),
                         Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                         Parameters::kRGBDProximityPathMaxNeighbors().c_str());
            uInsert(parameters_, ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "10"));
          }
          else if (this->isSubscribedToScan3d())
          {
            NODELET_WARN("Setting \"%s\" parameter to 1 (default 0) as \"subscribe_scan_cloud\" is "
                         "true and \"%s\" uses ICP. To disable, set \"%s\" to 0. To suppress this warning, "
                         "add <param name=\"%s\" type=\"string\" value=\"1\"/>",
                         Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                         Parameters::kRegStrategy().c_str(),
                         Parameters::kRGBDProximityPathMaxNeighbors().c_str(),
                         Parameters::kRGBDProximityPathMaxNeighbors().c_str());
            uInsert(parameters_, ParametersPair(Parameters::kRGBDProximityPathMaxNeighbors(), "1"));
          }
        }
      }
      if (updateParams)
      {
        rtabmap_.parseParameters(parameters_);
      }
    }

    // set private parameters
    for (ParametersMap::iterator iter = parameters_.begin(); iter != parameters_.end(); ++iter)
    {
      pnh.setParam(iter->first, iter->second);
    }

    userDataAsyncSub_ = nh.subscribe("user_data_async", 1, &CoreWrapper::userDataAsyncCallback, this);
    globalPoseAsyncSub_ = nh.subscribe("global_pose", 1, &CoreWrapper::globalPoseAsyncCallback, this);
    gpsFixAsyncSub_ = nh.subscribe("gps/fix", 1, &CoreWrapper::gpsFixAsyncCallback, this);
#ifdef WITH_APRILTAG_ROS
    tagDetectionsSub_ = nh.subscribe("tag_detections", 1, &CoreWrapper::tagDetectionsAsyncCallback, this);
#endif
#ifdef WITH_FIDUCIAL_MSGS
    fiducialTransfromsSub_ = nh.subscribe("fiducial_transforms", 1, &CoreWrapper::fiducialDetectionsAsyncCallback, this);
#endif
    imuSub_ = nh.subscribe("imu", 100, &CoreWrapper::imuAsyncCallback, this);
    republishNodeDataSub_ = nh.subscribe("republish_node_data", 100, &CoreWrapper::republishNodeDataCallback, this);

    // JHUAPL section

    landmarksMapPub_ = nh.advertise<visualization_msgs::MarkerArray>("landmarks_map", 1);
    obstaclesDataPub_ = nh.advertise<rtabmap_ros::ObstaclesData>("obstacles_data", 1);
    mapManagerStatsPub_ = nh.advertise<rtabmap_ros::MapManagerStats>("map_manager_stats", 1);

    alive_publisher_ = nh.advertise<std_msgs::Empty>("heartbeat", 1);

    if (threadedMode_)
    {
      inputProcessThreadStatsPub_ = nh.advertise<rtabmap_ros::InputDataStats>("input_process_thread_stats", 1);
    }

    landmarkInsertSrv_ = nh.advertiseService("landmarks_insert", &CoreWrapper::landmarksInsertSrvCallback, this);
    landmarksQuerySrv_ = nh.advertiseService("landmarks_available", &CoreWrapper::landmarksQuerySrvCallback, this);
    landmarksRemoveSrv_ = nh.advertiseService("landmarks_remove", &CoreWrapper::landmarksRemoveSrvCallback, this);
    semanticDataAssociationSrv_ = nh.advertiseService("semantic_data_association_map", &CoreWrapper::semanticDataAssociationSrvCallback, this);
    baseToCamTransformUpdateSrc_ = nh.advertiseService("baseToCam_Transform", &CoreWrapper::baseToCamTransformSrvCallback, this);

    if (publishVisualImage)
    {
      ros::NodeHandle visual_nh(nh, "rgb");
      image_transport::ImageTransport visual_it(visual_nh);
      visualImagePub_ = visual_it.advertise("rgb_image_rect", 1);
    }

    if (publishDepthImage)
    {
      ros::NodeHandle depth_nh(nh, "depth");
      image_transport::ImageTransport depth_it(depth_nh);
      depthImagePub_ = depth_it.advertise("depth_image_rect", 1);
    }

    // THIS IS NOT WORKING IN NOETIC
    //float heartbeat_time_update = 0.5; // in seconds [0.01 to 1.0]
    //ros::Timer keepAliveTimer = nh.createTimer(ros::Duration(heartbeat_time_update), boost::bind(&CoreWrapper::aliveEventCb, this, _1));
    eventHandlerThread_ = new boost::thread(boost::bind(&CoreWrapper::heartbeatEventThread, this));

    // this thread is only used in this mode.
    if (threadedMode_)
    {
      rtabmapProcessThreadRunning_ = true;
      rtabmapProcessThread_ = new boost::thread(boost::bind(&CoreWrapper::rtabmapProcessThread, this));
    }

    mapManagerUpdateThreadRunning_ = true;
    mapManagerUpdateThread_ = new boost::thread(boost::bind(&CoreWrapper::mapManagerUpdateThread, this, mapManagerUpdateRate));

    // JHUAPL section end
  }

  CoreWrapper::~CoreWrapper()
  {
    if (transformThread_)
    {
      tfThreadRunning_ = false;
      transformThread_->join();
      delete transformThread_;
    }

    // JHUAPL section
    if (rtabmapProcessThread_)
    {
      rtabmapProcessThreadRunning_ = false;
      rtabmapProcessThread_->join();
      delete rtabmapProcessThread_;
    }
    // JHUAPL section  end

    this->saveParameters(configPath_);

    printf("rtabmap: Saving database/long-term memory... (located at %s)\n", databasePath_.c_str());
    rtabmap_mtx_.lock();
    if (rtabmap_.getMemory())
    {
      // save the grid map
      float xMin = 0.0f, yMin = 0.0f, gridCellSize = 0.05f;
      cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
      if (!pixels.empty())
      {
        printf("rtabmap: 2D occupancy grid map saved.\n");
        rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
      }
    }
    rtabmap_mtx_.unlock();

    // JHUAPL section
    if (mapManagerUpdateThread_)
    {
      mapManagerUpdateThreadRunning_ = false;
      mapManagerUpdateThread_->join();
      delete mapManagerUpdateThread_;
    }

    if (eventHandlerThread_)
    {
      // ROS::okay should end it
      eventHandlerThread_->join();
      delete eventHandlerThread_;
    }
    // JHUAPL section  end

    rtabmap_mtx_.lock();
    rtabmap_.close();
    rtabmap_mtx_.unlock();
    printf("rtabmap: Saving database/long-term memory...done! (located at %s, %ld MB)\n", databasePath_.c_str(), UFile::length(databasePath_) / (1024 * 1024));

    delete interOdomSync_;
    delete mbClient_;
  }

  void CoreWrapper::loadParameters(const std::string &configFile, ParametersMap &parameters)
  {
    if (!configFile.empty())
    {
      NODELET_INFO("Loading parameters from %s", configFile.c_str());
      if (!UFile::exists(configFile.c_str()))
      {
        NODELET_WARN("Config file doesn't exist! It will be generated...");
      }
      Parameters::readINI(configFile.c_str(), parameters);
    }
  }

  void CoreWrapper::saveParameters(const std::string &configFile)
  {
    if (!configFile.empty())
    {
      printf("Saving parameters to %s\n", configFile.c_str());

      if (!UFile::exists(configFile.c_str()))
      {
        printf("Config file doesn't exist, a new one will be created.\n");
      }
      Parameters::writeINI(configFile.c_str(), parameters_);
    }
    else
    {
      NODELET_INFO("Parameters are not saved! (No configuration file provided...)");
    }
  }

  void CoreWrapper::publishLoop(double tfDelay, double tfTolerance)
  {
    if (tfDelay == 0)
      return;
    ros::Rate r(1.0 / tfDelay);
    while (tfThreadRunning_)
    {
      if (!odomFrameId_.empty())
      {
        mapToOdomMutex_.lock();
        ros::Time tfExpiration = ros::Time::now() + ros::Duration(tfTolerance);
        geometry_msgs::TransformStamped msg;
        msg.child_frame_id = odomFrameId_;
        msg.header.frame_id = mapFrameId_;
        msg.header.stamp = tfExpiration;
        rtabmap_ros::transformToGeometryMsg(mapToOdom_, msg.transform);
        tfBroadcaster_.sendTransform(msg);
        mapToOdomMutex_.unlock();
      }
      r.sleep();
    }
  }

  void CoreWrapper::defaultCallback(const sensor_msgs::ImageConstPtr &imageMsg)
  {
    if (!paused_)
    {
      ros::Time stamp = imageMsg->header.stamp;
      if (stamp.toSec() == 0.0)
      {
        ROS_WARN("A null stamp has been detected in the input topic. Make sure the stamp is set.");
        return;
      }

      if (rate_ > 0.0f)
      {
        if (previousStamp_.toSec() > 0.0 && stamp.toSec() > previousStamp_.toSec() && stamp - previousStamp_ < ros::Duration(1.0f / rate_))
        {
          return;
        }
      }
      previousStamp_ = stamp;

      if (!(imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
            imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0 ||
            imageMsg->encoding.compare(sensor_msgs::image_encodings::BGR8) == 0 ||
            imageMsg->encoding.compare(sensor_msgs::image_encodings::RGB8) == 0))
      {
        NODELET_ERROR("Input type must be image=mono8,mono16,rgb8,bgr8");
        return;
      }

      cv_bridge::CvImageConstPtr ptrImage;
      if (imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO8) == 0 ||
          imageMsg->encoding.compare(sensor_msgs::image_encodings::MONO16) == 0)
      {
        ptrImage = cv_bridge::toCvShare(imageMsg, "mono8");
      }
      else
      {
        ptrImage = cv_bridge::toCvShare(imageMsg, "bgr8");
      }

      // process data
      UTimer timer;
      if (rtabmap_.isIDsGenerated() || ptrImage->header.seq > 0)
      {
        if (!rtabmap_.process(ptrImage->image.clone(), ptrImage->header.seq))
        {
          NODELET_WARN("RTAB-Map could not process the data received! (ROS id = %d)", ptrImage->header.seq);
        }
        else
        {
          this->publishStats(ros::Time::now());
        }
      }
      else if (!rtabmap_.isIDsGenerated())
      {
        NODELET_WARN("Ignoring received image because its sequence ID=0. Please "
                     "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
                     "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
                     "when you need to have IDs output of RTAB-map synchronised with the source "
                     "image sequence ID.");
      }
      NODELET_INFO("rtabmap: Update rate=%fs, Limit=%fs, Processing time = %fs (%d local nodes)",
                   1.0f / rate_,
                   rtabmap_.getTimeThreshold() / 1000.0f,
                   timer.ticks(),
                   rtabmap_.getWMSize() + rtabmap_.getSTMSize());
    }
  }

  bool CoreWrapper::odomUpdate(const nav_msgs::OdometryConstPtr &odomMsg, ros::Time stamp)
  {
    if (!paused_)
    {
      Transform odom = rtabmap_ros::transformFromPoseMsg(odomMsg->pose.pose);
      if (!odom.isNull())
      {
        Transform odomTF;
        if (!stamp.isZero())
        {
          odomTF = rtabmap_ros::getTransform(odomMsg->header.frame_id, frameId_, stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
        }
        if (odomTF.isNull())
        {
          static bool shown = false;
          if (!shown)
          {
            NODELET_WARN("We received odometry message, but we cannot get the "
                         "corresponding TF %s->%s at data stamp %fs (odom msg stamp is %fs). Make sure TF of odometry is "
                         "also published to get more accurate pose estimation. This "
                         "warning is only printed once.",
                         odomMsg->header.frame_id.c_str(), frameId_.c_str(), stamp.toSec(), odomMsg->header.stamp.toSec());
            shown = true;
          }
          stamp = odomMsg->header.stamp;
        }
        else
        {
          odom = odomTF;
        }
      }

      if (!lastPose_.isIdentity() && !odom.isNull() && (odom.isIdentity() || (odomMsg->pose.covariance[0] >= BAD_COVARIANCE && odomMsg->twist.covariance[0] >= BAD_COVARIANCE)))
      {
        UWARN("Odometry is reset (identity pose or high variance (%f) detected). Increment map id!", MAX(odomMsg->pose.covariance[0], odomMsg->twist.covariance[0]));
        rtabmap_.triggerNewMap();
        covariance_ = cv::Mat();
      }

      lastPoseIntermediate_ = false;
      lastPose_ = odom;
      lastPoseStamp_ = stamp;

      // Only update variance if odom is not null
      if (!odom.isNull())
      {
        cv::Mat covariance;
        double variance = odomMsg->twist.covariance[0];
        if (variance == BAD_COVARIANCE || variance <= 0.0f)
        {
          // use the one of the pose
          covariance = cv::Mat(6, 6, CV_64FC1, (void *)odomMsg->pose.covariance.data()).clone();
          covariance /= 2.0;
        }
        else
        {
          covariance = cv::Mat(6, 6, CV_64FC1, (void *)odomMsg->twist.covariance.data()).clone();
        }

        if (uIsFinite(covariance.at<double>(0, 0)) &&
            covariance.at<double>(0, 0) != 1.0 &&
            covariance.at<double>(0, 0) > 0.0)
        {
          // Use largest covariance error (to be independent of the odometry frame rate)
          if (covariance_.empty() || covariance.at<double>(0, 0) > covariance_.at<double>(0, 0))
          {
            covariance_ = covariance;
          }
        }
      }

      // Throttle
      bool ignoreFrame = false;
      if (stamp.toSec() == 0.0)
      {
        ROS_WARN("A null stamp has been detected in the input topics. Make sure the stamp in all input topics is set.");
        ignoreFrame = true;
      }
      if (rate_ > 0.0f)
      {
        if (previousStamp_.toSec() > 0.0 && stamp.toSec() > previousStamp_.toSec() && stamp - previousStamp_ < ros::Duration(1.0f / rate_))
        {
          ignoreFrame = true;
        }
      }
      if (ignoreFrame)
      {
        if (createIntermediateNodes_)
        {
          lastPoseIntermediate_ = true;
        }
        else
        {
          return false;
        }
      }
      else if (!ignoreFrame)
      {
        previousStamp_ = stamp;
      }

      return true;
    }
    return false;
  }

  bool CoreWrapper::odomTFUpdate(const ros::Time &stamp)
  {
    if (!paused_)
    {
      // Odom TF ready?
      Transform odom = rtabmap_ros::getTransform(odomFrameId_, frameId_, stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
      if (odom.isNull())
      {
        return false;
      }

      if (!lastPose_.isIdentity() && odom.isIdentity())
      {
        UWARN("Odometry is reset (identity pose detected). Increment map id!");
        rtabmap_.triggerNewMap();
        covariance_ = cv::Mat();
      }

      lastPoseIntermediate_ = false;
      lastPose_ = odom;
      lastPoseStamp_ = stamp;

      bool ignoreFrame = false;
      if (stamp.toSec() == 0.0)
      {
        ROS_WARN("A null stamp has been detected in the input topics. Make sure the stamp in all input topics is set.");
        ignoreFrame = true;
      }
      if (rate_ > 0.0f)
      {
        if (previousStamp_.toSec() > 0.0 && stamp.toSec() > previousStamp_.toSec() && stamp - previousStamp_ < ros::Duration(1.0f / rate_))
        {
          ignoreFrame = true;
        }
      }
      if (ignoreFrame)
      {
        if (createIntermediateNodes_)
        {
          lastPoseIntermediate_ = true;
        }
        else
        {
          return false;
        }
      }
      else if (!ignoreFrame)
      {
        previousStamp_ = stamp;
      }

      return true;
    }
    return false;
  }

  /****************************
   *	JHUAPL section
   *****************************/

  void CoreWrapper::commonDepthCallback(
      const nav_msgs::OdometryConstPtr &odomMsg,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const std::vector<cv_bridge::CvImageConstPtr> &imageMsgs,
      const std::vector<cv_bridge::CvImageConstPtr> &depthMsgs,
      const std::vector<cv_bridge::CvImageConstPtr> &semanticMaskMsgs,
      const std::vector<sensor_msgs::CameraInfo> &cameraInfoMsgs,
      const sensor_msgs::LaserScan &scan2dMsg,
      const sensor_msgs::PointCloud2 &scan3dMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg,
      const std::vector<rtabmap_ros::GlobalDescriptor> &globalDescriptorMsgs,
      const std::vector<std::vector<rtabmap_ros::KeyPoint>> &localKeyPoints,
      const std::vector<std::vector<rtabmap_ros::Point3f>> &localPoints3d,
      const std::vector<cv::Mat> &localDescriptors)
  {
    std::string odomFrameId = odomFrameId_;
    if (odomMsg.get())
    {
      odomFrameId = odomMsg->header.frame_id;
      if (!scan2dMsg.ranges.empty())
      {
        if (!odomUpdate(odomMsg, scan2dMsg.header.stamp))
        {
          return;
        }
      }
      else if (!scan3dMsg.data.empty())
      {
        if (!odomUpdate(odomMsg, scan3dMsg.header.stamp))
        {
          return;
        }
      }
      else if (cameraInfoMsgs.size() == 0 || !odomUpdate(odomMsg, cameraInfoMsgs[0].header.stamp))
      {
        return;
      }
    }
    else if (!scan2dMsg.ranges.empty())
    {
      if (!odomTFUpdate(scan2dMsg.header.stamp))
      {
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!odomTFUpdate(scan3dMsg.header.stamp))
      {
        return;
      }
    }
    else if (cameraInfoMsgs.size() == 0 || !odomTFUpdate(cameraInfoMsgs[0].header.stamp))
    {
      return;
    }

    commonDepthCallbackImpl(odomFrameId,
                            userDataMsg,
                            imageMsgs,
                            depthMsgs,
                            semanticMaskMsgs,
                            cameraInfoMsgs,
                            scan2dMsg,
                            scan3dMsg,
                            odomInfoMsg,
                            globalDescriptorMsgs,
                            localKeyPoints,
                            localPoints3d,
                            localDescriptors);
  }

  void CoreWrapper::commonDepthCallbackImpl(
      const std::string &odomFrameId,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const std::vector<cv_bridge::CvImageConstPtr> &imageMsgs,
      const std::vector<cv_bridge::CvImageConstPtr> &depthMsgs,
      const std::vector<cv_bridge::CvImageConstPtr> &semanticMaskMsgs,
      const std::vector<sensor_msgs::CameraInfo> &cameraInfoMsgs,
      const sensor_msgs::LaserScan &scan2dMsg,
      const sensor_msgs::PointCloud2 &scan3dMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg,
      const std::vector<rtabmap_ros::GlobalDescriptor> &globalDescriptorMsgs,
      const std::vector<std::vector<rtabmap_ros::KeyPoint>> &localKeyPointsMsgs,
      const std::vector<std::vector<rtabmap_ros::Point3f>> &localPoints3dMsgs,
      const std::vector<cv::Mat> &localDescriptorsMsgs)
  {
    UTimer timerConversion;
    cv::Mat rgb;
    cv::Mat depth;
    std::vector<rtabmap::CameraModel> cameraModels;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point3f> points;
    cv::Mat descriptors;
    if (!rtabmap_ros::convertRGBDMsgs(
            imageMsgs,
            depthMsgs,
            cameraInfoMsgs,
            frameId_,
            odomSensorSync_ ? odomFrameId : "",
            lastPoseStamp_,
            rgb,
            depth,
            cameraModels,
            tfListener_,
            waitForTransform_ ? waitForTransformDuration_ : 0.0,
            localKeyPointsMsgs,
            localPoints3dMsgs,
            localDescriptorsMsgs,
            &keypoints,
            &points,
            &descriptors))
    {
      NODELET_ERROR("Could not convert rgb/depth msgs! Aborting rtabmap update...");
      return;
    }

    // semantic mask matrix corresponds to the class number identifier
    // it assumes that the size does not reflect rgb image size
    // it assumes the input rgb has been rectified
    cv::Mat semanticMask, semanticMaskResized;
    if (!rtabmap_ros::cvImageConstPtrToCvMat(semanticMaskMsgs, semanticMask))
    {
      NODELET_ERROR("Could not convert semantic mask msgs! Aborting rtabmap update...");
      return;
    }
    // resizing the semantic mask to correspond the rgb input image size
    if (semanticMask.rows != rgb.rows ||
        semanticMask.cols != rgb.cols)
    {
#ifdef OPENCV_CUDA
      cv::cuda::GpuMat semanticMaskGpu(semanticMask), semanticMaskResizedGpu;
      cv::cuda::resize(semanticMaskGpu, semanticMaskResizedGpu, cv::Size(rgb.cols, rgb.rows), cv::INTER_NEAREST);
      semanticMaskResizedGpu.download(semanticMaskResized);
#else
      cv::resize(semanticMask, semanticMaskResized, cv::Size(rgb.cols, rgb.rows), cv::INTER_NEAREST);
#endif
    }
    else
    {
      semanticMaskResized = semanticMask;
    }

    UASSERT(uContains(parameters_, rtabmap::Parameters::kMemSaveDepth16Format()));
    if (!depth.empty() && depth.type() == CV_32FC1 && uStr2Bool(parameters_.at(Parameters::kMemSaveDepth16Format())))
    {
      depth = rtabmap::util2d::cvtDepthFromFloat(depth);
      static bool shown = false;
      if (!shown)
      {
        NODELET_WARN("Save depth data to 16 bits format: depth type detected is "
                     "32FC1, use 16UC1 depth format to avoid this conversion "
                     "(or set parameter \"Mem/SaveDepth16Format=false\" to use "
                     "32bits format). This message is only printed once...");
        shown = true;
      }
    }

    // depth pre-processing
    if (depthFilters_)
    {
      // UTimer filterTimer;
      // filterTimer.start();
      utils::depthEdgeFilter(depth);
      // NODELET_INFO("===> depth Edge filter time: %f", filterTimer.ticks());
    }

    LaserScan scan;
    bool genMaxScanPts = 0;
    if (scan2dMsg.ranges.empty() && scan3dMsg.data.empty() && !depth.empty() && genScan_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud2d(new pcl::PointCloud<pcl::PointXYZ>);
      *scanCloud2d = util3d::laserScanFromDepthImages(
          depth,
          cameraModels,
          genScanMaxDepth_,
          genScanMinDepth_);
      genMaxScanPts += depth.cols;
      scan = LaserScan(rtabmap::util3d::laserScan2dFromPointCloud(*scanCloud2d), 0, genScanMaxDepth_);
    }
    else if (!scan2dMsg.ranges.empty())
    {
      if (!rtabmap_ros::convertScanMsg(
              scan2dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              // backward compatibility, project 2D scan in /base_link frame
              rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
      {
        NODELET_ERROR("Could not convert laser scan msg! Aborting rtabmap update...");
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!rtabmap_ros::convertScan3dMsg(
              scan3dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              scanCloudMaxPoints_))
      {
        NODELET_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap update...");
        return;
      }

      ROS_DEBUG("%d %d %d %d", rgb.empty() ? 1 : 0, depth.empty() ? 1 : 0, scan.isEmpty() ? 1 : 0, genDepth_ ? 1 : 0);
      if (!rgb.empty() && depth.empty() && !scan.isEmpty() && genDepth_)
      {
        for (size_t i = 0; i < cameraModels.size(); ++i)
        {
          rtabmap::CameraModel model = cameraModels[i];
          if (genDepthDecimation_ > 1)
          {
            if (model.imageWidth() % genDepthDecimation_ == 0 && model.imageHeight() % genDepthDecimation_ == 0)
            {
              model = model.scaled(1.0f / float(genDepthDecimation_));
            }
            else
            {
              ROS_ERROR("decimation (%d) not valid for image size %dx%d! Aborting depth generation from scan...",
                        genDepthDecimation_,
                        model.imageWidth(),
                        model.imageHeight());
              depth = cv::Mat();
              break;
            }
          }

          cv::Mat depthProjected = util3d::projectCloudToCamera(
              model.imageSize(),
              model.K(),
              scan.data(),
              scan.localTransform().inverse() * model.localTransform());

          if (genDepthFillHolesSize_ > 0 && genDepthFillIterations_ > 0)
          {
            for (int i = 0; i < genDepthFillIterations_; ++i)
            {
              depthProjected = util2d::fillDepthHoles(
                  depthProjected,
                  genDepthFillHolesSize_,
                  genDepthFillHolesError_);
            }
          }

          if (depth.empty())
          {
            depth = cv::Mat::zeros(model.imageHeight(), model.imageWidth() * cameraModels.size(), CV_32FC1);
          }
          depthProjected.copyTo(depth.colRange(i * model.imageWidth(), (i + 1) * model.imageWidth()));
        }
      }
    }

    cv::Mat userData;
    if (userDataMsg.get())
    {
      userData = rtabmap_ros::userDataFromROS(*userDataMsg);
      UScopeMutex lock(userDataMutex_);
      if (!userData_.empty())
      {
        NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
        userData_ = cv::Mat();
      }
    }
    else
    {
      UScopeMutex lock(userDataMutex_);
      userData = userData_;
      userData_ = cv::Mat();
    }

    SensorData data(
        scan,
        rgb,
        depth,
        cameraModels,
        semanticMaskResized,
        lastPoseIntermediate_ ? -1 : !cameraInfoMsgs.empty() ? cameraInfoMsgs[0].header.seq
                                                             : 0,
        rtabmap_ros::timestampFromROS(lastPoseStamp_),
        userData);

    OdometryInfo odomInfo;
    if (odomInfoMsg.get())
    {
      odomInfo = odomInfoFromROS(*odomInfoMsg);
    }

    if (!globalDescriptorMsgs.empty())
    {
      data.setGlobalDescriptors(rtabmap_ros::globalDescriptorsFromROS(globalDescriptorMsgs));
    }

    if (!keypoints.empty())
    {
      UASSERT(points.empty() || points.size() == keypoints.size());
      UASSERT(descriptors.empty() || descriptors.rows == (int)keypoints.size());
      data.setFeatures(keypoints, points, descriptors);
    }

    process(lastPoseStamp_,
            data,
            lastPose_,
            odomFrameId,
            covariance_,
            odomInfo,
            timerConversion.ticks());
    covariance_ = cv::Mat();
  }

  void CoreWrapper::publishLandmarksMap(const rtabmap::Memory *memory, const std::string &mapFrameId)
  {
    if (memory == 0)
    {
      return;
    }

    if (landmarksMapPub_.getNumSubscribers())
    {
      // [ signature id , set containing landmarks id]
      const std::map<int, std::set<int>> landmarksIndex = memory->getLandmarksIndex();

      visualization_msgs::MarkerArray markers;
      for (std::map<int, std::set<int>>::const_iterator lmIter = landmarksIndex.begin(); lmIter != landmarksIndex.end(); ++lmIter)
      {
        // this is the actual landmark id
        int landMarkId = -1 * lmIter->first;
        // we need one signature per landmark
        int signatureId = *lmIter->second.begin();
        // UDEBUG(" landmark:%d ; signature: %d; set size: %d", landMarkId, signatureId, (unsigned int)lmIter->second.size());

        std::map<int, rtabmap::Link> landmarksLinkMap;
        const rtabmap::Signature *s = memory->getSignature(signatureId);
        if (s)
        {
          landmarksLinkMap = s->getLandmarks();
        }

        if (!landmarksLinkMap.empty())
        {
          // compute the pose of landmark with respect to map reference frame
          // assuming the landmark's node is in the optimized map.
          rtabmap::Transform T_map_nodePose = rtabmap_.getPose(signatureId);

          // check if the node id pose was found; if not found, it would be all zeros
          if (T_map_nodePose.isNull())
          {
            T_map_nodePose = s->getPose();
            if (T_map_nodePose.isNull())
            {
              ROS_ERROR("		signature id POSE not found in memory!!! THIS SHOULD NOT HAPPEN");
              continue;
            }
          }

          visualization_msgs::Marker marker;
          std::map<int, rtabmap::Link>::iterator lIter = landmarksLinkMap.find(lmIter->first);

          if (lIter == landmarksLinkMap.end())
          {
            ROS_WARN("landmarks link Map did not contain the landmark id !!");
            continue;
          }

          // lIter->second.transform() -> T_nodePose_landmarkPose
          rtabmap::Transform T_map_landmarkPose = T_map_nodePose * lIter->second.transform();
          Eigen::Quaterniond quaternion = T_map_landmarkPose.getQuaterniond();

          marker.header.frame_id = mapFrameId;
          marker.header.stamp = ros::Time::now();
          marker.ns = "landmarks";
          marker.id = landMarkId;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = T_map_landmarkPose.x();
          marker.pose.position.y = T_map_landmarkPose.y();
          marker.pose.position.z = T_map_landmarkPose.z();
          marker.pose.orientation.x = quaternion.x();
          marker.pose.orientation.y = quaternion.y();
          marker.pose.orientation.z = quaternion.z();
          marker.pose.orientation.w = quaternion.w();
          marker.scale.x = 0.20;
          marker.scale.y = 0.20;
          marker.scale.z = 0.20;
          marker.color.a = 0.5;
          marker.color.r = 1.0;
          marker.color.g = 0.70;
          marker.color.b = 0.0;
          marker.lifetime = ros::Duration((1.f / rate_) * 4);

          marker.type = visualization_msgs::Marker::CUBE;

          markers.markers.push_back(marker);
        }
      }
      landmarksMapPub_.publish(markers);
    }
  }

  ///
  /// ROS callback functions
  ///

  bool CoreWrapper::landmarksInsertSrvCallback(rtabmap_ros::LandmarksInsert::Request &req, rtabmap_ros::LandmarksInsert::Response &res)
  {
    if (req.landmarks.empty())
    {
      ROS_WARN("landmark msg is empty!!");
      return false;
    }

    rtabmap_mtx_.lock();
    for (int i = 0; i < req.landmarks.size(); ++i)
    {
      rtabmap_ros::Landmark landmarkMsg = req.landmarks.at(i);
      double timeStamp = landmarkMsg.timeStamp;
      int landmarkId = landmarkMsg.landmarkId;
      int signatureId = landmarkMsg.nodeId;
      std::string description = landmarkMsg.description;

      if (timeStamp == 0 && signatureId <= 0)
      {
        ROS_WARN("landmark (%d) msg is missing timestamp", landmarkId);
        res.added.push_back(false);
        continue;
      }

      rtabmap::Transform landmarkPose = transformFromGeometryMsg(landmarkMsg.landmarkPose);
      rtabmap::Transform odometryPose = transformFromGeometryMsg(landmarkMsg.odometryPose);
      cv::Mat covariance = cv::Mat(6, 6, CV_64FC1, (void *)landmarkMsg.covariance.data()).clone();

      if (landmarkPose.isNull())
      {
        ROS_WARN("landmark (%d) msg is missing landmarkPose", landmarkId);
        res.added.push_back(false);
        continue;
      }
      if (odometryPose.isNull())
      {
        ROS_WARN("landmark (%d) msg is missing odometryPose", landmarkId);
        res.added.push_back(false);
        continue;
      }

      if (!rtabmap_.insertLandmark(landmarkId, timeStamp, signatureId, description, landmarkPose, covariance, odometryPose, LandmarkToMultiSignatureId_))
      {
        ROS_WARN("adding landmark (%d) timestamped (%lf) failed!", landmarkId, timeStamp);
        res.added.push_back(false);
      }
      res.added.push_back(true);
    }
    rtabmap_mtx_.unlock();
    return true;
  }

  bool CoreWrapper::landmarksQuerySrvCallback(rtabmap_ros::LandmarksQuery::Request &req, rtabmap_ros::LandmarksQuery::Response &res)
  {
    std::map<int, std::vector<rtabmap::Link>> landmarks;
    // gets all landmarks in the working map
    rtabmap_mtx_.lock();
    rtabmap_.getLandmarks(landmarks, req.maxRange);

    int index = 0;
    for (std::map<int, std::vector<rtabmap::Link>>::iterator lIter = landmarks.begin(); lIter != landmarks.end(); ++lIter, ++index)
    {
      // the "landmarks" map structure holds a list of link objects, corresponding to every signature id linkting to this (instance) landmark
      for (auto setLinkIter = lIter->second.begin(); setLinkIter != lIter->second.end(); ++setLinkIter)
      {
        rtabmap_ros::Landmark landmark;
        // the landmarks are negative in rtabmap, correcting to actual landmark id
        landmark.landmarkId = -1 * lIter->first;
        landmark.landmarkMapId = lIter->first;

        landmark.description = setLinkIter->getDescription();
        landmark.nodeId = setLinkIter->from();

        // compute the pose of landmark with respect to map reference frame
        // assuming the landmark's node is in the optimized map.
        rtabmap::Transform map2NodePose = rtabmap_.getPose(landmark.nodeId);
        // check if the node id pose was found; if not found, it would be all zeros
        if (map2NodePose.isNull())
        {
          // node with landmarks should always be in the optimized map ...
          ROS_WARN("signature id POSE not found!!! in optimized node map.");
          if (rtabmap_.getMemory())
          {
            // nodeid is the same as the signatureId
            const rtabmap::Signature *s = rtabmap_.getMemory()->getSignature(landmark.nodeId);
            map2NodePose = s->getPose();

            if (map2NodePose.isNull())
            {
              ROS_ERROR("signature id POSE not found in memory!!! THIS SHOULD NOT HAPPEN");
              return false;
            }
          }
          else
          {
            ROS_ERROR("signature id POSE not found!!! THIS SHOULD NOT HAPPEN");
            return false;
          }
        }

        // landmark pose with world coords
        rtabmap::Transform mapPose2landmarkPoseTf = map2NodePose * setLinkIter->transform();

        transformToGeometryMsg(setLinkIter->transform(), landmark.landmarkPose);
        transformToGeometryMsg(map2NodePose, landmark.odometryPose);
        transformToGeometryMsg(mapPose2landmarkPoseTf, landmark.landmarkPose_WorldCoords);

        if (setLinkIter->infMatrix().type() == CV_64FC1 &&
            setLinkIter->infMatrix().cols == 6 &&
            setLinkIter->infMatrix().rows == 6)
        {
          memcpy(landmark.covariance.data(), setLinkIter->infMatrix().data, 36 * sizeof(double));
        }

        res.landmarks.push_back(landmark);
      }
    }
    rtabmap_mtx_.unlock();

    return true;
  }

  bool CoreWrapper::landmarksRemoveSrvCallback(rtabmap_ros::LandmarksRemove::Request &req, rtabmap_ros::LandmarksRemove::Response &res)
  {
    rtabmap_mtx_.lock();
    if (req.removeAll)
    {
      std::vector<int> landmarkMapIds;
      std::vector<uint8_t> removedLandmarks;
      // remove all landmarks
      rtabmap_.removeLandmarks(landmarkMapIds, removedLandmarks, true);
    }
    else
    {
      rtabmap_.removeLandmarks(req.landmarkMapIds, res.landmarksRemoved);
    }
    rtabmap_mtx_.unlock();
    return true;
  }

  /****************************
   *	JHUAPL end of section
   *****************************/

  void CoreWrapper::commonDepthCallback(
      const nav_msgs::OdometryConstPtr &odomMsg,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const std::vector<cv_bridge::CvImageConstPtr> &imageMsgs,
      const std::vector<cv_bridge::CvImageConstPtr> &depthMsgs,
      const std::vector<sensor_msgs::CameraInfo> &cameraInfoMsgs,
      const sensor_msgs::LaserScan &scan2dMsg,
      const sensor_msgs::PointCloud2 &scan3dMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg,
      const std::vector<rtabmap_ros::GlobalDescriptor> &globalDescriptorMsgs,
      const std::vector<std::vector<rtabmap_ros::KeyPoint>> &localKeyPoints,
      const std::vector<std::vector<rtabmap_ros::Point3f>> &localPoints3d,
      const std::vector<cv::Mat> &localDescriptors)
  {
    std::string odomFrameId = odomFrameId_;
    if (odomMsg.get())
    {
      odomFrameId = odomMsg->header.frame_id;
      if (!scan2dMsg.ranges.empty())
      {
        if (!odomUpdate(odomMsg, scan2dMsg.header.stamp))
        {
          return;
        }
      }
      else if (!scan3dMsg.data.empty())
      {
        if (!odomUpdate(odomMsg, scan3dMsg.header.stamp))
        {
          return;
        }
      }
      else if (cameraInfoMsgs.size() == 0 || !odomUpdate(odomMsg, cameraInfoMsgs[0].header.stamp))
      {
        return;
      }
    }
    else if (!scan2dMsg.ranges.empty())
    {
      if (!odomTFUpdate(scan2dMsg.header.stamp))
      {
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!odomTFUpdate(scan3dMsg.header.stamp))
      {
        return;
      }
    }
    else if (cameraInfoMsgs.size() == 0 || !odomTFUpdate(cameraInfoMsgs[0].header.stamp))
    {
      return;
    }

    commonDepthCallbackImpl(odomFrameId,
                            userDataMsg,
                            imageMsgs,
                            depthMsgs,
                            cameraInfoMsgs,
                            scan2dMsg,
                            scan3dMsg,
                            odomInfoMsg,
                            globalDescriptorMsgs,
                            localKeyPoints,
                            localPoints3d,
                            localDescriptors);
  }

  void CoreWrapper::commonDepthCallbackImpl(
      const std::string &odomFrameId,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const std::vector<cv_bridge::CvImageConstPtr> &imageMsgs,
      const std::vector<cv_bridge::CvImageConstPtr> &depthMsgs,
      const std::vector<sensor_msgs::CameraInfo> &cameraInfoMsgs,
      const sensor_msgs::LaserScan &scan2dMsg,
      const sensor_msgs::PointCloud2 &scan3dMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg,
      const std::vector<rtabmap_ros::GlobalDescriptor> &globalDescriptorMsgs,
      const std::vector<std::vector<rtabmap_ros::KeyPoint>> &localKeyPointsMsgs,
      const std::vector<std::vector<rtabmap_ros::Point3f>> &localPoints3dMsgs,
      const std::vector<cv::Mat> &localDescriptorsMsgs)
  {
    UTimer timerConversion;
    cv::Mat rgb;
    cv::Mat depth;
    std::vector<rtabmap::CameraModel> cameraModels;
    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point3f> points;
    cv::Mat descriptors;
    if (!rtabmap_ros::convertRGBDMsgs(
            imageMsgs,
            depthMsgs,
            cameraInfoMsgs,
            frameId_,
            odomSensorSync_ ? odomFrameId : "",
            lastPoseStamp_,
            rgb,
            depth,
            cameraModels,
            tfListener_,
            waitForTransform_ ? waitForTransformDuration_ : 0.0,
            localKeyPointsMsgs,
            localPoints3dMsgs,
            localDescriptorsMsgs,
            &keypoints,
            &points,
            &descriptors))
    {
      NODELET_ERROR("Could not convert rgb/depth msgs! Aborting rtabmap update...");
      return;
    }

    UASSERT(uContains(parameters_, rtabmap::Parameters::kMemSaveDepth16Format()));
    if (!depth.empty() && depth.type() == CV_32FC1 && uStr2Bool(parameters_.at(Parameters::kMemSaveDepth16Format())))
    {
      depth = rtabmap::util2d::cvtDepthFromFloat(depth);
      static bool shown = false;
      if (!shown)
      {
        NODELET_WARN("Save depth data to 16 bits format: depth type detected is "
                     "32FC1, use 16UC1 depth format to avoid this conversion "
                     "(or set parameter \"Mem/SaveDepth16Format=false\" to use "
                     "32bits format). This message is only printed once...");
        shown = true;
      }
    }

    // JHUAPL section

    // depth pre-processing
    if (depthFilters_)
    {
      utils::depthEdgeFilter(depth);
    }

    // JHUAPL section end

    LaserScan scan;
    bool genMaxScanPts = 0;
    if (scan2dMsg.ranges.empty() && scan3dMsg.data.empty() && !depth.empty() && genScan_)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud2d(new pcl::PointCloud<pcl::PointXYZ>);
      *scanCloud2d = util3d::laserScanFromDepthImages(
          depth,
          cameraModels,
          genScanMaxDepth_,
          genScanMinDepth_);
      genMaxScanPts += depth.cols;
      scan = LaserScan(rtabmap::util3d::laserScan2dFromPointCloud(*scanCloud2d), 0, genScanMaxDepth_);
    }
    else if (!scan2dMsg.ranges.empty())
    {
      if (!rtabmap_ros::convertScanMsg(
              scan2dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              // backward compatibility, project 2D scan in /base_link frame
              rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
      {
        NODELET_ERROR("Could not convert laser scan msg! Aborting rtabmap update...");
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!rtabmap_ros::convertScan3dMsg(
              scan3dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              scanCloudMaxPoints_))
      {
        NODELET_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap update...");
        return;
      }

      ROS_DEBUG("%d %d %d %d", rgb.empty() ? 1 : 0, depth.empty() ? 1 : 0, scan.isEmpty() ? 1 : 0, genDepth_ ? 1 : 0);
      if (!rgb.empty() && depth.empty() && !scan.isEmpty() && genDepth_)
      {
        for (size_t i = 0; i < cameraModels.size(); ++i)
        {
          rtabmap::CameraModel model = cameraModels[i];
          if (genDepthDecimation_ > 1)
          {
            if (model.imageWidth() % genDepthDecimation_ == 0 && model.imageHeight() % genDepthDecimation_ == 0)
            {
              model = model.scaled(1.0f / float(genDepthDecimation_));
            }
            else
            {
              ROS_ERROR("decimation (%d) not valid for image size %dx%d! Aborting depth generation from scan...",
                        genDepthDecimation_,
                        model.imageWidth(),
                        model.imageHeight());
              depth = cv::Mat();
              break;
            }
          }

          cv::Mat depthProjected = util3d::projectCloudToCamera(
              model.imageSize(),
              model.K(),
              scan.data(),
              scan.localTransform().inverse() * model.localTransform());

          if (genDepthFillHolesSize_ > 0 && genDepthFillIterations_ > 0)
          {
            for (int i = 0; i < genDepthFillIterations_; ++i)
            {
              depthProjected = util2d::fillDepthHoles(
                  depthProjected,
                  genDepthFillHolesSize_,
                  genDepthFillHolesError_);
            }
          }

          if (depth.empty())
          {
            depth = cv::Mat::zeros(model.imageHeight(), model.imageWidth() * cameraModels.size(), CV_32FC1);
          }
          depthProjected.copyTo(depth.colRange(i * model.imageWidth(), (i + 1) * model.imageWidth()));
        }
      }
    }

    cv::Mat userData;
    if (userDataMsg.get())
    {
      userData = rtabmap_ros::userDataFromROS(*userDataMsg);
      UScopeMutex lock(userDataMutex_);
      if (!userData_.empty())
      {
        NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
        userData_ = cv::Mat();
      }
    }
    else
    {
      UScopeMutex lock(userDataMutex_);
      userData = userData_;
      userData_ = cv::Mat();
    }

    SensorData data(
        scan,
        rgb,
        depth,
        cameraModels,
        lastPoseIntermediate_ ? -1 : !cameraInfoMsgs.empty() ? cameraInfoMsgs[0].header.seq
                                                             : 0,
        rtabmap_ros::timestampFromROS(lastPoseStamp_),
        userData);

    OdometryInfo odomInfo;
    if (odomInfoMsg.get())
    {
      odomInfo = odomInfoFromROS(*odomInfoMsg);
    }

    if (!globalDescriptorMsgs.empty())
    {
      data.setGlobalDescriptors(rtabmap_ros::globalDescriptorsFromROS(globalDescriptorMsgs));
    }

    if (!keypoints.empty())
    {
      UASSERT(points.empty() || points.size() == keypoints.size());
      UASSERT(descriptors.empty() || descriptors.rows == (int)keypoints.size());
      data.setFeatures(keypoints, points, descriptors);
    }

    process(lastPoseStamp_,
            data,
            lastPose_,
            odomFrameId,
            covariance_,
            odomInfo,
            timerConversion.ticks());
    covariance_ = cv::Mat();
  }

  void CoreWrapper::commonStereoCallback(
      const nav_msgs::OdometryConstPtr &odomMsg,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const cv_bridge::CvImageConstPtr &leftImageMsg,
      const cv_bridge::CvImageConstPtr &rightImageMsg,
      const sensor_msgs::CameraInfo &leftCamInfoMsg,
      const sensor_msgs::CameraInfo &rightCamInfoMsg,
      const sensor_msgs::LaserScan &scan2dMsg,
      const sensor_msgs::PointCloud2 &scan3dMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg,
      const std::vector<rtabmap_ros::GlobalDescriptor> &globalDescriptorMsgs,
      const std::vector<rtabmap_ros::KeyPoint> &localKeyPointsMsg,
      const std::vector<rtabmap_ros::Point3f> &localPoints3dMsg,
      const cv::Mat &localDescriptorsMsg)
  {
    UTimer timerConversion;
    std::string odomFrameId = odomFrameId_;
    if (odomMsg.get())
    {
      odomFrameId = odomMsg->header.frame_id;
      if (!scan2dMsg.ranges.empty())
      {
        if (!odomUpdate(odomMsg, scan2dMsg.header.stamp))
        {
          return;
        }
      }
      else if (!scan3dMsg.data.empty())
      {
        if (!odomUpdate(odomMsg, scan3dMsg.header.stamp))
        {
          return;
        }
      }
      else if (leftImageMsg.get() == 0 || !odomUpdate(odomMsg, leftImageMsg->header.stamp))
      {
        return;
      }
    }
    else if (!scan2dMsg.ranges.empty())
    {
      if (!odomTFUpdate(scan2dMsg.header.stamp))
      {
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!odomTFUpdate(scan3dMsg.header.stamp))
      {
        return;
      }
    }
    else if (leftImageMsg.get() == 0 || !odomTFUpdate(leftImageMsg->header.stamp))
    {
      return;
    }

    cv::Mat left;
    cv::Mat right;
    StereoCameraModel stereoModel;
    if (!rtabmap_ros::convertStereoMsg(
            leftImageMsg,
            rightImageMsg,
            leftCamInfoMsg,
            rightCamInfoMsg,
            frameId_,
            odomSensorSync_ ? odomFrameId : "",
            lastPoseStamp_,
            left,
            right,
            stereoModel,
            tfListener_,
            waitForTransform_ ? waitForTransformDuration_ : 0.0,
            alreadyRectifiedImages_))
    {
      NODELET_ERROR("Could not convert stereo msgs! Aborting rtabmap update...");
      return;
    }

    if (stereoToDepth_)
    {
      // cv::stereoBM() see "$ rosrun rtabmap_ros rtabmap --params | grep StereoBM" for parameters
      cv::Mat disparity = rtabmap::util2d::disparityFromStereoImages(
          left,
          right,
          parameters_);
      if (disparity.empty())
      {
        NODELET_ERROR("Could not compute disparity image (\"stereo_to_depth\" is true)!");
        return;
      }
      cv::Mat depth = rtabmap::util2d::depthFromDisparity(
          disparity,
          stereoModel.left().fx(),
          stereoModel.baseline());

      if (depth.empty())
      {
        NODELET_ERROR("Could not compute depth image (\"stereo_to_depth\" is true)!");
        return;
      }
      UASSERT(depth.type() == CV_16UC1 || depth.type() == CV_32FC1);

      // move to common depth callback
      cv_bridge::CvImagePtr imgDepth(new cv_bridge::CvImage);
      if (depth.type() == CV_16UC1)
      {
        imgDepth->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
      }
      else // CV_32FC1
      {
        imgDepth->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
      }
      imgDepth->image = depth;
      imgDepth->header = leftImageMsg->header;
      std::vector<cv_bridge::CvImageConstPtr> rgbImages(1);
      std::vector<cv_bridge::CvImageConstPtr> depthImages(1);
      std::vector<sensor_msgs::CameraInfo> cameraInfos(1);
      rgbImages[0] = leftImageMsg;
      depthImages[0] = imgDepth;
      cameraInfos[0] = leftCamInfoMsg;

      std::vector<std::vector<rtabmap_ros::KeyPoint>> localKeyPointsMsgs;
      std::vector<std::vector<rtabmap_ros::Point3f>> localPoints3dMsgs;
      std::vector<cv::Mat> localDescriptorsMsgs;
      if (!localKeyPointsMsg.empty())
      {
        localKeyPointsMsgs.push_back(localKeyPointsMsg);
      }
      if (!localPoints3dMsg.empty())
      {
        localPoints3dMsgs.push_back(localPoints3dMsg);
      }
      if (!localDescriptorsMsg.empty())
      {
        localDescriptorsMsgs.push_back(localDescriptorsMsg);
      }
      commonDepthCallbackImpl(odomFrameId,
                              rtabmap_ros::UserDataConstPtr(),
                              rgbImages, depthImages, cameraInfos,
                              scan2dMsg, scan3dMsg,
                              odomInfoMsg,
                              globalDescriptorMsgs, localKeyPointsMsgs, localPoints3dMsgs, localDescriptorsMsgs);
      return;
    }

    LaserScan scan;
    if (!scan2dMsg.ranges.empty())
    {
      if (!rtabmap_ros::convertScanMsg(
              scan2dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              // backward compatibility, project 2D scan in /base_link frame
              rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
      {
        NODELET_ERROR("Could not convert laser scan msg! Aborting rtabmap update...");
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!rtabmap_ros::convertScan3dMsg(
              scan3dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              scanCloudMaxPoints_))
      {
        NODELET_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap update...");
        return;
      }
    }

    cv::Mat userData;
    if (userDataMsg.get())
    {
      userData = rtabmap_ros::userDataFromROS(*userDataMsg);
      UScopeMutex lock(userDataMutex_);
      if (!userData_.empty())
      {
        NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
        userData_ = cv::Mat();
      }
    }
    else
    {
      UScopeMutex lock(userDataMutex_);
      userData = userData_;
      userData_ = cv::Mat();
    }

    SensorData data(
        scan,
        left,
        right,
        stereoModel,
        lastPoseIntermediate_ ? -1 : leftImageMsg->header.seq,
        rtabmap_ros::timestampFromROS(lastPoseStamp_),
        userData);

    OdometryInfo odomInfo;
    if (odomInfoMsg.get())
    {
      odomInfo = odomInfoFromROS(*odomInfoMsg);
    }

    if (!globalDescriptorMsgs.empty())
    {
      data.setGlobalDescriptors(rtabmap_ros::globalDescriptorsFromROS(globalDescriptorMsgs));
    }

    std::vector<cv::KeyPoint> keypoints;
    std::vector<cv::Point3f> points;
    if (!localKeyPointsMsg.empty())
    {
      keypoints = rtabmap_ros::keypointsFromROS(localKeyPointsMsg);
    }
    if (!localPoints3dMsg.empty())
    {
      // Points should be in base frame
      points = rtabmap_ros::points3fFromROS(localPoints3dMsg, stereoModel.localTransform());
    }
    if (!keypoints.empty())
    {
      UASSERT(points.empty() || points.size() == keypoints.size());
      UASSERT(localDescriptorsMsg.empty() || localDescriptorsMsg.rows == (int)keypoints.size());
      data.setFeatures(keypoints, points, localDescriptorsMsg);
    }

    process(lastPoseStamp_,
            data,
            lastPose_,
            odomFrameId,
            covariance_,
            odomInfo,
            timerConversion.ticks());

    covariance_ = cv::Mat();
  }

  void CoreWrapper::commonLaserScanCallback(
      const nav_msgs::OdometryConstPtr &odomMsg,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const sensor_msgs::LaserScan &scan2dMsg,
      const sensor_msgs::PointCloud2 &scan3dMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg,
      const rtabmap_ros::GlobalDescriptor &globalDescriptor)
  {
    UTimer timerConversion;
    std::string odomFrameId = odomFrameId_;
    if (odomMsg.get())
    {
      odomFrameId = odomMsg->header.frame_id;
      if (!scan2dMsg.ranges.empty())
      {
        if (!odomUpdate(odomMsg, scan2dMsg.header.stamp))
        {
          return;
        }
      }
      else if (!scan3dMsg.data.empty())
      {
        if (!odomUpdate(odomMsg, scan3dMsg.header.stamp))
        {
          return;
        }
      }
      else
      {
        return;
      }
    }
    else if (!scan2dMsg.ranges.empty())
    {
      if (!odomTFUpdate(scan2dMsg.header.stamp))
      {
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!odomTFUpdate(scan3dMsg.header.stamp))
      {
        return;
      }
    }
    else
    {
      return;
    }

    LaserScan scan;
    if (!scan2dMsg.ranges.empty())
    {
      if (!rtabmap_ros::convertScanMsg(
              scan2dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              // backward compatibility, project 2D scan in /base_link frame
              rtabmap_.getMemory() && uStrNumCmp(rtabmap_.getMemory()->getDatabaseVersion(), "0.11.10") < 0))
      {
        NODELET_ERROR("Could not convert laser scan msg! Aborting rtabmap update...");
        return;
      }
    }
    else if (!scan3dMsg.data.empty())
    {
      if (!rtabmap_ros::convertScan3dMsg(
              scan3dMsg,
              frameId_,
              odomSensorSync_ ? odomFrameId : "",
              lastPoseStamp_,
              scan,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0,
              scanCloudMaxPoints_))
      {
        NODELET_ERROR("Could not convert 3d laser scan msg! Aborting rtabmap update...");
        return;
      }
    }

    cv::Mat userData;
    if (userDataMsg.get())
    {
      userData = rtabmap_ros::userDataFromROS(*userDataMsg);
      UScopeMutex lock(userDataMutex_);
      if (!userData_.empty())
      {
        NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
        userData_ = cv::Mat();
      }
    }
    else
    {
      UScopeMutex lock(userDataMutex_);
      userData = userData_;
      userData_ = cv::Mat();
    }

    SensorData data(
        scan,
        cv::Mat(),
        cv::Mat(),
        CameraModel(),
        lastPoseIntermediate_ ? -1 : !scan2dMsg.ranges.empty() ? scan2dMsg.header.seq
                                                               : scan3dMsg.header.seq,
        rtabmap_ros::timestampFromROS(lastPoseStamp_),
        userData);

    OdometryInfo odomInfo;
    if (odomInfoMsg.get())
    {
      odomInfo = odomInfoFromROS(*odomInfoMsg);
    }

    if (!globalDescriptor.data.empty())
    {
      data.addGlobalDescriptor(rtabmap_ros::globalDescriptorFromROS(globalDescriptor));
    }

    process(lastPoseStamp_,
            data,
            lastPose_,
            odomFrameId,
            covariance_,
            odomInfo,
            timerConversion.ticks());

    covariance_ = cv::Mat();
  }

  void CoreWrapper::commonOdomCallback(
      const nav_msgs::OdometryConstPtr &odomMsg,
      const rtabmap_ros::UserDataConstPtr &userDataMsg,
      const rtabmap_ros::OdomInfoConstPtr &odomInfoMsg)
  {
    UTimer timerConversion;
    UASSERT(odomMsg.get());
    std::string odomFrameId = odomFrameId_;

    odomFrameId = odomMsg->header.frame_id;
    if (!odomUpdate(odomMsg, odomMsg->header.stamp))
    {
      return;
    }

    cv::Mat userData;
    if (userDataMsg.get())
    {
      userData = rtabmap_ros::userDataFromROS(*userDataMsg);
      UScopeMutex lock(userDataMutex_);
      if (!userData_.empty())
      {
        NODELET_WARN("Synchronized and asynchronized user data topics cannot be used at the same time. Async user data dropped!");
        userData_ = cv::Mat();
      }
    }
    else
    {
      UScopeMutex lock(userDataMutex_);
      userData = userData_;
      userData_ = cv::Mat();
    }

    SensorData data(
        cv::Mat(),
        cv::Mat(),
        CameraModel(),
        lastPoseIntermediate_ ? -1 : odomMsg->header.seq,
        rtabmap_ros::timestampFromROS(lastPoseStamp_),
        userData);

    OdometryInfo odomInfo;
    if (odomInfoMsg.get())
    {
      odomInfo = odomInfoFromROS(*odomInfoMsg);
    }

    process(lastPoseStamp_,
            data,
            lastPose_,
            odomFrameId,
            covariance_,
            odomInfo,
            timerConversion.ticks());

    covariance_ = cv::Mat();
  }

  void CoreWrapper::process(
      const ros::Time &stamp,
      SensorData &data,
      const Transform &odom,
      const std::string &odomFrameId,
      const cv::Mat &odomCovariance,
      const OdometryInfo &odomInfo,
      double timeMsgConversion)
  {
    UTimer timer;
    if (rtabmap_.isIDsGenerated() || data.id() > 0)
    {
      // Add intermediate nodes?
      for (std::list<std::pair<nav_msgs::Odometry, rtabmap_ros::OdomInfo>>::iterator iter = interOdoms_.begin(); iter != interOdoms_.end();)
      {
        if (iter->first.header.stamp < lastPoseStamp_)
        {
          Transform interOdom;
          if (!rtabmap_.getLocalOptimizedPoses().empty())
          {
            // add intermediate poses only if the current local graph is not empty
            interOdom = rtabmap_ros::transformFromPoseMsg(iter->first.pose.pose);
          }
          if (!interOdom.isNull())
          {
            cv::Mat covariance;
            double variance = iter->first.twist.covariance[0];
            if (variance == BAD_COVARIANCE || variance <= 0.0f)
            {
              // use the one of the pose
              covariance = cv::Mat(6, 6, CV_64FC1, (void *)iter->first.pose.covariance.data()).clone();
              covariance /= 2.0;
            }
            else
            {
              covariance = cv::Mat(6, 6, CV_64FC1, (void *)iter->first.twist.covariance.data()).clone();
            }
            if (!uIsFinite(covariance.at<double>(0, 0)) || covariance.at<double>(0, 0) <= 0.0f)
            {
              covariance = cv::Mat::eye(6, 6, CV_64FC1);
              if (odomDefaultLinVariance_ > 0.0f)
              {
                covariance.at<double>(0, 0) = odomDefaultLinVariance_;
                covariance.at<double>(1, 1) = odomDefaultLinVariance_;
                covariance.at<double>(2, 2) = odomDefaultLinVariance_;
              }
              if (odomDefaultAngVariance_ > 0.0f)
              {
                covariance.at<double>(3, 3) = odomDefaultAngVariance_;
                covariance.at<double>(4, 4) = odomDefaultAngVariance_;
                covariance.at<double>(5, 5) = odomDefaultAngVariance_;
              }
            }
            else if (twoDMapping_)
            {
              // If 2d mapping, make sure all diagonal values of the covariance that even not used are not null.
              covariance.at<double>(2, 2) = uIsFinite(covariance.at<double>(2, 2)) && covariance.at<double>(2, 2) != 0 ? covariance.at<double>(2, 2) : 1;
              covariance.at<double>(3, 3) = uIsFinite(covariance.at<double>(3, 3)) && covariance.at<double>(3, 3) != 0 ? covariance.at<double>(3, 3) : 1;
              covariance.at<double>(4, 4) = uIsFinite(covariance.at<double>(4, 4)) && covariance.at<double>(4, 4) != 0 ? covariance.at<double>(4, 4) : 1;
            }

            SensorData interData(cv::Mat(), cv::Mat(), CameraModel(), -1, rtabmap_ros::timestampFromROS(iter->first.header.stamp));
            Transform gt;
            if (!groundTruthFrameId_.empty())
            {
              gt = rtabmap_ros::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, iter->first.header.stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
            }
            interData.setGroundTruth(gt);

            std::map<std::string, float> externalStats;
            std::vector<float> odomVelocity;
            if (iter->second.timeEstimation != 0.0f)
            {
              OdometryInfo info = odomInfoFromROS(iter->second, true);
              externalStats = rtabmap_ros::odomInfoToStatistics(info);

              if (info.interval > 0.0)
              {
                odomVelocity.resize(6);
                float x, y, z, roll, pitch, yaw;
                info.transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
                odomVelocity[0] = x / info.interval;
                odomVelocity[1] = y / info.interval;
                odomVelocity[2] = z / info.interval;
                odomVelocity[3] = roll / info.interval;
                odomVelocity[4] = pitch / info.interval;
                odomVelocity[5] = yaw / info.interval;
              }
            }
            rtabmap_mtx_.lock();
            rtabmap_.process(interData, interOdom, covariance, odomVelocity, externalStats);
            rtabmap_mtx_.unlock();
          }
          interOdoms_.erase(iter++);
        }
        else if (iter->first.header.stamp == lastPoseStamp_)
        {
          interOdoms_.erase(iter++);
          break;
        }
        else
        {
          break;
        }
      }
      // --- IGNORE THIS ---

      // Add async stuff
      Transform groundTruthPose;
      if (!groundTruthFrameId_.empty())
      {
        groundTruthPose = rtabmap_ros::getTransform(groundTruthFrameId_, groundTruthBaseFrameId_, lastPoseStamp_, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
      }
      data.setGroundTruth(groundTruthPose);

      // global pose
      if (!globalPose_.header.stamp.isZero())
      {
        // assume sensor is fixed
        Transform sensorToBase = rtabmap_ros::getTransform(
            globalPose_.header.frame_id,
            frameId_,
            lastPoseStamp_,
            tfListener_,
            waitForTransform_ ? waitForTransformDuration_ : 0.0);
        if (!sensorToBase.isNull())
        {
          Transform globalPose = rtabmap_ros::transformFromPoseMsg(globalPose_.pose.pose);
          globalPose *= sensorToBase; // transform global pose from sensor frame to robot base frame

          // Correction of the global pose accounting the odometry movement since we received it
          Transform correction = rtabmap_ros::getTransform(
              frameId_,
              odomFrameId,
              globalPose_.header.stamp,
              lastPoseStamp_,
              tfListener_,
              waitForTransform_ ? waitForTransformDuration_ : 0.0);
          if (!correction.isNull())
          {
            globalPose *= correction;
          }
          else
          {
            NODELET_WARN("Could not adjust global pose accordingly to latest odometry pose. "
                         "If odometry is small since it received the global pose and "
                         "covariance is large, this should not be a problem.");
          }
          cv::Mat globalPoseCovariance = cv::Mat(6, 6, CV_64FC1, (void *)globalPose_.pose.covariance.data()).clone();
          data.setGlobalPose(globalPose, globalPoseCovariance);
        }
      }
      globalPose_.header.stamp = ros::Time(0);

      if (gps_.stamp() > 0.0)
      {
        data.setGPS(gps_);
      }
      gps_ = rtabmap::GPS();

      // tag detections
      rtabmap::Landmarks landmarks = rtabmap_ros::landmarksFromROS(
          tags_,
          frameId_,
          odomFrameId,
          lastPoseStamp_,
          tfListener_,
          waitForTransform_ ? waitForTransformDuration_ : 0,
          landmarkDefaultLinVariance_,
          landmarkDefaultAngVariance_);
      tags_.clear();
      if (!landmarks.empty())
      {
        data.setLandmarks(landmarks);
      }

      // IMU
      if (!imus_.empty())
      {
        Transform t = Transform::getTransform(imus_, data.stamp());
        if (!t.isNull())
        {
          // get local transform
          rtabmap::Transform localTransform;
          if (frameId_.compare(imuFrameId_) != 0)
          {
            localTransform = getTransform(frameId_, imuFrameId_, ros::Time(data.stamp()), tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
          }
          else
          {
            localTransform = rtabmap::Transform::getIdentity();
          }

          if (!localTransform.isNull())
          {
            Eigen::Quaterniond q = t.getQuaterniond();
            data.setIMU(IMU(cv::Vec4d(q.x(), q.y(), q.z(), q.w()), cv::Mat::eye(3, 3, CV_64FC1),
                            cv::Vec3d(), cv::Mat(),
                            cv::Vec3d(), cv::Mat(),
                            localTransform));
          }
        }
        else
        {
          ROS_WARN("We are receiving imu data (buffer=%d), but cannot interpolate "
                   "imu transform at time %f. IMU won't be added to graph.",
                   (int)imus_.size(), data.stamp());
        }
      }

      double timeRtabmap = 0.0;
      double timeUpdateMaps = 0.0;
      // double timePublishMaps = 0.0;

      cv::Mat covariance = odomCovariance;
      if (covariance.empty() || !uIsFinite(covariance.at<double>(0, 0)) || covariance.at<double>(0, 0) <= 0.0f)
      {
        covariance = cv::Mat::eye(6, 6, CV_64FC1);
        if (odomDefaultLinVariance_ > 0.0f)
        {
          covariance.at<double>(0, 0) = odomDefaultLinVariance_;
          covariance.at<double>(1, 1) = odomDefaultLinVariance_;
          covariance.at<double>(2, 2) = odomDefaultLinVariance_;
        }
        if (odomDefaultAngVariance_ > 0.0f)
        {
          covariance.at<double>(3, 3) = odomDefaultAngVariance_;
          covariance.at<double>(4, 4) = odomDefaultAngVariance_;
          covariance.at<double>(5, 5) = odomDefaultAngVariance_;
        }
      }
      else if (twoDMapping_)
      {
        // If 2d mapping, make sure all diagonal values of the covariance that even not used are not null.
        covariance.at<double>(2, 2) = uIsFinite(covariance.at<double>(2, 2)) && covariance.at<double>(2, 2) != 0 ? covariance.at<double>(2, 2) : 1;
        covariance.at<double>(3, 3) = uIsFinite(covariance.at<double>(3, 3)) && covariance.at<double>(3, 3) != 0 ? covariance.at<double>(3, 3) : 1;
        covariance.at<double>(4, 4) = uIsFinite(covariance.at<double>(4, 4)) && covariance.at<double>(4, 4) != 0 ? covariance.at<double>(4, 4) : 1;
      }

      std::map<std::string, float> externalStats;
      std::vector<float> odomVelocity;
      if (odomInfo.timeEstimation != 0.0f)
      {
        externalStats = rtabmap_ros::odomInfoToStatistics(odomInfo);

        if (odomInfo.interval > 0.0)
        {
          odomVelocity.resize(6);
          float x, y, z, roll, pitch, yaw;
          odomInfo.transform.getTranslationAndEulerAngles(x, y, z, roll, pitch, yaw);
          odomVelocity[0] = x / odomInfo.interval;
          odomVelocity[1] = y / odomInfo.interval;
          odomVelocity[2] = z / odomInfo.interval;
          odomVelocity[3] = roll / odomInfo.interval;
          odomVelocity[4] = pitch / odomInfo.interval;
          odomVelocity[5] = yaw / odomInfo.interval;
        }
      }
      if (rtabmapROSStats_.size())
      {
        externalStats.insert(rtabmapROSStats_.begin(), rtabmapROSStats_.end());
        rtabmapROSStats_.clear();
      }

      // JHUAPL Section

      // if the map manager has not initialized pre processing strategies based on data from
      // inputs. It would only take place once.
      if (!mapManagerInitialized_)
      {
        // When Octomap's Ray Tracing is Enabled, it needs to initialized a reference boundary cloud.
        if (mapsManager_.isOctomapRayTracingEnabled())
        {
          // ray tracing needs camera instrinsic params and image size.
          mapManagerInitialized_ = mapsManager_.octomapRayTracingInit(data);
        }
        else 
        {
          // it needs tobe set to true for the map manager thread to start.
          mapManagerInitialized_ = true;
        }
      }

      // JHUAPL end of Section

      if (threadedMode_)
      {
        odomFrameId_ = odomFrameId;
        timeMsgConversion += timer.ticks();

        // getting the time of over head
        double overheadTimeCallback = 0;
        if (timeInputLastProcess_ > 0)
        {
          overheadTimeCallback = ros::Time::now().toSec() - timeInputLastProcess_;
          // removing the time msg conversion from the overhead.
          overheadTimeCallback = overheadTimeCallback - timeMsgConversion;
        }

        boost::shared_ptr<PreProcessSignatureDataType> preprocessedData;
        bool preprocess_ok = rtabmap_.preprocess_threaded(stamp.toSec(), data, odom, odomFrameId, covariance, odomVelocity, externalStats, preprocessedData);

        if (preprocess_ok)
        {
          if (data.id() < 0)
          {
            NODELET_INFO("Intermediate node added");
          }
          else
          {
            // JHUAPL section

            // Publish Pre-Processed Senosr Data
            // subscribing to this slows down the registration rate.
            if (rtabmap_.getMemory()->getOccupancyGrid()->isEnableSemanticSegmentation())
            {
              // This is the data struture with all data process in the preprocess functions
              //   the last process data.
              rtabmap::SensorData &sd = *preprocessedData->data;

              // one time update to set the transformation transforming points from camera to base frame.
              camTobaseT_mtx_.lock();
              if (camTobaseTransform_.isIdentity() || camTobaseTransform_.isNull())
              {
                camTobaseTransform_ = sd.cameraModels().at(0).localTransform();
              }
              camTobaseT_mtx_.unlock();
              
              // publish the newest semantic mask added to map
              mapsManager_.publishSemenaticMaskImage(sd);

              // publish visual and depth image
              publishVisualDepthImages(sd);

              // publish obstacles data of the last added Data
              publishObstaclesData(sd);
            }
            else
            {
              // RGBD mode

              // this is the same as the "data" sent to rtabmap except that it now
              // contains local occupancy grid information and post-processing
              // (e.g., decimation) of the sensor data
              // rtabmap::SensorData sd = rtabmap_.getLastAddedData();

              rtabmap::SensorData &sd = *preprocessedData->data;
              // rtabmap::SensorData sd = rtabmap_.getLastAddedData();

              // one time update to set the transformation transforming points from camera to base frame.
              camTobaseT_mtx_.lock();
              if (camTobaseTransform_.isIdentity() || camTobaseTransform_.isNull())
              {
                camTobaseTransform_ = sd.cameraModels().at(0).localTransform();
              }
              camTobaseT_mtx_.unlock();

              // publish visual and depth image
              publishVisualDepthImages(sd);
            }

            // JHUAPL section end
          }
        }
        timeRtabmap = timer.ticks();
        float total_processing_time = timeMsgConversion + timeRtabmap;

        NODELET_INFO("  rtabmap (input callback): target Rate=%.2fs, RTAB-Map conversion:%.4fs, processing=%.4fs (total=%.4fsec)",
                     rate_ > 0 ? 1 / rate_ : 0,
                     timeMsgConversion,
                     timeRtabmap,
                     total_processing_time);

        // stats message
        rtabmap_ros::InputDataStats msg;
        msg.header.stamp = ros::Time::now();
        msg.time_elapsed_since_last_called = overheadTimeCallback;
        msg.total_elapsed_time = total_processing_time;

        // publish stats when there are subscribers.
        if (inputProcessThreadStatsPub_.getNumSubscribers())
        {
          inputProcessThreadStatsPub_.publish(msg);
        }

        timeInputLastProcess_ = ros::Time::now().toSec();
      }
      else
      {
        // mutex to sync with map manager thread
        rtabmap_mtx_.lock();
        timeMsgConversion += timer.ticks();
        bool rtabmapProcessed = rtabmap_.process(data, odom, covariance, odomVelocity, externalStats, signature_complete_callback, this);
        rtabmap_mtx_.unlock();

        if (rtabmapProcessed)
        {
          timeRtabmap = timer.ticks();
          mapToOdomMutex_.lock();
          // The author of rtabmap is not consistent in his naming convention for transformation variables;
          //  most of the time, AtoB means the transformation that transforms points from Frame A to Frame B;
          //  however, for mapToOdom_ this naming convention is flipped
          //
          // mapToOdom_ : actually transforms points from Odom frame to Map frame
          //
          mapToOdom_ = rtabmap_.getMapCorrection();
          odomFrameId_ = odomFrameId;
          mapToOdomMutex_.unlock();

          rtabmap::Transform baseToMap = rtabmap::Transform::getIdentity();

          if (data.id() < 0)
          {
            NODELET_INFO("Intermediate node added");
          }
          else
          {
            // JHUAPL section

            // subscribing to this slows down the registration rate.
            if (rtabmap_.getMemory()->getOccupancyGrid()->isEnableSemanticSegmentation())
            {
              // this is the same as the "data" sent to rtabmap except that it now
              // contains local occupancy grid information and post-processing
              // (e.g., decimation) of the sensor data
              rtabmap::SensorData sd = rtabmap_.getMemory()->getLastAddedData();

              // one time update to set the transformation transforming points from camera to base frame.
              camTobaseT_mtx_.lock();
              if (camTobaseTransform_.isIdentity() || camTobaseTransform_.isNull())
              {
                camTobaseTransform_ = sd.cameraModels().at(0).localTransform();
              }
              camTobaseT_mtx_.unlock();

              // publish the newest semantic mask added to map
              mapsManager_.publishSemenaticMaskImage(sd);

              // publish visual and depth image
              publishVisualDepthImages(sd);

              // moved this to callback to publish sooner
              // publish obstacles data of the last added Data
              // publishObstaclesData(sd);
            }
            else
            {
              // RGBD mode

              // this is the same as the "data" sent to rtabmap except that it now
              // contains local occupancy grid information and post-processing
              // (e.g., decimation) of the sensor data
              rtabmap::SensorData sd = rtabmap_.getMemory()->getLastAddedData();

              // one time update to set the transformation transforming points from camera to base frame.
              camTobaseT_mtx_.lock();
              if (camTobaseTransform_.isIdentity() || camTobaseTransform_.isNull())
              {
                camTobaseTransform_ = sd.cameraModels().at(0).localTransform();
              }
              camTobaseT_mtx_.unlock();

              // publish visual and depth image
              publishVisualDepthImages(sd);
            }

            // JHUAPL section end

            // Publish local graph, info
            this->publishStats(stamp);
            // JHUAPL modification: pubs pose on every iteration
            // publish localization with correction regardless whether covariance is available
            if (localizationPosePub_.getNumSubscribers())
            {
              geometry_msgs::PoseWithCovarianceStamped poseMsg;
              poseMsg.header.frame_id = mapFrameId_;
              poseMsg.header.stamp = stamp;

              poseMsg.pose.covariance;
              if (!rtabmap_.getStatistics().localizationCovariance().empty())
              {
                // The author of rtabmap is not consistent in his naming convention for transformation variables;
                //  most of the time, AtoB means the transformation that transforms points from Frame A to Frame B;
                //  however, for mapToOdom_ this naming convention is flipped
                //
                //  mapToOdom_ : actually transforms points from Odom frame to Map frame
                //
                //  odom        : T_odom_base : transforms pt from base to odom frame
                //  mapToOdom_  : T_map_odom  : transforms pt from odom frame to map frame [NAMING CONVENTION REVERSED FOR "mapToOdom_"]
                //  baseToMap   : T_map_base  : transforms pt from base to map frame
                baseToMap = mapToOdom_ * odom;
                rtabmap_ros::transformToPoseMsg(baseToMap, poseMsg.pose.pose);
                const cv::Mat &cov = rtabmap_.getStatistics().localizationCovariance();
                memcpy(poseMsg.pose.covariance.data(), cov.data, cov.total() * sizeof(double));
                mapToOdomPrev_ = mapToOdom_;
              }
              else
              {
                // NOTE: due to mapToOdom_ naming convention being flipped, mapToOdomPrev_ also uses
                //       a reverse naming convention, i.e.
                //       mapToOdomPrev_  transforms point from odom frame to map frame
                //
                baseToMap = mapToOdomPrev_ * odom;
                rtabmap_ros::transformToPoseMsg(baseToMap, poseMsg.pose.pose);
              }
              localizationPosePub_.publish(poseMsg);
            }
            std::map<int, rtabmap::Transform> filteredPoses(rtabmap_.getLocalOptimizedPoses().lower_bound(1), rtabmap_.getLocalOptimizedPoses().end());

            // create a tmp signature with latest sensory data if latest signature was ignored
            std::map<int, rtabmap::Signature> tmpSignature;
            if (rtabmap_.getMemory() == 0)
            {
              UDEBUG(" --> tmp signature mem == 0; latest data from input");
              SensorData tmpData = data;
              tmpData.setId(0);
              tmpSignature.insert(std::make_pair(0, Signature(0, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
              filteredPoses.insert(std::make_pair(0, mapToOdom_ * odom));
            }
            /// JHUAPL section
            else if (filteredPoses.size() == 0 ||
                     rtabmap_.getMemory()->getLastSignatureId() != filteredPoses.rbegin()->first ||
                     rtabmap_.getMemory()->getLastWorkingSignature() == 0 ||
                     rtabmap_.getMemory()->getLastWorkingSignature()->sensorData().gridCellSize() == 0 ||
                     mapsManager_.getIsAlwaysUpdateMap() ||
                     (!mapsManager_.getOccupancyGrid()->isGridFromDepth() && data.laserScanRaw().is2d())) // 2d laser scan would fill empty space for latest data
            {
              UDEBUG(" ----------> incoming node id(input cloud): %d (now id zero)", rtabmap_.getMemory()->getLastAddedData().id());
              SensorData tmpData = rtabmap_.getMemory()->getLastAddedData();
              tmpData.setId(0);
              tmpData.setStamp(data.stamp());
              tmpSignature.insert(std::make_pair(0, Signature(0, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
              filteredPoses.insert(std::make_pair(0, mapToOdom_ * odom));
            }
            /// JHUAPL section end

            if ((mappingMaxNodes_ > 0 || mappingAltitudeDelta_ > 0.0) && filteredPoses.size() > 1)
            {
              std::map<int, Transform> nearestPoses;
              nearestPoses = filterNodesToAssemble(filteredPoses, mapToOdom_ * odom);

              // add latest/zero and make sure those on a planned path are not filtered
              std::set<int> onPath;
              if (rtabmap_.getPath().size())
              {
                std::vector<int> nextNodes = rtabmap_.getPathNextNodes();
                onPath.insert(nextNodes.begin(), nextNodes.end());
              }
              for (std::map<int, Transform>::iterator iter = filteredPoses.begin(); iter != filteredPoses.end(); ++iter)
              {
                if (iter->first == 0 || onPath.find(iter->first) != onPath.end())
                {
                  nearestPoses.insert(*iter);
                }
                else if (onPath.empty())
                {
                  break;
                }
              }

              filteredPoses = nearestPoses;
            }

            mmu_data_mtx_.lock();
            filteredPoses_ = filteredPoses;
            tmpSignature_ = tmpSignature;
            stamp_ = stamp;
            // last_baseToMap_ : last T_map_base : transform pt from base to map frame
            last_baseToMap_ = baseToMap;
            mmu_data_mtx_.unlock();

            this->publishLandmarksMap(rtabmap_.getMemory(), mapFrameId_);

            timeUpdateMaps = timer.ticks();

            // Update maps
            // filteredPoses = mapsManager_.updateMapCaches(
            // 		filteredPoses,
            // 		rtabmap_.getMemory(),
            // 		false,
            // 		false,
            // 		tmpSignature);

            // timeUpdateMaps = timer.ticks();

            // UDEBUG("filteredPoses map size: (%d)", filteredPoses.size());

            // update goal if planning is enabled
            // if(!currentMetricGoal_.isNull())
            // {
            // 	if(rtabmap_.getPath().size() == 0)
            // 	{
            // 		// Don't send status yet if move_base actionlib is used unless it failed,
            // 		// let move_base finish reaching the goal
            // 		if(mbClient_ == 0 || rtabmap_.getPathStatus() <= 0)
            // 		{
            // 			if(rtabmap_.getPathStatus() > 0)
            // 			{
            // 				// Goal reached
            // 				NODELET_INFO("Planning: Publishing goal reached!");
            // 			}
            // 			else if(rtabmap_.getPathStatus() <= 0)
            // 			{
            // 				NODELET_WARN("Planning: Plan failed!");
            // 				if(mbClient_ && mbClient_->isServerConnected())
            // 				{
            // 					mbClient_->cancelGoal();
            // 				}
            // 			}

            // 		if(goalReachedPub_.getNumSubscribers())
            // 		{
            // 			std_msgs::Bool result;
            // 			result.data = rtabmap_.getPathStatus() > 0;
            // 			goalReachedPub_.publish(result);
            // 		}
            // 		currentMetricGoal_.setNull();
            // 		lastPublishedMetricGoal_.setNull();
            // 		goalFrameId_.clear();
            // 		latestNodeWasReached_ = false;
            // 	}
            // }
            // else
            // {
            // 	currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
            // 	if(!currentMetricGoal_.isNull())
            // 	{
            // 		// Adjust the target pose relative to last node
            // 		if(rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
            // 		{
            // 			if(latestNodeWasReached_ ||
            // 			   rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
            // 			{
            // 				latestNodeWasReached_ = true;
            // 				Transform goalLocalTransform = Transform::getIdentity();
            // 				if(!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
            // 				{
            // 					Transform localT = rtabmap_ros::getTransform(frameId_, goalFrameId_, ros::Time::now(), tfListener_, waitForTransform_?waitForTransformDuration_:0.0);
            // 					if(!localT.isNull())
            // 					{
            // 						goalLocalTransform = localT.inverse().to3DoF();
            // 					}
            // 				}
            // 				currentMetricGoal_ *= rtabmap_.getPathTransformToGoal()*goalLocalTransform;
            // 			}
            // 		}

            // 			// publish next goal with updated currentMetricGoal_
            // 			publishCurrentGoal(stamp);

            // 			// publish local path
            // 			publishLocalPath(stamp);

            // 			// publish global path
            // 			publishGlobalPath(stamp);
            // 		}
            // 		else
            // 		{
            // 			NODELET_ERROR("Planning: Local map broken, current goal id=%d (the robot may have moved to far from planned nodes)",
            // 					rtabmap_.getPathCurrentGoalId());
            // 			rtabmap_.clearPath(-1);
            // 			if(goalReachedPub_.getNumSubscribers())
            // 			{
            // 				std_msgs::Bool result;
            // 				result.data = false;
            // 				goalReachedPub_.publish(result);
            // 			}
            // 			currentMetricGoal_.setNull();
            // 			lastPublishedMetricGoal_.setNull();
            // 			goalFrameId_.clear();
            // 			latestNodeWasReached_ = false;
            // 		}
            // 	}
            // }

            // timePublishMaps = timer.ticks();
          }
        }
        else
        {
          timeRtabmap = timer.ticks();
        }
        NODELET_INFO("rtabmap (%d): Rate=%.2fs, Limit=%.3fs, Conversion=%.4fs, RTAB-Map=%.4fs, pub landmarks=%.4fs total=%.4fs (local map=%d, WM=%d)",
                     rtabmap_.getLastLocationId(),
                     rate_ > 0 ? 1.0f / rate_ : 0,
                     rtabmap_.getTimeThreshold() / 1000.0f,
                     timeMsgConversion,
                     timeRtabmap,
                     timeUpdateMaps,
                     timeRtabmap + timeUpdateMaps,
                     (int)rtabmap_.getLocalOptimizedPoses().size(),
                     rtabmap_.getWMSize() + rtabmap_.getSTMSize());
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/HasSubscribers/"), mapsManager_.hasSubscribers() ? 1 : 0));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeMsgConversion/ms"), timeMsgConversion * 1000.0f));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeRtabmap/ms"), timeRtabmap * 1000.0f));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeUpdatingMaps/ms"), timeUpdateMaps * 1000.0f));
        // rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimePublishing/ms"), timePublishMaps*1000.0f));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeTotal/ms"), (timeRtabmap + timeUpdateMaps) * 1000.0f));
      }
    }
    else if (!rtabmap_.isIDsGenerated())
    {
      NODELET_WARN("Ignoring received image because its sequence ID=0. Please "
                   "set \"Mem/GenerateIds\"=\"true\" to ignore ros generated sequence id. "
                   "Use only \"Mem/GenerateIds\"=\"false\" for once-time run of RTAB-Map and "
                   "when you need to have IDs output of RTAB-map synchronized with the source "
                   "image sequence ID.");
    }
  }

  std::map<int, Transform> CoreWrapper::filterNodesToAssemble(
      const std::map<int, Transform> &nodes,
      const Transform &currentPose)
  {
    std::map<int, Transform> output;
    if (mappingMaxNodes_ > 0)
    {
      std::map<int, float> nodesDist = graph::findNearestNodes(currentPose, nodes, 0, 0, mappingMaxNodes_);
      for (std::map<int, float>::iterator iter = nodesDist.begin(); iter != nodesDist.end(); ++iter)
      {
        if (mappingAltitudeDelta_ <= 0.0 ||
            fabs(nodes.at(iter->first).z() - currentPose.z()) < mappingAltitudeDelta_)
        {
          output.insert(*nodes.find(iter->first));
        }
      }
    }
    else // mappingAltitudeDelta_>0.0
    {
      for (std::map<int, Transform>::const_iterator iter = nodes.begin(); iter != nodes.end(); ++iter)
      {
        if (fabs(iter->second.z() - currentPose.z()) < mappingAltitudeDelta_)
        {
          output.insert(*iter);
        }
      }
    }
    return output;
  }

  void CoreWrapper::userDataAsyncCallback(const rtabmap_ros::UserDataConstPtr &dataMsg)
  {
    if (!paused_)
    {
      UScopeMutex lock(userDataMutex_);
      static bool warningShow = false;
      if (!userData_.empty() && !warningShow)
      {
        ROS_WARN("Overwriting previous user data set. When asynchronous user "
                 "data input topic rate is higher than "
                 "map update rate (current %s=%f), only latest data is saved "
                 "in the next node created. This message will is shown only once.",
                 Parameters::kRtabmapDetectionRate().c_str(), rate_);
        warningShow = true;
      }
      userData_ = rtabmap_ros::userDataFromROS(*dataMsg);
    }
  }

  void CoreWrapper::globalPoseAsyncCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &globalPoseMsg)
  {
    if (!paused_)
    {
      globalPose_ = *globalPoseMsg;
    }
  }

  void CoreWrapper::gpsFixAsyncCallback(const sensor_msgs::NavSatFixConstPtr &gpsFixMsg)
  {
    if (!paused_)
    {
      double error = 10.0;
      if (gpsFixMsg->position_covariance_type != sensor_msgs::NavSatFix::COVARIANCE_TYPE_UNKNOWN)
      {
        double variance = uMax3(gpsFixMsg->position_covariance.at(0), gpsFixMsg->position_covariance.at(4), gpsFixMsg->position_covariance.at(8));
        if (variance > 0.0)
        {
          error = sqrt(variance);
        }
      }
      gps_ = rtabmap::GPS(
          gpsFixMsg->header.stamp.toSec(),
          gpsFixMsg->longitude,
          gpsFixMsg->latitude,
          gpsFixMsg->altitude,
          error,
          0);
    }
  }

#ifdef WITH_APRILTAG_ROS
  void CoreWrapper::tagDetectionsAsyncCallback(const apriltag_ros::AprilTagDetectionArray &tagDetections)
  {
    if (!paused_)
    {
      for (unsigned int i = 0; i < tagDetections.detections.size(); ++i)
      {
        if (tagDetections.detections[i].id.size() >= 1)
        {
          geometry_msgs::PoseWithCovarianceStamped p = tagDetections.detections[i].pose;
          p.header = tagDetections.header;
          if (!tagDetections.detections[i].pose.header.frame_id.empty())
          {
            p.header.frame_id = tagDetections.detections[i].pose.header.frame_id;

            static bool warned = false;
            if (!warned &&
                !tagDetections.header.frame_id.empty() &&
                tagDetections.detections[i].pose.header.frame_id.compare(tagDetections.header.frame_id) != 0)
            {
              NODELET_WARN("frame_id set for individual tag detections (%s) doesn't match the frame_id of the message (%s), "
                           "the resulting pose of the tag may be wrong. This message is only printed once.",
                           tagDetections.detections[i].pose.header.frame_id.c_str(), tagDetections.header.frame_id.c_str());
              warned = true;
            }
          }
          if (!tagDetections.detections[i].pose.header.stamp.isZero())
          {
            p.header.stamp = tagDetections.detections[i].pose.header.stamp;

            static bool warned = false;
            if (!warned &&
                !tagDetections.header.stamp.isZero() &&
                tagDetections.detections[i].pose.header.stamp != tagDetections.header.stamp)
            {
              NODELET_WARN("stamp set for individual tag detections (%f) doesn't match the stamp of the message (%f), "
                           "the resulting pose of the tag may be wrongly interpolated. This message is only printed once.",
                           tagDetections.detections[i].pose.header.stamp.toSec(), tagDetections.header.stamp.toSec());
              warned = true;
            }
          }
          uInsert(tags_,
                  std::make_pair(tagDetections.detections[i].id[0],
                                 std::make_pair(p, tagDetections.detections[i].size.size() == 1 ? (float)tagDetections.detections[i].size[0] : 0.0f)));
        }
      }
    }
  }
#endif

#ifdef WITH_FIDUCIAL_MSGS
  void CoreWrapper::fiducialDetectionsAsyncCallback(const fiducial_msgs::FiducialTransformArray &fiducialDetections)
  {
    if (!paused_)
    {
      for (unsigned int i = 0; i < fiducialDetections.transforms.size(); ++i)
      {
        geometry_msgs::PoseWithCovarianceStamped p;
        p.pose.pose.orientation = fiducialDetections.transforms[i].transform.rotation;
        p.pose.pose.position.x = fiducialDetections.transforms[i].transform.translation.x;
        p.pose.pose.position.y = fiducialDetections.transforms[i].transform.translation.y;
        p.pose.pose.position.z = fiducialDetections.transforms[i].transform.translation.z;
        p.header = fiducialDetections.header;
        uInsert(tags_,
                std::make_pair(fiducialDetections.transforms[i].fiducial_id,
                               std::make_pair(p, 0.0f)));
      }
    }
  }
#endif

  void CoreWrapper::imuAsyncCallback(const sensor_msgs::ImuConstPtr &msg)
  {
    if (!paused_)
    {
      if (msg->orientation.x == 0 && msg->orientation.y == 0 && msg->orientation.z == 0 && msg->orientation.w == 0)
      {
        UERROR("IMU received doesn't have orientation set, it is ignored.");
      }
      else
      {
        Transform orientation(0, 0, 0, msg->orientation.x, msg->orientation.y, msg->orientation.z, msg->orientation.w);
        imus_.insert(std::make_pair(msg->header.stamp.toSec(), orientation));
        if (imus_.size() > 1000)
        {
          imus_.erase(imus_.begin());
        }
        if (!imuFrameId_.empty() && imuFrameId_.compare(msg->header.frame_id) != 0)
        {
          ROS_ERROR("IMU frame_id has changed from %s to %s! Are "
                    "multiple nodes publishing "
                    "on same topic %s? IMU buffer is cleared!",
                    imuFrameId_.c_str(),
                    msg->header.frame_id.c_str(),
                    imuSub_.getTopic().c_str());
          imus_.clear();
          imuFrameId_.clear();
        }
        else
        {
          imuFrameId_ = msg->header.frame_id;
        }
      }
    }
  }

  void CoreWrapper::republishNodeDataCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
  {
    if (maxNodesRepublished_ > 0)
    {
      nodesToRepublish_.insert(msg->data.begin(), msg->data.end());
    }
    else
    {
      static bool warned = false;
      if (!warned)
      {
        NODELET_WARN("A node is requesting some node data "
                     "to be republished after the next update, "
                     "but parameter \"max_nodes_republished\" is not over 0, "
                     "ignoring the call. This warning is only printed once.");
        warned = true;
      }
    }
  }

  void CoreWrapper::interOdomCallback(const nav_msgs::OdometryConstPtr &msg)
  {
    if (!paused_)
    {
      interOdoms_.push_back(std::make_pair(*msg, rtabmap_ros::OdomInfo()));
    }
  }

  void CoreWrapper::interOdomInfoCallback(const nav_msgs::OdometryConstPtr &msg1, const rtabmap_ros::OdomInfoConstPtr &msg2)
  {
    if (!paused_)
    {
      interOdoms_.push_back(std::make_pair(*msg1, *msg2));
    }
  }

  void CoreWrapper::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr &msg)
  {
    Transform intialPose = rtabmap_ros::transformFromPoseMsg(msg->pose.pose);
    if (intialPose.isNull())
    {
      NODELET_ERROR("Pose received is null!");
      return;
    }

    rtabmap_.setInitialPose(intialPose);
  }

  void CoreWrapper::goalCommonCallback(
      int id,
      const std::string &label,
      const std::string &frameId,
      const Transform &pose,
      const ros::Time &stamp,
      double *planningTime)
  {
    UTimer timer;

    if (id == 0 && !label.empty() && rtabmap_.getMemory())
    {
      id = rtabmap_.getMemory()->getSignatureIdByLabel(label);
    }

    if (id > 0)
    {
      NODELET_INFO("Planning: set goal to node %d", id);
    }
    else if (id < 0)
    {
      NODELET_INFO("Planning: set goal to landmark %d", id);
    }
    else if (!pose.isNull())
    {
      NODELET_INFO("Planning: set goal %s", pose.prettyPrint().c_str());
    }

    if (planningTime)
    {
      *planningTime = 0.0;
    }

    bool success = false;
    if ((id != 0 && rtabmap_.computePath(id, true)) ||
        (!pose.isNull() && rtabmap_.computePath(pose)))
    {
      if (planningTime)
      {
        *planningTime = timer.elapsed();
      }
      NODELET_INFO("Planning: Time computing path = %f s", timer.ticks());
      const std::vector<std::pair<int, Transform>> &poses = rtabmap_.getPath();

      currentMetricGoal_.setNull();
      lastPublishedMetricGoal_.setNull();
      goalFrameId_.clear();
      latestNodeWasReached_ = false;
      if (poses.size() == 0)
      {
        NODELET_WARN("Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
                     rtabmap_.getGoalReachedRadius());
        rtabmap_.clearPath(1);
        if (goalReachedPub_.getNumSubscribers())
        {
          std_msgs::Bool result;
          result.data = true;
          goalReachedPub_.publish(result);
        }
        success = true;
      }
      else
      {
        currentMetricGoal_ = rtabmap_.getPose(rtabmap_.getPathCurrentGoalId());
        if (!currentMetricGoal_.isNull())
        {
          NODELET_INFO("Planning: Path successfully created (size=%d)", (int)poses.size());
          goalFrameId_ = frameId;

          // Adjust the target pose relative to last node
          if (rtabmap_.getPathCurrentGoalId() == rtabmap_.getPath().back().first && rtabmap_.getLocalOptimizedPoses().size())
          {
            if (rtabmap_.getLastLocalizationPose().getDistance(currentMetricGoal_) < rtabmap_.getLocalRadius())
            {
              latestNodeWasReached_ = true;
              Transform goalLocalTransform = Transform::getIdentity();
              if (!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
              {
                Transform localT = rtabmap_ros::getTransform(frameId_, goalFrameId_, stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
                if (!localT.isNull())
                {
                  goalLocalTransform = localT.inverse().to3DoF();
                }
              }
              currentMetricGoal_ *= rtabmap_.getPathTransformToGoal() * goalLocalTransform;
            }
          }

          publishCurrentGoal(stamp);
          publishLocalPath(stamp);
          publishGlobalPath(stamp);

          // Just output the path on screen
          std::stringstream stream;
          for (std::vector<std::pair<int, Transform>>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
          {
            if (iter != poses.begin())
            {
              stream << " ";
            }
            stream << iter->first;
          }
          NODELET_INFO("Global path: [%s]", stream.str().c_str());
          success = true;
        }
        else
        {
          NODELET_ERROR("Pose of node %d not found!? Cannot send a metric goal...", rtabmap_.getPathCurrentGoalId());
        }
      }
    }
    else if (!label.empty())
    {
      NODELET_ERROR("Planning: Node with label \"%s\" not found!", label.c_str());
    }
    else if (pose.isNull())
    {
      if (id > 0)
      {
        NODELET_ERROR("Planning: Could not plan to node %d! The node is not in map's graph (look for warnings before this message for more details).", id);
      }
      else if (id < 0)
      {
        NODELET_ERROR("Planning: Could not plan to landmark %d! The landmark is not in map's graph (look for warnings before this message for more details).", id);
      }
      else
      {
        NODELET_ERROR("Planning: Node id should be > 0 !");
      }
    }
    else
    {
      NODELET_ERROR("Planning: A node near the goal's pose not found! The pose may be to far from the graph (RGBD/LocalRadius=%f m)", rtabmap_.getLocalRadius());
    }

    if (!success)
    {
      rtabmap_.clearPath(-1);
      if (goalReachedPub_.getNumSubscribers())
      {
        std_msgs::Bool result;
        result.data = false;
        goalReachedPub_.publish(result);
      }
    }
  }

  void CoreWrapper::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    Transform targetPose = rtabmap_ros::transformFromPoseMsg(msg->pose, true);

    // transform goal in /map frame
    if (!msg->header.frame_id.empty() && mapFrameId_.compare(msg->header.frame_id) != 0)
    {
      Transform t = rtabmap_ros::getTransform(mapFrameId_, msg->header.frame_id, msg->header.stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
      if (t.isNull())
      {
        NODELET_ERROR("Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
                      msg->header.frame_id.c_str(), mapFrameId_.c_str());
        if (goalReachedPub_.getNumSubscribers())
        {
          std_msgs::Bool result;
          result.data = false;
          goalReachedPub_.publish(result);
        }
        return;
      }
      targetPose = t * targetPose;
    }
    // else assume map frame if not set

    goalCommonCallback(0, "", "", targetPose, msg->header.stamp);
  }

  void CoreWrapper::goalNodeCallback(const rtabmap_ros::GoalConstPtr &msg)
  {
    if (msg->node_id == 0 && msg->node_label.empty())
    {
      NODELET_ERROR("Node id or label should be set!");
      if (goalReachedPub_.getNumSubscribers())
      {
        std_msgs::Bool result;
        result.data = false;
        goalReachedPub_.publish(result);
      }
      return;
    }
    goalCommonCallback(msg->node_id, msg->node_label, msg->frame_id, Transform(), msg->header.stamp);
  }

  bool CoreWrapper::updateRtabmapCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    ros::NodeHandle pnh("~");
    for (rtabmap::ParametersMap::iterator iter = parameters_.begin(); iter != parameters_.end(); ++iter)
    {
      std::string vStr;
      bool vBool;
      int vInt;
      double vDouble;
      if (pnh.getParam(iter->first, vStr))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), vStr.c_str());
        iter->second = vStr;
      }
      else if (pnh.getParam(iter->first, vBool))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uBool2Str(vBool).c_str());
        iter->second = uBool2Str(vBool);
      }
      else if (pnh.getParam(iter->first, vInt))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vInt).c_str());
        iter->second = uNumber2Str(vInt).c_str();
      }
      else if (pnh.getParam(iter->first, vDouble))
      {
        NODELET_INFO("Setting RTAB-Map parameter \"%s\"=\"%s\"", iter->first.c_str(), uNumber2Str(vDouble).c_str());
        iter->second = uNumber2Str(vDouble).c_str();
      }
    }
    NODELET_INFO("rtabmap: Updating parameters");
    if (parameters_.find(Parameters::kRtabmapDetectionRate()) != parameters_.end())
    {
      rate_ = uStr2Float(parameters_.at(Parameters::kRtabmapDetectionRate()));
      NODELET_INFO("RTAB-Map rate detection = %f Hz", rate_);
    }
    if (parameters_.find(Parameters::kRtabmapCreateIntermediateNodes()) != parameters_.end())
    {
      createIntermediateNodes_ = uStr2Bool(parameters_.at(Parameters::kRtabmapCreateIntermediateNodes()));
      NODELET_INFO("Create intermediate nodes = %s", createIntermediateNodes_ ? "true" : "false");
    }
    if (parameters_.find(Parameters::kGridGlobalMaxNodes()) != parameters_.end())
    {
      mappingMaxNodes_ = uStr2Int(parameters_.at(Parameters::kGridGlobalMaxNodes()));
      NODELET_INFO("Max mapping nodes = %d", mappingMaxNodes_);
    }
    if (parameters_.find(Parameters::kGridGlobalAltitudeDelta()) != parameters_.end())
    {
      mappingAltitudeDelta_ = uStr2Float(parameters_.at(Parameters::kGridGlobalAltitudeDelta()));
      NODELET_INFO("Mapping altitude delta = %f", mappingAltitudeDelta_);
    }
    if (parameters_.find(Parameters::kRtabmapImagesAlreadyRectified()) != parameters_.end())
    {
      alreadyRectifiedImages_ = uStr2Bool(parameters_.at(Parameters::kRtabmapImagesAlreadyRectified()));
      NODELET_INFO("Already rectified images = %s", alreadyRectifiedImages_ ? "true" : "false");
    }
    if (parameters_.find(Parameters::kRegForce3DoF()) != parameters_.end())
    {
      twoDMapping_ = uStr2Bool(parameters_.at(Parameters::kRegForce3DoF()));
      NODELET_INFO("2D mapping = %s", twoDMapping_ ? "true" : "false");
    }
    rtabmap_.parseParameters(parameters_);
    mapsManager_.setParameters(parameters_);
    return true;
  }

  bool CoreWrapper::resetRtabmapCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Reset");
    rtabmap_.resetMemory();
    covariance_ = cv::Mat();
    lastPose_.setIdentity();
    lastPoseIntermediate_ = false;
    currentMetricGoal_.setNull();
    lastPublishedMetricGoal_.setNull();
    goalFrameId_.clear();
    latestNodeWasReached_ = false;
    mapsManager_.clear();
    previousStamp_ = ros::Time(0);
    globalPose_.header.stamp = ros::Time(0);
    gps_ = rtabmap::GPS();
    tags_.clear();
    userDataMutex_.lock();
    userData_ = cv::Mat();
    userDataMutex_.unlock();
    imus_.clear();
    imuFrameId_.clear();
    interOdoms_.clear();
    mapToOdomMutex_.lock();
    mapToOdom_.setIdentity();
    mapToOdomMutex_.unlock();
    nodesToRepublish_.clear();

    return true;
  }

  bool CoreWrapper::pauseRtabmapCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    if (paused_)
    {
      NODELET_WARN("rtabmap: Already paused!");
    }
    else
    {
      paused_ = true;
      NODELET_INFO("rtabmap: paused!");
      ros::NodeHandle pnh("~");
      pnh.setParam("is_rtabmap_paused", true);
    }
    return true;
  }

  bool CoreWrapper::resumeRtabmapCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    if (!paused_)
    {
      NODELET_WARN("rtabmap: Already running!");
    }
    else
    {
      paused_ = false;
      NODELET_INFO("rtabmap: resumed!");
      ros::NodeHandle pnh("~");
      pnh.setParam("is_rtabmap_paused", false);
    }
    return true;
  }

  bool CoreWrapper::loadDatabaseCallback(rtabmap_ros::LoadDatabase::Request &req, rtabmap_ros::LoadDatabase::Response &)
  {
    NODELET_INFO("LoadDatabase: Loading database (%s, clear=%s)...", req.database_path.c_str(), req.clear ? "true" : "false");
    std::string newDatabasePath = uReplaceChar(req.database_path, '~', UDirectory::homeDir());
    std::string dir = UDirectory::getDir(newDatabasePath);
    if (!UDirectory::exists(dir))
    {
      ROS_ERROR("Directory %s doesn't exist! Cannot load database \"%s\"", newDatabasePath.c_str(), dir.c_str());
      return false;
    }

    if (UFile::exists(newDatabasePath) && req.clear)
    {
      UFile::erase(newDatabasePath);
    }

    // Close old database
    NODELET_INFO("LoadDatabase: Saving current map (%s)...", databasePath_.c_str());
    if (rtabmap_.getMemory())
    {
      // save the grid map
      float xMin = 0.0f, yMin = 0.0f, gridCellSize = 0.05f;
      cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
      if (!pixels.empty())
      {
        printf("rtabmap: 2D occupancy grid map saved.\n");
        rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
      }
    }
    rtabmap_.close();
    NODELET_INFO("LoadDatabase: Saving current map (%s, %ld MB)... done!", databasePath_.c_str(), UFile::length(databasePath_) / (1024 * 1024));

    covariance_ = cv::Mat();
    lastPose_.setIdentity();
    lastPoseIntermediate_ = false;
    currentMetricGoal_.setNull();
    lastPublishedMetricGoal_.setNull();
    goalFrameId_.clear();
    latestNodeWasReached_ = false;
    mapsManager_.clear();
    previousStamp_ = ros::Time(0);
    globalPose_.header.stamp = ros::Time(0);
    gps_ = rtabmap::GPS();
    tags_.clear();
    userDataMutex_.lock();
    userData_ = cv::Mat();
    userDataMutex_.unlock();
    imus_.clear();
    imuFrameId_.clear();
    interOdoms_.clear();
    mapToOdomMutex_.lock();
    mapToOdom_.setIdentity();
    mapToOdomMutex_.unlock();
    nodesToRepublish_.clear();

    // Open new database
    databasePath_ = newDatabasePath;

    // modify default parameters with those in the database
    if (!req.clear && UFile::exists(databasePath_))
    {
      ParametersMap dbParameters;
      rtabmap::DBDriver *driver = rtabmap::DBDriver::create();
      if (driver->openConnection(databasePath_))
      {
        dbParameters = driver->getLastParameters(); // parameter migration is already done
      }
      delete driver;
      for (ParametersMap::iterator iter = dbParameters.begin(); iter != dbParameters.end(); ++iter)
      {
        if (iter->first.compare(Parameters::kRtabmapWorkingDirectory()) == 0)
        {
          // ignore working directory
          continue;
        }
        if (parameters_.find(iter->first) == parameters_.end() &&
            parameters_.find(iter->first)->second.compare(iter->second) != 0)
        {
          NODELET_WARN("RTAB-Map parameter \"%s\" from database (%s) is different "
                       "from the current used one (%s). We still keep the "
                       "current parameter value (%s). If you want to switch between databases "
                       "with different configurations, restart rtabmap node instead of using this service.",
                       iter->first.c_str(), iter->second.c_str(),
                       parameters_.find(iter->first)->second.c_str(),
                       parameters_.find(iter->first)->second.c_str());
        }
      }
    }

    NODELET_INFO("LoadDatabase: Loading database...");
    rtabmap_.init(parameters_, databasePath_);
    NODELET_INFO("LoadDatabase: Loading database... done!");

    if (rtabmap_.getMemory())
    {
      if (useSavedMap_ && !rtabmap_.getMemory()->isIncremental())
      {
        float xMin, yMin, gridCellSize;
        cv::Mat map = rtabmap_.getMemory()->load2DMap(xMin, yMin, gridCellSize);
        if (!map.empty())
        {
          NODELET_INFO("LoadDatabase: 2D occupancy grid map loaded (%dx%d).", map.cols, map.rows);
          mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());
        }
      }

      if (rtabmap_.getMemory()->getWorkingMem().size() > 1)
      {
        NODELET_INFO("LoadDatabase: Working Memory = %d, Local map = %d.",
                     (int)rtabmap_.getMemory()->getWorkingMem().size() - 1,
                     (int)rtabmap_.getLocalOptimizedPoses().size());
      }

      if (databasePath_.size())
      {
        NODELET_INFO("LoadDatabase: Database version = \"%s\".", rtabmap_.getMemory()->getDatabaseVersion().c_str());
      }

      if (rtabmap_.getMemory()->isIncremental())
      {
        NODELET_INFO("LoadDatabase: SLAM mode (%s=true)", Parameters::kMemIncrementalMemory().c_str());
      }
      else
      {
        NODELET_INFO("LoadDatabase: Localization mode (%s=false)", Parameters::kMemIncrementalMemory().c_str());
      }

      return true;
    }

    return false;
  }

  bool CoreWrapper::triggerNewMapCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Trigger new map");
    rtabmap_.triggerNewMap();
    return true;
  }

  bool CoreWrapper::backupDatabaseCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("Backup: Saving memory...");
    if (rtabmap_.getMemory())
    {
      // save the grid map
      float xMin = 0.0f, yMin = 0.0f, gridCellSize = 0.05f;
      cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
      if (!pixels.empty())
      {
        printf("rtabmap: 2D occupancy grid map saved.\n");
        rtabmap_.getMemory()->save2DMap(pixels, xMin, yMin, gridCellSize);
      }
    }
    rtabmap_.close();
    NODELET_INFO("Backup: Saving memory... done!");

    covariance_ = cv::Mat();
    lastPose_.setIdentity();
    currentMetricGoal_.setNull();
    lastPublishedMetricGoal_.setNull();
    goalFrameId_.clear();
    latestNodeWasReached_ = false;
    userDataMutex_.lock();
    userData_ = cv::Mat();
    userDataMutex_.unlock();
    globalPose_.header.stamp = ros::Time(0);
    gps_ = rtabmap::GPS();
    tags_.clear();
    nodesToRepublish_.clear();

    NODELET_INFO("Backup: Saving \"%s\" to \"%s\"...", databasePath_.c_str(), (databasePath_ + ".back").c_str());
    UFile::copy(databasePath_, databasePath_ + ".back");
    NODELET_INFO("Backup: Saving \"%s\" to \"%s\"... done!", databasePath_.c_str(), (databasePath_ + ".back").c_str());

    NODELET_INFO("Backup: Reloading memory...");
    rtabmap_.init(parameters_, databasePath_);
    NODELET_INFO("Backup: Reloading memory... done!");

    return true;
  }

  void CoreWrapper::republishMaps()
  {
    ros::Time stamp = ros::Time::now();
    mapsManager_.publishMaps(rtabmap_.getLocalOptimizedPoses(), stamp, mapFrameId_);

    if (mapDataPub_.getNumSubscribers())
    {
      rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;

      rtabmap_ros::mapDataToROS(
          rtabmap_.getLocalOptimizedPoses(),
          rtabmap_.getLocalConstraints(),
          std::map<int, Signature>(),
          rtabmap_.getMapCorrection(),
          *msg);

      mapDataPub_.publish(msg);
    }

    if (mapGraphPub_.getNumSubscribers())
    {
      rtabmap_ros::MapGraphPtr msg(new rtabmap_ros::MapGraph);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;

      rtabmap_ros::mapGraphToROS(
          rtabmap_.getLocalOptimizedPoses(),
          rtabmap_.getLocalConstraints(),
          rtabmap_.getMapCorrection(),
          *msg);

      mapGraphPub_.publish(msg);
    }
  }

  bool CoreWrapper::detectMoreLoopClosuresCallback(rtabmap_ros::DetectMoreLoopClosures::Request &req, rtabmap_ros::DetectMoreLoopClosures::Response &res)
  {
    NODELET_WARN("Detect more loop closures service called");

    UTimer timer;
    float clusterRadiusMax = 1;
    float clusterRadiusMin = 0;
    float clusterAngle = 0;
    int iterations = 1;
    bool intraSession = true;
    bool interSession = true;
    if (req.cluster_radius_max > 0.0f)
    {
      clusterRadiusMax = req.cluster_radius_max;
    }
    if (req.cluster_radius_min >= 0.0f)
    {
      clusterRadiusMin = req.cluster_radius_min;
    }
    if (req.cluster_angle >= 0.0f)
    {
      clusterAngle = req.cluster_angle;
    }
    if (req.iterations >= 1.0f)
    {
      iterations = (int)req.iterations;
    }
    if (req.intra_only)
    {
      interSession = false;
    }
    else if (req.inter_only)
    {
      intraSession = false;
    }
    NODELET_WARN("Post-Processing service called: Detecting more loop closures "
                 "(max radius=%f, min radius=%f, angle=%f, iterations=%d, intra=%s, inter=%s)...",
                 clusterRadiusMax,
                 clusterRadiusMin,
                 clusterAngle,
                 iterations,
                 intraSession ? "true" : "false",
                 interSession ? "true" : "false");
    res.detected = rtabmap_.detectMoreLoopClosures(
        clusterRadiusMax,
        clusterAngle * M_PI / 180.0,
        iterations,
        intraSession,
        interSession,
        0,
        clusterRadiusMin);
    if (res.detected < 0)
    {
      NODELET_ERROR("Post-Processing: Detecting more loop closures failed!");
    }
    else
    {
      NODELET_WARN("Post-Processing: Detected %d loop closures! (%fs)", res.detected, timer.ticks());

      if (res.detected > 0)
      {
        republishMaps();
      }
      return true;
    }
    return false;
  }

  bool CoreWrapper::cleanupLocalGridsCallback(rtabmap_ros::CleanupLocalGrids::Request &req, rtabmap_ros::CleanupLocalGrids::Response &res)
  {
    NODELET_WARN("Cleanup local grids service called");
    UTimer timer;
    int radius = 1;
    bool filterScans = false;
    if (req.radius > 1.0f)
    {
      radius = (int)req.radius;
    }
    filterScans = req.filter_scans;
    float xMin, yMin, gridCellSize;
    cv::Mat map = mapsManager_.getGridMap(xMin, yMin, gridCellSize);
    if (map.empty())
    {
      NODELET_ERROR("Post-Processing: Cleanup local grids failed! There is no optimized map.");
      return false;
    }
    std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
    NODELET_WARN("Post-Processing: Cleanup local grids... (radius=%d, filter scans=%s)",
                 radius,
                 filterScans ? "true" : "false");
    res.modified = rtabmap_.cleanupLocalGrids(poses, map, xMin, yMin, gridCellSize, radius, filterScans);
    if (res.modified < 0)
    {
      NODELET_ERROR("Post-Processing: Cleanup local grids failed!");
    }
    else
    {
      if (filterScans)
      {
        NODELET_WARN("Post-Processing: %d grids and scans modified! (%fs)", res.modified, timer.ticks());
      }
      else
      {
        NODELET_WARN("Post-Processing: %d grids modified! (%fs)", res.modified, timer.ticks());
      }
      if (res.modified > 0)
      {
        // We should update MapsManager's cache with the modifications
        mapsManager_.clear();
        mapsManager_.set2DMap(map, xMin, yMin, gridCellSize, rtabmap_.getLocalOptimizedPoses(), rtabmap_.getMemory());

        republishMaps();
      }
      return true;
    }

    return false;
  }
  bool CoreWrapper::globalBundleAdjustmentCallback(rtabmap_ros::GlobalBundleAdjustment::Request &req, rtabmap_ros::GlobalBundleAdjustment::Response &res)
  {
    NODELET_WARN("Global bundle adjustment service called");

    UTimer timer;
    int optimizer = (int)Optimizer::kTypeG2O; // g2o
    int iterations = Parameters::defaultOptimizerIterations();
    float pixelVariance = Parameters::defaultg2oPixelVariance();
    bool rematchFeatures = true;
    Parameters::parse(parameters_, Parameters::kOptimizerIterations(), iterations);
    Parameters::parse(parameters_, Parameters::kg2oPixelVariance(), pixelVariance);
    if (req.type == 1.0f)
    {
      optimizer = (int)Optimizer::kTypeCVSBA;
    }
    if (req.iterations >= 1.0f)
    {
      iterations = req.iterations;
    }
    if (req.pixel_variance > 0.0f)
    {
      pixelVariance = req.pixel_variance;
    }
    rematchFeatures = !req.voc_matches;

    NODELET_WARN("Post-Processing: Global Bundle Adjustment... "
                 "(Optimizer=%s, iterations=%d, pixel variance=%f, rematch=%s)...",
                 optimizer == Optimizer::kTypeG2O ? "g2o" : "cvsba",
                 iterations,
                 pixelVariance,
                 rematchFeatures ? "true" : "false");
    bool success = rtabmap_.globalBundleAdjustment((Optimizer::Type)optimizer, rematchFeatures, iterations, pixelVariance);
    if (!success)
    {
      NODELET_ERROR("Post-Processing: Global Bundle Adjustment failed!");
    }
    else
    {
      NODELET_WARN("Post-Processing: Global Bundle Adjustment... done! (%fs)", timer.ticks());
      republishMaps();
      return true;
    }

    return false;
  }

  bool CoreWrapper::setModeLocalizationCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Set localization mode");
    rtabmap::ParametersMap parameters;
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "false"));
    ros::NodeHandle &nh = getNodeHandle();
    nh.setParam(rtabmap::Parameters::kMemIncrementalMemory(), "false");
    rtabmap_.parseParameters(parameters);
    NODELET_INFO("rtabmap: Localization mode enabled!");
    return true;
  }

  bool CoreWrapper::setModeMappingCallback(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Set mapping mode");
    rtabmap::ParametersMap parameters;
    parameters.insert(rtabmap::ParametersPair(rtabmap::Parameters::kMemIncrementalMemory(), "true"));
    ros::NodeHandle &nh = getNodeHandle();
    nh.setParam(rtabmap::Parameters::kMemIncrementalMemory(), "true");
    rtabmap_.parseParameters(parameters);
    NODELET_INFO("rtabmap: Mapping mode enabled!");
    return true;
  }

  bool CoreWrapper::setLogDebug(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Set log level to Debug");
    ULogger::setLevel(ULogger::kDebug);
    return true;
  }
  bool CoreWrapper::setLogInfo(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Set log level to Info");
    ULogger::setLevel(ULogger::kInfo);
    return true;
  }
  bool CoreWrapper::setLogWarn(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Set log level to Warning");
    ULogger::setLevel(ULogger::kWarning);
    return true;
  }
  bool CoreWrapper::setLogError(std_srvs::Empty::Request &, std_srvs::Empty::Response &)
  {
    NODELET_INFO("rtabmap: Set log level to Error");
    ULogger::setLevel(ULogger::kError);
    return true;
  }

  bool CoreWrapper::getNodeDataCallback(rtabmap_ros::GetNodeData::Request &req, rtabmap_ros::GetNodeData::Response &res)
  {
    NODELET_INFO("rtabmap: Getting node data (%d node(s), images=%s scan=%s grid=%s user_data=%s)...",
                 (int)req.ids.size(),
                 req.images ? "true" : "false",
                 req.scan ? "true" : "false",
                 req.grid ? "true" : "false",
                 req.user_data ? "true" : "false");

    if (req.ids.empty() && rtabmap_.getMemory() && rtabmap_.getMemory()->getLastWorkingSignature())
    {
      req.ids.push_back(rtabmap_.getMemory()->getLastWorkingSignature()->id());
    }
    for (size_t i = 0; i < req.ids.size(); ++i)
    {
      int id = req.ids[i];
      Signature s = rtabmap_.getSignatureCopy(id, req.images, req.scan, req.user_data, req.grid, true, true);

      if (s.id() > 0)
      {
        NodeData msg;
        rtabmap_ros::nodeDataToROS(s, msg);
        res.data.push_back(msg);
      }
    }

    return !res.data.empty();
  }

  bool CoreWrapper::getMapDataCallback(rtabmap_ros::GetMap::Request &req, rtabmap_ros::GetMap::Response &res)
  {
    NODELET_INFO("rtabmap: Getting map (global=%s optimized=%s graphOnly=%s)...",
                 req.global ? "true" : "false",
                 req.optimized ? "true" : "false",
                 req.graphOnly ? "true" : "false");
    std::map<int, Signature> signatures;
    std::map<int, Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;

    rtabmap_.getGraph(
        poses,
        constraints,
        req.optimized,
        req.global,
        &signatures,
        !req.graphOnly,
        !req.graphOnly,
        !req.graphOnly,
        !req.graphOnly);

    // RGB-D SLAM data
    rtabmap_ros::mapDataToROS(poses,
                              constraints,
                              signatures,
                              mapToOdom_,
                              res.data);

    res.data.header.stamp = ros::Time::now();
    res.data.header.frame_id = mapFrameId_;

    return true;
  }

  bool CoreWrapper::getMapData2Callback(rtabmap_ros::GetMap2::Request &req, rtabmap_ros::GetMap2::Response &res)
  {
    NODELET_INFO("rtabmap: Getting map (global=%s optimized=%s with_images=%s with_scans=%s with_user_data=%s with_grids=%s)...",
                 req.global ? "true" : "false",
                 req.optimized ? "true" : "false",
                 req.with_images ? "true" : "false",
                 req.with_scans ? "true" : "false",
                 req.with_user_data ? "true" : "false",
                 req.with_grids ? "true" : "false");
    std::map<int, Signature> signatures;
    std::map<int, Transform> poses;
    std::multimap<int, rtabmap::Link> constraints;

    rtabmap_.getGraph(
        poses,
        constraints,
        req.optimized,
        req.global,
        &signatures,
        req.with_images,
        req.with_scans,
        req.with_user_data,
        req.with_grids,
        req.with_words,
        req.with_global_descriptors);

    // RGB-D SLAM data
    rtabmap_ros::mapDataToROS(poses,
                              constraints,
                              signatures,
                              mapToOdom_,
                              res.data);

    res.data.header.stamp = ros::Time::now();
    res.data.header.frame_id = mapFrameId_;

    return true;
  }

  // bool CoreWrapper::getProjMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
  // {
  //   if (parameters_.find(Parameters::kGridSensor()) != parameters_.end() &&
  //       uStr2Int(parameters_.at(Parameters::kGridSensor())) == 0)
  //   {
  //     NODELET_WARN("/get_proj_map service is deprecated! Call /get_grid_map service "
  //                  "instead with <param name=\"%s\" type=\"string\" value=\"1\"/>. "
  //                  "Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
  //                  "all occupancy grid parameters.",
  //                  Parameters::kGridSensor().c_str());
  //   }
  //   else
  //   {
  //     NODELET_WARN("/get_proj_map service is deprecated! Call /get_grid_map service instead.");
  //   }
  //   return getGridMapCallback(req, res);
  // }

  // bool CoreWrapper::getGridMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
  // {
  //   NODELET_WARN("/get_grid_map service is deprecated! Call /get_map service instead.");
  //   return getMapCallback(req, res);
  // }

  // bool CoreWrapper::getMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
  // {
  //   // Make sure grid map cache is up to date (in case there is no subscriber on map topics)
  //   std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
  //   mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), true, false);

  //   // create the grid map
  //   float xMin = 0.0f, yMin = 0.0f, gridCellSize = 0.05f;
  //   cv::Mat pixels = mapsManager_.getGridMap(xMin, yMin, gridCellSize);

  //   if (!pixels.empty())
  //   {
  //     // init
  //     res.map.info.resolution = gridCellSize;
  //     res.map.info.origin.position.x = 0.0;
  //     res.map.info.origin.position.y = 0.0;
  //     res.map.info.origin.position.z = 0.0;
  //     res.map.info.origin.orientation.x = 0.0;
  //     res.map.info.origin.orientation.y = 0.0;
  //     res.map.info.origin.orientation.z = 0.0;
  //     res.map.info.origin.orientation.w = 1.0;

  //     res.map.info.width = pixels.cols;
  //     res.map.info.height = pixels.rows;
  //     res.map.info.origin.position.x = xMin;
  //     res.map.info.origin.position.y = yMin;
  //     res.map.data.resize(res.map.info.width * res.map.info.height);

  //     memcpy(res.map.data.data(), pixels.data, res.map.info.width * res.map.info.height);

  //     res.map.header.frame_id = mapFrameId_;
  //     res.map.header.stamp = ros::Time::now();
  //     return true;
  //   }
  //   else
  //   {
  //     NODELET_WARN("rtabmap: The map is empty!");
  //   }
  //   return false;
  // }

  // bool CoreWrapper::getProbMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res)
  // {
  //   // Make sure grid map cache is up to date (in case there is no subscriber on map topics)
  //   std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
  //   mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), true, false);

  //   // create the grid map
  //   float xMin = 0.0f, yMin = 0.0f, gridCellSize = 0.05f;
  //   cv::Mat pixels = mapsManager_.getGridProbMap(xMin, yMin, gridCellSize);

  //   if (!pixels.empty())
  //   {
  //     // init
  //     res.map.info.resolution = gridCellSize;
  //     res.map.info.origin.position.x = 0.0;
  //     res.map.info.origin.position.y = 0.0;
  //     res.map.info.origin.position.z = 0.0;
  //     res.map.info.origin.orientation.x = 0.0;
  //     res.map.info.origin.orientation.y = 0.0;
  //     res.map.info.origin.orientation.z = 0.0;
  //     res.map.info.origin.orientation.w = 1.0;

  //     res.map.info.width = pixels.cols;
  //     res.map.info.height = pixels.rows;
  //     res.map.info.origin.position.x = xMin;
  //     res.map.info.origin.position.y = yMin;
  //     res.map.data.resize(res.map.info.width * res.map.info.height);

  //     memcpy(res.map.data.data(), pixels.data, res.map.info.width * res.map.info.height);

  //     res.map.header.frame_id = mapFrameId_;
  //     res.map.header.stamp = ros::Time::now();
  //     return true;
  //   }
  //   else
  //   {
  //     NODELET_WARN("rtabmap: The map is empty!");
  //   }
  //   return false;
  // }

  ///
  /// TODO: need to evaluate use of rtabmap_mtx_ in callback below if using this callback;
  ///       (hasn't been fully evaluated at present since we aren't using this callback)
  ///
  bool CoreWrapper::publishMapCallback(rtabmap_ros::PublishMap::Request &req, rtabmap_ros::PublishMap::Response &res)
  {
    NODELET_INFO("rtabmap: Publishing map...");

    ros::Time now = ros::Time::now();

    if (mapDataPub_.getNumSubscribers() ||
        (!req.graphOnly && mapsManager_.hasSubscribers()) ||
        (req.graphOnly && (labelsPub_.getNumSubscribers() || mapGraphPub_.getNumSubscribers() || mapPathPub_.getNumSubscribers())))
    {
      std::map<int, Transform> poses;
      std::multimap<int, rtabmap::Link> constraints;
      std::map<int, Signature> signatures;

      rtabmap_mtx_.lock();
      rtabmap_.getGraph(
          poses,
          constraints,
          req.optimized,
          req.global,
          &signatures,
          !req.graphOnly,
          !req.graphOnly,
          !req.graphOnly,
          !req.graphOnly);
      rtabmap_mtx_.unlock();

      ros::Time now = ros::Time::now();
      if (mapDataPub_.getNumSubscribers())
      {
        rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
        msg->header.stamp = now;
        msg->header.frame_id = mapFrameId_;

        rtabmap_ros::mapDataToROS(poses,
                                  constraints,
                                  signatures,
                                  mapToOdom_,
                                  *msg);

        mapDataPub_.publish(msg);
      }

      if (mapGraphPub_.getNumSubscribers())
      {
        rtabmap_ros::MapGraphPtr msg(new rtabmap_ros::MapGraph);
        msg->header.stamp = now;
        msg->header.frame_id = mapFrameId_;

        rtabmap_ros::mapGraphToROS(poses,
                                   constraints,
                                   mapToOdom_,
                                   *msg);

        mapGraphPub_.publish(msg);
      }

      bool pubLabels = labelsPub_.getNumSubscribers();
      visualization_msgs::MarkerArray markers;
      if ((landmarksPub_.getNumSubscribers() || pubLabels) && !poses.empty() && poses.begin()->first < 0)
      {
        geometry_msgs::PoseArrayPtr msg(new geometry_msgs::PoseArray);
        msg->header.stamp = now;
        msg->header.frame_id = mapFrameId_;
        for (std::map<int, Transform>::const_iterator iter = poses.begin(); iter != poses.end() && iter->first < 0; ++iter)
        {
          geometry_msgs::Pose p;
          rtabmap_ros::transformToPoseMsg(iter->second, p);
          msg->poses.push_back(p);

          if (pubLabels)
          {
            // Add landmark ids
            visualization_msgs::Marker marker;
            marker.header.frame_id = mapFrameId_;
            marker.header.stamp = now;
            marker.ns = "landmarks";
            marker.id = iter->first;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = iter->second.x();
            marker.pose.position.y = iter->second.y();
            marker.pose.position.z = iter->second.z();
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.35;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;
            marker.lifetime = ros::Duration(2.0f / rate_);

            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = uNumber2Str(iter->first);

            markers.markers.push_back(marker);
          }
        }

        landmarksPub_.publish(msg);
      }

      if (!req.graphOnly)
      {
        if (mapsManager_.hasSubscribers())
        {
          std::map<int, Transform> filteredPoses(poses.lower_bound(1), poses.end());
          if ((mappingMaxNodes_ > 0 || mappingAltitudeDelta_ > 0.0) && filteredPoses.size() > 1)
          {
            std::map<int, Transform> nearestPoses = filterNodesToAssemble(filteredPoses, filteredPoses.rbegin()->second);
          }
          if (signatures.size())
          {
          #ifndef RTABMAP_OCTOMAP
            filteredPoses = mapsManager_.updateMapCaches(
                filteredPoses,
                rtabmap_.getMemory(),
                false,
                false,
                rtabmap_mtx_,
                signatures);
          #else
            rtabmap::SemanticOctoMap::AuxSignatureData auxSignatureData;
            filteredPoses = mapsManager_.updateMapCaches(
                filteredPoses,
                rtabmap_.getMemory(),
                false,
                false,
                rtabmap_mtx_,
                auxSignatureData,
                signatures);
          #endif
          }
          else
          {
            filteredPoses = mapsManager_.getFilteredPoses(filteredPoses);
          }
          mapsManager_.publishMaps(filteredPoses, now, mapFrameId_);
        }
        else
        {
          // this will cleanup the cache if there are no subscribers
          mapsManager_.publishMaps(std::map<int, Transform>(), now, mapFrameId_);
        }
      }

      bool pubPath = mapPathPub_.getNumSubscribers();
      if (pubLabels || pubPath)
      {
        if (poses.size() && signatures.size())
        {
          nav_msgs::Path path;
          if (pubPath)
          {
            path.poses.resize(poses.size());
          }
          int oi = 0;
          for (std::map<int, Signature>::const_iterator iter = signatures.begin();
               iter != signatures.end();
               ++iter)
          {
            std::map<int, Transform>::const_iterator poseIter = poses.find(iter->first);
            if (poseIter != poses.end())
            {
              if (pubLabels)
              {
                // Add labels
                if (!iter->second.getLabel().empty())
                {
                  visualization_msgs::Marker marker;
                  marker.header.frame_id = mapFrameId_;
                  marker.header.stamp = now;
                  marker.ns = "labels";
                  marker.id = iter->first;
                  marker.action = visualization_msgs::Marker::ADD;
                  marker.pose.position.x = poseIter->second.x();
                  marker.pose.position.y = poseIter->second.y();
                  marker.pose.position.z = poseIter->second.z();
                  marker.pose.orientation.x = 0.0;
                  marker.pose.orientation.y = 0.0;
                  marker.pose.orientation.z = 0.0;
                  marker.pose.orientation.w = 1.0;
                  marker.scale.x = 1;
                  marker.scale.y = 1;
                  marker.scale.z = 0.5;
                  marker.color.a = 0.7;
                  marker.color.r = 1.0;
                  marker.color.g = 0.0;
                  marker.color.b = 0.0;

                  marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                  marker.text = iter->second.getLabel();

                  markers.markers.push_back(marker);
                }
                // Add node ids
                visualization_msgs::Marker marker;
                marker.header.frame_id = mapFrameId_;
                marker.header.stamp = now;
                marker.ns = "ids";
                marker.id = iter->first;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = poseIter->second.x();
                marker.pose.position.y = poseIter->second.y();
                marker.pose.position.z = poseIter->second.z();
                marker.pose.orientation.x = 0.0;
                marker.pose.orientation.y = 0.0;
                marker.pose.orientation.z = 0.0;
                marker.pose.orientation.w = 1.0;
                marker.scale.x = 1;
                marker.scale.y = 1;
                marker.scale.z = 0.2;
                marker.color.a = 0.5;
                marker.color.r = 1.0;
                marker.color.g = 1.0;
                marker.color.b = 1.0;
                marker.lifetime = ros::Duration(2.0f / rate_);

                marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
                marker.text = uNumber2Str(iter->first);

                markers.markers.push_back(marker);
              }
              if (pubPath)
              {
                rtabmap_ros::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
                path.poses.at(oi).header.frame_id = mapFrameId_;
                path.poses.at(oi).header.stamp = ros::Time(iter->second.getStamp());
                ++oi;
              }
            }
          }

          if (pubLabels && markers.markers.size())
          {
            labelsPub_.publish(markers);
          }
          if (pubPath && oi)
          {
            path.header.frame_id = mapFrameId_;
            path.header.stamp = now;
            path.poses.resize(oi);
            mapPathPub_.publish(path);
          }
        }
      }
    }
    else
    {
      UWARN("No subscribers, don't need to publish!");
      if (!req.graphOnly)
      {
        // this will cleanup the cache if there are no subscribers
        mapsManager_.publishMaps(std::map<int, Transform>(), now, mapFrameId_);
      }
    }

    return true;
  }

  bool CoreWrapper::getPlanCallback(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res)
  {
    Transform pose = rtabmap_ros::transformFromPoseMsg(req.goal.pose, true);
    UTimer timer;
    if (!pose.isNull())
    {
      // transform goal in /map frame
      Transform coordinateTransform = Transform::getIdentity();
      if (!req.goal.header.frame_id.empty() && mapFrameId_.compare(req.goal.header.frame_id) != 0)
      {
        coordinateTransform = rtabmap_ros::getTransform(mapFrameId_, req.goal.header.frame_id, req.goal.header.stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
        if (coordinateTransform.isNull())
        {
          NODELET_ERROR("Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
                        req.goal.header.frame_id.c_str(), mapFrameId_.c_str());
          return false;
        }
        pose = coordinateTransform * pose;
      }
      // else assume map frame if not set

      // To convert back the poses in goal frame
      coordinateTransform = coordinateTransform.inverse();

      if (rtabmap_.computePath(pose, req.tolerance))
      {
        NODELET_INFO("Planning: Time computing path = %f s", timer.ticks());
        const std::vector<std::pair<int, Transform>> &poses = rtabmap_.getPath();
        res.plan.header.frame_id = req.goal.header.frame_id;
        res.plan.header.stamp = req.goal.header.stamp;
        if (poses.size() == 0)
        {
          NODELET_WARN("Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
                       rtabmap_.getGoalReachedRadius());
          // just set the goal directly
          res.plan.poses.resize(1);
          rtabmap_ros::transformToPoseMsg(coordinateTransform * pose, res.plan.poses[0].pose);
        }
        else
        {
          res.plan.poses.resize(poses.size());
          int oi = 0;
          for (std::vector<std::pair<int, Transform>>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
          {
            res.plan.poses[oi].header = res.plan.header;
            rtabmap_ros::transformToPoseMsg(coordinateTransform * iter->second, res.plan.poses[oi].pose);
            ++oi;
          }
          if (!rtabmap_.getPathTransformToGoal().isIdentity())
          {
            res.plan.poses.resize(res.plan.poses.size() + 1);
            res.plan.poses[res.plan.poses.size() - 1].header = res.plan.header;
            Transform p = rtabmap_.getPath().back().second * rtabmap_.getPathTransformToGoal();
            rtabmap_ros::transformToPoseMsg(coordinateTransform * p, res.plan.poses[res.plan.poses.size() - 1].pose);
          }

          // Just output the path on screen
          std::stringstream stream;
          for (std::vector<std::pair<int, Transform>>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
          {
            if (iter != poses.begin())
            {
              stream << " ";
            }
            stream << iter->first;
          }
          NODELET_INFO("Planned path: [%s]", stream.str().c_str());
        }
      }
      rtabmap_.clearPath(0);
    }
    return true;
  }

  bool CoreWrapper::getPlanNodesCallback(rtabmap_ros::GetPlan::Request &req, rtabmap_ros::GetPlan::Response &res)
  {
    Transform pose;
    if (req.goal_node <= 0)
    {
      pose = rtabmap_ros::transformFromPoseMsg(req.goal.pose, true);
    }
    UTimer timer;
    if (req.goal_node > 0 || !pose.isNull())
    {
      Transform coordinateTransform = Transform::getIdentity();
      // transform goal in /map frame
      if (!pose.isNull() && !req.goal.header.frame_id.empty() && mapFrameId_.compare(req.goal.header.frame_id) != 0)
      {
        coordinateTransform = rtabmap_ros::getTransform(mapFrameId_, req.goal.header.frame_id, req.goal.header.stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
        if (coordinateTransform.isNull())
        {
          NODELET_ERROR("Cannot transform goal pose from \"%s\" frame to \"%s\" frame!",
                        req.goal.header.frame_id.c_str(), mapFrameId_.c_str());
          return false;
        }
        if (!pose.isNull())
        {
          pose = coordinateTransform * pose;
        }
      }
      // else assume map frame if not set

      // To convert back the poses in goal frame
      coordinateTransform = coordinateTransform.inverse();

      if ((req.goal_node > 0 && rtabmap_.computePath(req.goal_node, req.tolerance)) ||
          (req.goal_node <= 0 && rtabmap_.computePath(pose, req.tolerance)))
      {
        NODELET_INFO("Planning: Time computing path = %f s", timer.ticks());
        const std::vector<std::pair<int, Transform>> &poses = rtabmap_.getPath();
        res.plan.header.frame_id = mapFrameId_;
        res.plan.header.stamp = req.goal_node > 0 ? ros::Time::now() : req.goal.header.stamp;
        if (poses.size() == 0)
        {
          NODELET_WARN("Planning: Goal already reached (RGBD/GoalReachedRadius=%fm).",
                       rtabmap_.getGoalReachedRadius());
          if (!pose.isNull())
          {
            // just set the goal directly
            res.plan.poses.resize(1);
            res.plan.nodeIds.resize(1);
            rtabmap_ros::transformToPoseMsg(coordinateTransform * pose, res.plan.poses[0]);
            res.plan.nodeIds[0] = 0;
          }
        }
        else
        {
          res.plan.poses.resize(poses.size());
          res.plan.nodeIds.resize(poses.size());
          int oi = 0;
          for (std::vector<std::pair<int, Transform>>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
          {
            rtabmap_ros::transformToPoseMsg(coordinateTransform * iter->second, res.plan.poses[oi]);
            res.plan.nodeIds[oi] = iter->first;
            ++oi;
          }
          if (!rtabmap_.getPathTransformToGoal().isIdentity())
          {
            res.plan.poses.resize(res.plan.poses.size() + 1);
            res.plan.nodeIds.resize(res.plan.nodeIds.size() + 1);
            Transform p = rtabmap_.getPath().back().second * rtabmap_.getPathTransformToGoal();
            rtabmap_ros::transformToPoseMsg(coordinateTransform * p, res.plan.poses[res.plan.poses.size() - 1]);
            res.plan.nodeIds[res.plan.nodeIds.size() - 1] = 0;
          }

          // Just output the path on screen
          std::stringstream stream;
          for (std::vector<std::pair<int, Transform>>::const_iterator iter = poses.begin(); iter != poses.end(); ++iter)
          {
            if (iter != poses.begin())
            {
              stream << " ";
            }
            stream << iter->first;
          }
          NODELET_INFO("Planned path: [%s]", stream.str().c_str());
        }
      }
      rtabmap_.clearPath(0);
    }
    return true;
  }

  bool CoreWrapper::setGoalCallback(rtabmap_ros::SetGoal::Request &req, rtabmap_ros::SetGoal::Response &res)
  {
    double planningTime = 0.0;
    goalCommonCallback(req.node_id, req.node_label, req.frame_id, Transform(), ros::Time::now(), &planningTime);
    const std::vector<std::pair<int, Transform>> &path = rtabmap_.getPath();
    res.path_ids.resize(path.size());
    res.path_poses.resize(path.size());
    res.planning_time = planningTime;
    for (unsigned int i = 0; i < path.size(); ++i)
    {
      res.path_ids[i] = path[i].first;
      rtabmap_ros::transformToPoseMsg(path[i].second, res.path_poses[i]);
    }
    return true;
  }

  bool CoreWrapper::cancelGoalCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
  {
    if (rtabmap_.getPath().size())
    {
      NODELET_WARN("Goal cancelled!");
      rtabmap_.clearPath(0);
      currentMetricGoal_.setNull();
      lastPublishedMetricGoal_.setNull();
      goalFrameId_.clear();
      latestNodeWasReached_ = false;
      if (goalReachedPub_.getNumSubscribers())
      {
        std_msgs::Bool result;
        result.data = false;
        goalReachedPub_.publish(result);
      }
    }
    if (mbClient_ && mbClient_->isServerConnected())
    {
      mbClient_->cancelGoal();
    }

    return true;
  }

  bool CoreWrapper::setLabelCallback(rtabmap_ros::SetLabel::Request &req, rtabmap_ros::SetLabel::Response &res)
  {
    if (rtabmap_.labelLocation(req.node_id, req.node_label))
    {
      if (req.node_id > 0)
      {
        NODELET_INFO("Set label \"%s\" to node %d", req.node_label.c_str(), req.node_id);
      }
      else
      {
        NODELET_INFO("Set label \"%s\" to last node", req.node_label.c_str());
      }
    }
    else
    {
      if (req.node_id > 0)
      {
        NODELET_ERROR("Could not set label \"%s\" to node %d", req.node_label.c_str(), req.node_id);
      }
      else
      {
        NODELET_ERROR("Could not set label \"%s\" to last node", req.node_label.c_str());
      }
    }
    return true;
  }

  bool CoreWrapper::listLabelsCallback(rtabmap_ros::ListLabels::Request &req, rtabmap_ros::ListLabels::Response &res)
  {
    if (rtabmap_.getMemory())
    {
      std::map<int, std::string> labels = rtabmap_.getMemory()->getAllLabels();
      res.ids = uKeys(labels);
      res.labels = uValues(labels);
      NODELET_INFO("List labels service: %d labels found.", (int)res.labels.size());
    }
    return true;
  }

  bool CoreWrapper::removeLabelCallback(rtabmap_ros::RemoveLabel::Request &req, rtabmap_ros::RemoveLabel::Response &res)
  {
    if (rtabmap_.getMemory())
    {
      int id = rtabmap_.getMemory()->getSignatureIdByLabel(req.label, true);
      if (id == 0)
      {
        NODELET_WARN("Label \"%s\" not found in the map, cannot remove it!", req.label.c_str());
      }
      else if (!rtabmap_.labelLocation(id, ""))
      {
        NODELET_ERROR("Failed removing label \"%s\".", req.label.c_str());
      }
      else
      {
        NODELET_INFO("Removed label \"%s\".", req.label.c_str());
      }
    }
    return true;
  }

  bool CoreWrapper::addLinkCallback(rtabmap_ros::AddLink::Request &req, rtabmap_ros::AddLink::Response &)
  {
    if (rtabmap_.getMemory())
    {
      ROS_INFO("Adding external link %d -> %d", req.link.fromId, req.link.toId);
      rtabmap_.addLink(linkFromROS(req.link));
      return true;
    }
    return false;
  }

  bool CoreWrapper::getNodesInRadiusCallback(rtabmap_ros::GetNodesInRadius::Request &req, rtabmap_ros::GetNodesInRadius::Response &res)
  {
    ROS_INFO("Get nodes in radius (%f): node_id=%d pose=(%f,%f,%f)", req.radius, req.node_id, req.x, req.y, req.z);
    std::map<int, Transform> poses;
    std::map<int, float> dists;
    if (req.node_id != 0 || (req.x == 0.0f && req.y == 0.0f && req.z == 0.0f))
    {
      poses = rtabmap_.getNodesInRadius(req.node_id, req.radius, req.k, &dists);
    }
    else
    {
      poses = rtabmap_.getNodesInRadius(Transform(req.x, req.y, req.z, 0, 0, 0), req.radius, req.k, &dists);
    }

    // Optimized graph
    res.ids.resize(poses.size());
    res.poses.resize(poses.size());
    res.distsSqr.resize(poses.size());
    int index = 0;
    for (std::map<int, rtabmap::Transform>::const_iterator iter = poses.begin();
         iter != poses.end();
         ++iter)
    {
      res.ids[index] = iter->first;
      transformToPoseMsg(iter->second, res.poses[index]);
      UASSERT(dists.find(iter->first) != dists.end());
      res.distsSqr[index] = dists.at(iter->first);
      ++index;
    }

    return true;
  }

  void CoreWrapper::publishStats(const ros::Time &stamp)
  {
    UDEBUG("Publishing stats...");
    const rtabmap::Statistics &stats = rtabmap_.getStatistics();

    if (infoPub_.getNumSubscribers())
    {
      // NODELET_INFO("Sending RtabmapInfo msg (last_id=%d)...", stat.refImageId());
      rtabmap_ros::InfoPtr msg(new rtabmap_ros::Info);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;

      rtabmap_ros::infoToROS(stats, *msg);
      infoPub_.publish(msg);
    }

    if (mapDataPub_.getNumSubscribers())
    {
      rtabmap_ros::MapDataPtr msg(new rtabmap_ros::MapData);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;

      std::map<int, Signature> signatures;
      if (stats.getLastSignatureData().id() > 0)
      {
        signatures.insert(std::make_pair(stats.getLastSignatureData().id(), stats.getLastSignatureData()));
      }

      if (nodesToRepublish_.size() && !rtabmap_.getLastLocalizationPose().isNull())
      {
        // Republish data from closest nodes of the current localization
        std::map<int, Transform> nodesOnly(rtabmap_.getLocalOptimizedPoses().lower_bound(1), rtabmap_.getLocalOptimizedPoses().end());
        int id = rtabmap::graph::findNearestNode(nodesOnly, rtabmap_.getLastLocalizationPose());
        if (id > 0)
        {
          std::map<int, int> ids = rtabmap_.getMemory()->getNeighborsId(id, 0, 0, false, false, true);
          std::map<int, int> missingIds;
          for (std::map<int, int>::iterator iter = ids.begin(); iter != ids.end(); ++iter)
          {
            if (nodesToRepublish_.find(iter->first) != nodesToRepublish_.end())
            {
              missingIds.insert(std::make_pair(iter->second, iter->first));
            }
          }

          if (nodesToRepublish_.size() != missingIds.size())
          {
            // remove requested nodes not anymore in the graph
            for (std::set<int>::iterator iter = nodesToRepublish_.begin(); iter != nodesToRepublish_.end();)
            {
              if (ids.find(*iter) == ids.end())
              {
                iter = nodesToRepublish_.erase(iter);
              }
              else
              {
                ++iter;
              }
            }
          }

          int loaded = 0;
          std::stringstream stream;
          for (std::map<int, int>::iterator iter = missingIds.begin(); iter != missingIds.end() && loaded < maxNodesRepublished_; ++iter)
          {
            signatures.insert(std::make_pair(iter->second, rtabmap_.getMemory()->getNodeData(iter->second, true, true, true, true)));
            nodesToRepublish_.erase(iter->second);
            ++loaded;
            stream << iter->second << " ";
          }
          if (loaded)
          {
            NODELET_WARN("Republishing data of requested node(s) %sfrom \"%s\" input topic (max_nodes_republished=%d)",
                         stream.str().c_str(),
                         republishNodeDataSub_.getTopic().c_str(),
                         maxNodesRepublished_);
          }
        }
      }
      rtabmap_ros::mapDataToROS(
          stats.poses(),
          stats.constraints(),
          signatures,
          stats.mapCorrection(),
          *msg);

      mapDataPub_.publish(msg);
    }

    if (mapGraphPub_.getNumSubscribers())
    {
      rtabmap_ros::MapGraphPtr msg(new rtabmap_ros::MapGraph);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;

      rtabmap_ros::mapGraphToROS(
          stats.poses(),
          stats.constraints(),
          stats.mapCorrection(),
          *msg);

      mapGraphPub_.publish(msg);
    }

    if (odomCachePub_.getNumSubscribers())
    {
      rtabmap_ros::MapGraphPtr msg(new rtabmap_ros::MapGraph);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;

      // For visualization of the constraints (MapGraph rviz plugin), we should include target nodes from the map
      std::map<int, Transform> poses = stats.odomCachePoses();
      // transform in map frame
      for (std::map<int, Transform>::iterator iter = poses.begin();
           iter != poses.end();
           ++iter)
      {
        iter->second = stats.mapCorrection() * iter->second;
      }
      for (std::multimap<int, rtabmap::Link>::const_iterator iter = stats.odomCacheConstraints().begin();
           iter != stats.odomCacheConstraints().end();
           ++iter)
      {
        std::map<int, Transform>::const_iterator pter = stats.poses().find(iter->second.to());
        if (pter != stats.poses().end())
        {
          poses.insert(*pter);
        }
      }
      rtabmap_ros::mapGraphToROS(
          poses,
          stats.odomCacheConstraints(),
          stats.mapCorrection(),
          *msg);

      odomCachePub_.publish(msg);
    }

    if (localGridObstacle_.getNumSubscribers() && !stats.getLastSignatureData().sensorData().gridObstacleCellsRaw().empty())
    {
      pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridObstacleCellsRaw()));
      sensor_msgs::PointCloud2 msg;
      pcl_conversions::moveFromPCL(*cloud, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = frameId_;
      localGridObstacle_.publish(msg);
    }
    if (localGridEmpty_.getNumSubscribers() && !stats.getLastSignatureData().sensorData().gridEmptyCellsRaw().empty())
    {
      pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridEmptyCellsRaw()));
      sensor_msgs::PointCloud2 msg;
      pcl_conversions::moveFromPCL(*cloud, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = frameId_;
      localGridEmpty_.publish(msg);
    }
    if (localGridGround_.getNumSubscribers() && !stats.getLastSignatureData().sensorData().gridGroundCellsRaw().empty())
    {
      pcl::PCLPointCloud2::Ptr cloud = rtabmap::util3d::laserScanToPointCloud2(LaserScan::backwardCompatibility(stats.getLastSignatureData().sensorData().gridGroundCellsRaw()));
      sensor_msgs::PointCloud2 msg;
      pcl_conversions::moveFromPCL(*cloud, msg);
      msg.header.stamp = stamp;
      msg.header.frame_id = frameId_;
      localGridGround_.publish(msg);
    }

    bool pubLabels = labelsPub_.getNumSubscribers();
    visualization_msgs::MarkerArray markers;
    if ((landmarksPub_.getNumSubscribers() || pubLabels) && !stats.poses().empty() && stats.poses().begin()->first < 0)
    {
      geometry_msgs::PoseArrayPtr msg(new geometry_msgs::PoseArray);
      msg->header.stamp = stamp;
      msg->header.frame_id = mapFrameId_;
      for (std::map<int, Transform>::const_iterator iter = stats.poses().begin(); iter != stats.poses().end() && iter->first < 0; ++iter)
      {
        geometry_msgs::Pose p;
        rtabmap_ros::transformToPoseMsg(iter->second, p);
        msg->poses.push_back(p);

        if (pubLabels)
        {
          // Add landmark ids
          visualization_msgs::Marker marker;
          marker.header.frame_id = mapFrameId_;
          marker.header.stamp = stamp;
          marker.ns = "landmarks";
          marker.id = iter->first;
          marker.action = visualization_msgs::Marker::ADD;
          marker.pose.position.x = iter->second.x();
          marker.pose.position.y = iter->second.y();
          marker.pose.position.z = iter->second.z();
          marker.pose.orientation.x = 0.0;
          marker.pose.orientation.y = 0.0;
          marker.pose.orientation.z = 0.0;
          marker.pose.orientation.w = 1.0;
          marker.scale.x = 1;
          marker.scale.y = 1;
          marker.scale.z = 0.35;
          marker.color.a = 0.7;
          marker.color.r = 0.0;
          marker.color.g = 1.0;
          marker.color.b = 0.0;
          marker.lifetime = ros::Duration(2.0f / rate_);

          marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
          marker.text = uNumber2Str(iter->first);

          markers.markers.push_back(marker);
        }
      }

      landmarksPub_.publish(msg);
    }

    bool pubPath = mapPathPub_.getNumSubscribers();
    if (pubLabels || pubPath)
    {
      if (stats.poses().size())
      {
        nav_msgs::Path path;
        if (pubPath)
        {
          path.poses.resize(stats.poses().size());
        }
        int oi = 0;
        for (std::map<int, Transform>::const_iterator poseIter = stats.poses().begin();
             poseIter != stats.poses().end();
             ++poseIter)
        {
          if (pubLabels && rtabmap_.getMemory())
          {
            // Add labels
            std::map<int, std::string>::const_iterator lter = rtabmap_.getMemory()->getAllLabels().find(poseIter->first);
            if (lter != rtabmap_.getMemory()->getAllLabels().end() && !lter->second.empty())
            {
              visualization_msgs::Marker marker;
              marker.header.frame_id = mapFrameId_;
              marker.header.stamp = stamp;
              marker.ns = "labels";
              marker.id = -poseIter->first;
              marker.action = visualization_msgs::Marker::ADD;
              marker.pose.position.x = poseIter->second.x();
              marker.pose.position.y = poseIter->second.y();
              marker.pose.position.z = poseIter->second.z();
              marker.pose.orientation.x = 0.0;
              marker.pose.orientation.y = 0.0;
              marker.pose.orientation.z = 0.0;
              marker.pose.orientation.w = 1.0;
              marker.scale.x = 1;
              marker.scale.y = 1;
              marker.scale.z = 0.5;
              marker.color.a = 0.7;
              marker.color.r = 1.0;
              marker.color.g = 0.0;
              marker.color.b = 0.0;

              marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
              marker.text = lter->second;

              markers.markers.push_back(marker);
            }

            // Add node ids
            visualization_msgs::Marker marker;
            marker.header.frame_id = mapFrameId_;
            marker.header.stamp = stamp;
            marker.ns = "ids";
            marker.id = poseIter->first;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = poseIter->second.x();
            marker.pose.position.y = poseIter->second.y();
            marker.pose.position.z = poseIter->second.z();
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = 1;
            marker.scale.y = 1;
            marker.scale.z = 0.2;
            marker.color.a = 0.5;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.lifetime = ros::Duration(2.0f / rate_);

            marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.text = uNumber2Str(poseIter->first);

            markers.markers.push_back(marker);
          }
          if (pubPath)
          {
            rtabmap_ros::transformToPoseMsg(poseIter->second, path.poses.at(oi).pose);
            path.poses.at(oi).header.frame_id = mapFrameId_;
            path.poses.at(oi).header.stamp = stamp;
            ++oi;
          }
        }

        if (pubLabels && markers.markers.size())
        {
          labelsPub_.publish(markers);
        }
        if (pubPath && oi)
        {
          path.header.frame_id = mapFrameId_;
          path.header.stamp = stamp;
          path.poses.resize(oi);
          mapPathPub_.publish(path);
        }
      }
    }
  }

  void CoreWrapper::publishCurrentGoal(const ros::Time &stamp)
  {
    if (!currentMetricGoal_.isNull() && currentMetricGoal_ != lastPublishedMetricGoal_)
    {
      NODELET_INFO("Publishing next goal: %d -> %s",
                   rtabmap_.getPathCurrentGoalId(), currentMetricGoal_.prettyPrint().c_str());

      geometry_msgs::PoseStamped poseMsg;
      poseMsg.header.frame_id = mapFrameId_;
      poseMsg.header.stamp = stamp;
      rtabmap_ros::transformToPoseMsg(currentMetricGoal_, poseMsg.pose);

      if (useActionForGoal_)
      {
        if (mbClient_ == 0 || !mbClient_->isServerConnected())
        {
          NODELET_INFO("Connecting to move_base action server...");
          if (mbClient_ == 0)
          {
            mbClient_ = new MoveBaseClient("move_base", true);
          }
          mbClient_->waitForServer(ros::Duration(5.0));
        }
        if (mbClient_ && mbClient_->isServerConnected())
        {
          move_base_msgs::MoveBaseGoal goal;
          goal.target_pose = poseMsg;

          mbClient_->sendGoal(goal,
                              boost::bind(&CoreWrapper::goalDoneCb, this, _1, _2),
                              boost::bind(&CoreWrapper::goalActiveCb, this),
                              boost::bind(&CoreWrapper::goalFeedbackCb, this, _1));
          lastPublishedMetricGoal_ = currentMetricGoal_;
        }
        else
        {
          NODELET_ERROR("Cannot connect to move_base action server (called \"%s\")!", this->getNodeHandle().resolveName("move_base").c_str());
        }
      }
      if (nextMetricGoalPub_.getNumSubscribers())
      {
        nextMetricGoalPub_.publish(poseMsg);
        if (!useActionForGoal_)
        {
          lastPublishedMetricGoal_ = currentMetricGoal_;
        }
      }
    }
  }

  // Called once when the goal completes
  void CoreWrapper::goalDoneCb(const actionlib::SimpleClientGoalState &state,
                               const move_base_msgs::MoveBaseResultConstPtr &result)
  {
    bool ignore = false;
    if (!currentMetricGoal_.isNull())
    {
      if (state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        if (rtabmap_.getPath().size() &&
            rtabmap_.getPathCurrentGoalId() != rtabmap_.getPath().back().first &&
            (!uContains(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPath().back().first) || !latestNodeWasReached_))
        {
          NODELET_WARN("Planning: move_base reached current goal but it is not "
                       "the last one planned by rtabmap. A new goal should be sent when "
                       "rtabmap will be able to retrieve next locations on the path.");
          ignore = true;
        }
        else
        {
          NODELET_INFO("Planning: move_base success!");
        }
      }
      else
      {
        NODELET_ERROR("Planning: move_base failed for some reason. Aborting the plan...");
      }

      if (!ignore && goalReachedPub_.getNumSubscribers())
      {
        std_msgs::Bool result;
        result.data = state == actionlib::SimpleClientGoalState::SUCCEEDED;
        goalReachedPub_.publish(result);
      }
    }

    if (!ignore)
    {
      rtabmap_.clearPath(1);
      currentMetricGoal_.setNull();
      lastPublishedMetricGoal_.setNull();
      goalFrameId_.clear();
      latestNodeWasReached_ = false;
    }
  }

  // Called once when the goal becomes active
  void CoreWrapper::goalActiveCb()
  {
    // NODELET_INFO("Planning: Goal just went active");
  }

  // Called every time feedback is received for the goal
  void CoreWrapper::goalFeedbackCb(const move_base_msgs::MoveBaseFeedbackConstPtr &feedback)
  {
    // Transform basePosition = rtabmap_ros::transformFromPoseMsg(feedback->base_position.pose);
    // NODELET_INFO("Planning: feedback base_position = %s", basePosition.prettyPrint().c_str());
  }

  void CoreWrapper::publishLocalPath(const ros::Time &stamp)
  {
    if (rtabmap_.getPath().size())
    {
      std::vector<std::pair<int, Transform>> poses = rtabmap_.getPathNextPoses();
      if (poses.size())
      {
        if (localPathPub_.getNumSubscribers() || localPathNodesPub_.getNumSubscribers())
        {
          nav_msgs::Path path;
          rtabmap_ros::Path pathNodes;
          path.header.frame_id = pathNodes.header.frame_id = mapFrameId_;
          path.header.stamp = pathNodes.header.stamp = stamp;
          path.poses.resize(poses.size());
          pathNodes.nodeIds.resize(poses.size());
          pathNodes.poses.resize(poses.size());
          int oi = 0;
          for (std::vector<std::pair<int, Transform>>::iterator iter = poses.begin(); iter != poses.end(); ++iter)
          {
            path.poses[oi].header = path.header;
            rtabmap_ros::transformToPoseMsg(iter->second, path.poses[oi].pose);
            pathNodes.poses[oi] = path.poses[oi].pose;
            pathNodes.nodeIds[oi] = iter->first;
            ++oi;
          }
          if (localPathPub_.getNumSubscribers())
          {
            localPathPub_.publish(path);
          }
          if (localPathNodesPub_.getNumSubscribers())
          {
            localPathNodesPub_.publish(pathNodes);
          }
        }
      }
    }
  }

  void CoreWrapper::publishGlobalPath(const ros::Time &stamp)
  {
    if ((globalPathPub_.getNumSubscribers() || globalPathNodesPub_.getNumSubscribers()) && rtabmap_.getPath().size())
    {
      Transform pose = uValue(rtabmap_.getLocalOptimizedPoses(), rtabmap_.getPathCurrentGoalId(), Transform());
      if (!pose.isNull() && rtabmap_.getPathCurrentGoalIndex() < rtabmap_.getPath().size())
      {
        // transform the global path in the goal referential
        Transform t = pose * rtabmap_.getPath().at(rtabmap_.getPathCurrentGoalIndex()).second.inverse();

        nav_msgs::Path path;
        rtabmap_ros::Path pathNodes;
        path.header.frame_id = pathNodes.header.frame_id = mapFrameId_;
        path.header.stamp = pathNodes.header.stamp = stamp;
        path.poses.resize(rtabmap_.getPath().size());
        pathNodes.nodeIds.resize(rtabmap_.getPath().size());
        pathNodes.poses.resize(rtabmap_.getPath().size());
        int oi = 0;
        for (std::vector<std::pair<int, Transform>>::const_iterator iter = rtabmap_.getPath().begin(); iter != rtabmap_.getPath().end(); ++iter)
        {
          path.poses[oi].header = path.header;
          rtabmap_ros::transformToPoseMsg(t * iter->second, path.poses[oi].pose);
          pathNodes.poses[oi] = path.poses[oi].pose;
          pathNodes.nodeIds[oi] = iter->first;
          ++oi;
        }
        Transform goalLocalTransform = Transform::getIdentity();
        if (!goalFrameId_.empty() && goalFrameId_.compare(frameId_) != 0)
        {
          Transform localT = rtabmap_ros::getTransform(frameId_, goalFrameId_, stamp, tfListener_, waitForTransform_ ? waitForTransformDuration_ : 0.0);
          if (!localT.isNull())
          {
            goalLocalTransform = localT.inverse().to3DoF();
          }
        }

        if (!rtabmap_.getPathTransformToGoal().isIdentity() || !goalLocalTransform.isIdentity())
        {
          path.poses.resize(path.poses.size() + 1);
          path.poses[path.poses.size() - 1].header = path.header;
          pathNodes.nodeIds.resize(pathNodes.nodeIds.size() + 1);
          pathNodes.poses.resize(pathNodes.poses.size() + 1);
          Transform p = t * rtabmap_.getPath().back().second * rtabmap_.getPathTransformToGoal() * goalLocalTransform;
          rtabmap_ros::transformToPoseMsg(p, path.poses[path.poses.size() - 1].pose);
          pathNodes.poses[pathNodes.poses.size() - 1] = path.poses[path.poses.size() - 1].pose;
          pathNodes.nodeIds[pathNodes.nodeIds.size() - 1] = 0;
        }
        if (globalPathPub_.getNumSubscribers())
        {
          globalPathPub_.publish(path);
        }
        if (globalPathNodesPub_.getNumSubscribers())
        {
          globalPathNodesPub_.publish(pathNodes);
        }
      }
    }
  }

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP

  bool CoreWrapper::octomapBinaryCallback(
      octomap_msgs::GetOctomap::Request &req,
      octomap_msgs::GetOctomap::Response &res)
  {
    NODELET_INFO("Sending binary map data on service request");
    res.map.header.frame_id = mapFrameId_;
    res.map.header.stamp = ros::Time::now();

    rtabmap_mtx_.lock();
    std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
    if ((mappingMaxNodes_ > 0 || mappingAltitudeDelta_ > 0.0) && poses.size() > 1)
    {
      poses = filterNodesToAssemble(poses, poses.rbegin()->second);
    }

    rtabmap_mtx_.unlock();
    rtabmap::SemanticOctoMap::AuxSignatureData auxSignatureData;
    poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true, rtabmap_mtx_, auxSignatureData);

    const rtabmap::OctoMap *octomap = mapsManager_.getOctomap();
    bool success = octomap->octree()->size() && octomap_msgs::binaryMapToMsg(*octomap->octree(), res.map);
    return success;
  }

  bool CoreWrapper::octomapFullCallback(
      octomap_msgs::GetOctomap::Request &req,
      octomap_msgs::GetOctomap::Response &res)
  {
    NODELET_INFO("Sending full map data on service request");
    res.map.header.frame_id = mapFrameId_;
    res.map.header.stamp = ros::Time::now();

    rtabmap_mtx_.lock();
    std::map<int, Transform> poses = rtabmap_.getLocalOptimizedPoses();
    if ((mappingMaxNodes_ > 0 || mappingAltitudeDelta_ > 0.0) && poses.size() > 1)
    {
      poses = filterNodesToAssemble(poses, poses.rbegin()->second);
    }

    rtabmap_mtx_.unlock();
    rtabmap::SemanticOctoMap::AuxSignatureData auxSignatureData;
    poses = mapsManager_.updateMapCaches(poses, rtabmap_.getMemory(), false, true, rtabmap_mtx_, auxSignatureData);

    const rtabmap::OctoMap *octomap = mapsManager_.getOctomap();
    bool success = octomap->octree()->size() && octomap_msgs::fullMapToMsg(*octomap->octree(), res.map);
    return success;
  }

#endif
#endif

  // JHUAPL section

  void CoreWrapper::publishVisualDepthImages(const rtabmap::SensorData &data)
  {
    if (visualImagePub_.getNumSubscribers() || depthImagePub_.getNumSubscribers())
    {
      cv::Mat rgb, depth;
      cv::Mat semanticMask;
      if (!data.imageRaw().empty() && !data.depthOrRightRaw().empty())
      {
        rgb = data.imageRaw();
        depth = data.depthOrRightRaw();
      }
      else
      {
        data.uncompressDataConst(&rgb, &depth, &semanticMask);
      }

      if (visualImagePub_.getNumSubscribers())
      {
        if (rgb.empty())
        {
          ROS_ERROR("rgb image is empty!! ");
        }

        cv::Mat img_rgb;
        cv::cvtColor(rgb, img_rgb, cv::COLOR_RGB2BGR);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img_rgb).toImageMsg();
        visualImagePub_.publish(msg);
      }

      if (depthImagePub_.getNumSubscribers())
      {
        if (depth.empty())
        {
          ROS_ERROR("depth image is empty!! ");
        }

        cv::Mat img_depth;
        double min;
        double max;
        cv::minMaxIdx(depth, &min, &max);
        cv::convertScaleAbs(depth, img_depth, 255 / max);

        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img_depth).toImageMsg();
        depthImagePub_.publish(msg);
      }
    }
  }

  void CoreWrapper::rtabmapProcessThread()
  {
    UTimer timer;
    double timeRtabmap = 0.0;
    double timeUpdateMaps = 0.0;

    ROS_INFO("Starting RTabMap Process Thread");

    boost::shared_ptr<PreProcessSignatureDataType> preProcessedData;

    while (rtabmapProcessThreadRunning_)
    {
      // mutex to sync with map manager thread
      rtabmap_mtx_.lock();
      bool rtabmapProcessed = rtabmap_.process_threaded(preProcessedData);
      rtabmap_mtx_.unlock();

      if (rtabmapProcessed)
      {
        // get last processed data
        ros::Time stamp = ros::Time(preProcessedData->stamp);
        Transform &odom = *preProcessedData->pose;
        std::string &odomFrameId = preProcessedData->odomFrameId;
        SensorData &data = *preProcessedData->data;

        timeRtabmap = timer.ticks();
        mapToOdomMutex_.lock();
        mapToOdom_ = rtabmap_.getMapCorrection();
        odomFrameId_ = odomFrameId;

        rtabmap::Transform mapToOdom = mapToOdom_;
        mapToOdomMutex_.unlock();

        rtabmap::Transform baseToMap = rtabmap::Transform::getIdentity();

        if (data.id() < 0)
        {
          NODELET_INFO("Intermediate node added");
        }
        else
        {
          // Publish local graph, info
          this->publishStats(stamp);
          // JHUAPL modification: pubs pose on every iteration
          // publish localization with correction regardless whether covariance is available
          if (localizationPosePub_.getNumSubscribers())
          {
            geometry_msgs::PoseWithCovarianceStamped poseMsg;
            poseMsg.header.frame_id = mapFrameId_;
            poseMsg.header.stamp = stamp;

            poseMsg.pose.covariance;
            if (!rtabmap_.getStatistics().localizationCovariance().empty())
            {
              baseToMap = mapToOdom * odom;
              rtabmap_ros::transformToPoseMsg(baseToMap, poseMsg.pose.pose);
              const cv::Mat &cov = rtabmap_.getStatistics().localizationCovariance();
              memcpy(poseMsg.pose.covariance.data(), cov.data, cov.total() * sizeof(double));
              mapToOdomPrev_ = mapToOdom;
            }
            else
            {
              baseToMap = mapToOdomPrev_ * odom;
              rtabmap_ros::transformToPoseMsg(baseToMap, poseMsg.pose.pose);
            }
            localizationPosePub_.publish(poseMsg);
          }
          std::map<int, rtabmap::Transform> filteredPoses(rtabmap_.getLocalOptimizedPoses().lower_bound(1), rtabmap_.getLocalOptimizedPoses().end());

          // create a tmp signature with latest sensory data if latest signature was ignored
          std::map<int, rtabmap::Signature> tmpSignature;
          if (rtabmap_.getMemory() == 0)
          {
            UDEBUG(" --> tmp signature mem == 0; latest data from input");
            SensorData tmpData = data;
            tmpData.setId(0);
            tmpSignature.insert(std::make_pair(0, Signature(0, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
            filteredPoses.insert(std::make_pair(0, mapToOdom * odom));
          }
          /// JHUAPL section
          else if (filteredPoses.size() == 0 ||
                   rtabmap_.getMemory()->getLastSignatureId() != filteredPoses.rbegin()->first ||
                   rtabmap_.getMemory()->getLastWorkingSignature() == 0 ||
                   rtabmap_.getMemory()->getLastWorkingSignature()->sensorData().gridCellSize() == 0 ||
                   mapsManager_.getIsAlwaysUpdateMap() ||
                   (!mapsManager_.getOccupancyGrid()->isGridFromDepth() && data.laserScanRaw().is2d())) // 2d laser scan would fill empty space for latest data
          {
            UDEBUG(" ----------> incoming node id(input cloud): %d (now id zero)", rtabmap_.getMemory()->getLastAddedData().id());
            SensorData tmpData = rtabmap_.getMemory()->getLastAddedData();
            tmpData.setId(0);
            tmpSignature.insert(std::make_pair(0, Signature(0, -1, 0, data.stamp(), "", odom, Transform(), tmpData)));
            filteredPoses.insert(std::make_pair(0, mapToOdom * odom));
          }
          /// JHUAPL section end

          if ((mappingMaxNodes_ > 0 || mappingAltitudeDelta_ > 0.0) && filteredPoses.size() > 1)
          {
            std::map<int, Transform> nearestPoses;
            nearestPoses = filterNodesToAssemble(filteredPoses, mapToOdom * odom);

            // add latest/zero and make sure those on a planned path are not filtered
            std::set<int> onPath;
            if (rtabmap_.getPath().size())
            {
              std::vector<int> nextNodes = rtabmap_.getPathNextNodes();
              onPath.insert(nextNodes.begin(), nextNodes.end());
            }
            for (std::map<int, Transform>::iterator iter = filteredPoses.begin(); iter != filteredPoses.end(); ++iter)
            {
              if (iter->first == 0 || onPath.find(iter->first) != onPath.end())
              {
                nearestPoses.insert(*iter);
              }
              else if (onPath.empty())
              {
                break;
              }
            }

            filteredPoses = nearestPoses;
          }

          mmu_data_mtx_.lock();
          filteredPoses_ = filteredPoses;
          tmpSignature_ = tmpSignature;
          stamp_ = stamp;
          // last_baseToMap_ : last T_map_pose or T_map_baselink : reads from baselink to map frame
          last_baseToMap_ = baseToMap;
          mmu_data_mtx_.unlock();

          this->publishLandmarksMap(rtabmap_.getMemory(), mapFrameId_);

          timeUpdateMaps = timer.ticks();
        }

        // NODELET_INFO("rtabmap (%d): Rate=%.2fs, Limit=%.3fs, Conversion=%.4fs, RTAB-Map=%.4fs, pub landmarks=%.4fs total=%.4fs (local map=%d, WM=%d)",
        NODELET_INFO("rtabmap (%d): Rate=%.2fs, Limit=%.3fs, RTAB-Map=%.4fs, pub landmarks=%.4fs total=%.4fs (local map=%d, WM=%d)",
                     rtabmap_.getLastLocationId(),
                     rate_ > 0 ? 1.0f / rate_ : 0,
                     rtabmap_.getTimeThreshold() / 1000.0f,
                     // timeMsgConversion,
                     timeRtabmap,
                     timeUpdateMaps,
                     timeRtabmap + timeUpdateMaps,
                     (int)rtabmap_.getLocalOptimizedPoses().size(),
                     rtabmap_.getWMSize() + rtabmap_.getSTMSize());
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/HasSubscribers/"), mapsManager_.hasSubscribers() ? 1 : 0));
        // rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeMsgConversion/ms"), timeMsgConversion * 1000.0f));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeRtabmap/ms"), timeRtabmap * 1000.0f));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeUpdatingMaps/ms"), timeUpdateMaps * 1000.0f));
        // rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimePublishing/ms"), timePublishMaps*1000.0f));
        rtabmapROSStats_.insert(std::make_pair(std::string("RtabmapROS/TimeTotal/ms"), (timeRtabmap + timeUpdateMaps) * 1000.0f));
      }
    }
  }

  void CoreWrapper::mapManagerUpdateThread(const double &threadDelay)
  {
    if (threadDelay == 0)
      return;
    ros::Rate rate(1.0 / threadDelay);
    NODELET_INFO(" map manager update thread, expected update rate: %fsecs", rate.expectedCycleTime().toSec());
    boost::thread publishMapThread;

    // wait for the map manager to be initialized
    while (!mapManagerInitialized_)
    {
      NODELET_WARN(" Waiting for Map Manager Initialization");
      rate.sleep();
    }

    ros::Time startTime;

    while (mapManagerUpdateThreadRunning_)
    {
      startTime = ros::Time::now();
      rtabmap_ros::MapManagerStats mapManagerStats;

      std::map<int, rtabmap::Transform> filteredPoses;
      std::map<int, rtabmap::Signature> tmpSignature;
      ros::Time stamp;
      rtabmap::Transform g_map_pose;

      // grab latest signature pose estimates
      mmu_data_mtx_.lock();
      filteredPoses = filteredPoses_;
      tmpSignature = tmpSignature_;
      stamp = stamp_;
      // last_baseToMap_ : last T_map_pose or T_map_base : reads from baselink to map frame
      g_map_pose = last_baseToMap_;
      mmu_data_mtx_.unlock();

      if (!filteredPoses.empty())
      {
      #ifndef RTABMAP_OCTOMAP
        // built without Octomap library
        filteredPoses = mapsManager_.updateMapCaches(
            filteredPoses,
            rtabmap_.getMemory(),
            false,
            false,
            rtabmap_mtx_,
            tmpSignature,
            &mapManagerStats);
      #else

        mapsManager_.updateTransformMapPose(g_map_pose);

        rtabmap::SemanticOctoMap::AuxSignatureData auxSignatureData;
        filteredPoses = mapsManager_.updateMapCaches(
            filteredPoses,
            rtabmap_.getMemory(),
            false,
            false,
            rtabmap_mtx_,
            auxSignatureData,
            tmpSignature,
            &mapManagerStats);

        // updates map to the data base
        if (auxSignatureData.emptyPoints.size() > 0 || auxSignatureData.groundReferences.size() > 0)
        {
          mapsManager_.semanticOctomapStoreData(auxSignatureData, rtabmap_.getMemory(), rtabmap_mtx_);
        }

        // publish objects of interest visible state from map
        mapsManager_.publishTrackedVisibilityPts();

      #endif

        int filteredPoses_size = filteredPoses.size();   // need this due to std::move call below

        pflag_mtx_.lock();
        bool publishMapThreadRunning = publishMapThreadRunning_;
        pflag_mtx_.unlock();

        if (!publishMapThreadRunning)
        {
          // the thread has finished, creating a new one. no need to lock for variable
          publishMapThreadRunning_ = true;
          publishMapThread = boost::thread(boost::bind(&CoreWrapper::publishMapThread, this, std::move(filteredPoses), g_map_pose, stamp, mapFrameId_));

          std::string threadId = boost::lexical_cast<std::string>(publishMapThread.get_id());
          UDEBUG(" created new thread for publishing map! (%s)", threadId.c_str());
        }
        
        // update stats for ros message
        mapManagerStats.map_manager_update_cache_time = (ros::Time::now() - startTime).toSec();
        mapManagerStats.header.stamp = ros::Time::now();

        // publish stats when there are subscribers.
        if (mapManagerStatsPub_.getNumSubscribers())
        {
          mapManagerStatsPub_.publish(mapManagerStats);
        }

        double total_elapsed = (ros::Time::now() - startTime).toSec();
        NODELET_INFO("****  Map manager update : filteredPoses size= %d; runtime= %.4lf sec", filteredPoses_size, total_elapsed);
      }
      rate.sleep();
    }

    publishMapThread.interrupt();
    // exiting thread - wait for publish map thread to finish before ending this thread.
    publishMapThread.join();
  }

  void CoreWrapper::publishMapThread(const std::map<int, rtabmap::Transform> filteredPoses,
        const rtabmap::Transform &baseToMap, 
        const ros::Time &stamp, 
        const std::string &mapFrameId)
  {
    try
    {
      ros::Time startTime = ros::Time::now();

      ros::Time timeStamp = stamp;
      rtabmap::Transform pose = baseToMap;

      if (!mapsManager_.isSemanticSegmentationEnabled())
      {
        mapsManager_.publishMaps(filteredPoses, timeStamp, mapFrameId);
      }
      else
      {
        // publish maps in semantic segmentation mode
        mapsManager_.publishAPLMaps(pose, timeStamp, mapFrameId);
      }

      double total_elapsed = (ros::Time::now() - startTime).toSec();
      NODELET_INFO("       publishing map took %.4lf secs", total_elapsed);
    }
    catch (boost::thread_interrupted &)
    {
    }

    pflag_mtx_.lock();
    publishMapThreadRunning_ = false;
    pflag_mtx_.unlock();
  }

  bool CoreWrapper::semanticDataAssociationSrvCallback(rtabmap_ros::SemanticDataAssociation::Request &req, rtabmap_ros::SemanticDataAssociation::Response &res)
  {
    // Class ID is the semantic label id, corresponds to a supported classed detected in the semantic segmentation model.
    std::map<unsigned int, std::string> classId2StringMap = mapsManager_.getClassIdAssociation();

    // copy the data association into the ROS message.
    for (auto iter = classId2StringMap.begin(); iter != classId2StringMap.end(); ++iter)
    {
      int classId = iter->first;
      std::string className = iter->second;

      res.ClassId.push_back(classId);
      res.classNameString.push_back(className);
    }

    // Class ID to Occupancy name association
    std::map<unsigned int, std::string> classId2occupancyMap = mapsManager_.getOccupancyAssociation();

    // copy the class id to occupancy name association into a ROS service message.
    for (auto iter = classId2occupancyMap.begin(); iter != classId2occupancyMap.end(); ++iter)
    {
      int classId = iter->first;
      std::string occupancy = iter->second;

      res.occupancyClassId.push_back(classId);
      res.occupancyNameString.push_back(occupancy);
    }

    return true;
  }

  void CoreWrapper::publishObstaclesData(const rtabmap::SensorData &data)
  {
    if (obstaclesDataPub_.getNumSubscribers())
    {
      UTimer timer;
      timer.start();

      rtabmap_ros::ObstaclesData msg;
      msg.header.seq = ++obstacleDataSeq;
      msg.header.stamp = ros::Time(data.stamp());
      msg.header.frame_id = frameId_;
      msg.signatureId = data.id();

      // localTransform is the transformation from camera coordinates
      // to the base frame; we need the inverse to transform points back
      // to camera coordinates
      rtabmap::Transform localTransform = data.cameraModels().at(0).localTransform();
      transformToPoseMsg(localTransform.inverse(), msg.T_cam_pts);

      // obstaclesCellMap contains all class id detected in a keyframe, not all classes are actual obstacles.
      //  we are going to select the set of points belonging to actual obstacles and copying them into a ROS message.
      // actual obstacles are associated to occupancy types : static, movable, and dynamic
      std::map<unsigned int, std::string> occupancyAssociation = data.getOccupancyAssociation();
      std::map<unsigned int, cv::Mat> obstaclesCellMap = data.gridObstacleCellsMapRaw();
      for (auto iter = obstaclesCellMap.begin(); iter != obstaclesCellMap.end(); ++iter)
      {
        auto occupancyIter = occupancyAssociation.find(iter->first);
        if (occupancyIter != occupancyAssociation.end())
        {
          if (occupancyIter->second == "static" ||
              occupancyIter->second == "movable" ||
              occupancyIter->second == "dynamic")
          {
            unsigned int classId = iter->first;

            msg.obstacleIds.push_back(classId);

            cv_bridge::CvImagePtr cv_ptr(new cv_bridge::CvImage);

            // XYZRGB encapulation in 4D
            // [0] = X; [1] = Y, [2] = Z ; [3] = B | G  << 8 | R << 16
            if (iter->second.type() == CV_32FC(4))
            {
              cv_ptr->encoding = "32FC4";
            }
            cv_ptr->image = iter->second.clone();

            sensor_msgs::ImagePtr imgMsgPtr = cv_ptr->toImageMsg();
            // add point cloud encapsulated as an image
            msg.pointClouds.push_back(*imgMsgPtr);
          }
        }
      }
      NODELET_DEBUG(" publishObstaclesData copying msg time elapsed: %f secs", timer.ticks());
      timer.start();
      obstaclesDataPub_.publish(msg);
      NODELET_DEBUG(" publishObstaclesData sending msg time elapsed: %f secs", timer.ticks());
    }
  }

  bool CoreWrapper::baseToCamTransformSrvCallback(rtabmap_ros::BaseToCamTransform::Request &req, rtabmap_ros::BaseToCamTransform::Response &res)
  {
    camTobaseT_mtx_.lock();
    if (camTobaseTransform_.isIdentity() || camTobaseTransform_.isNull())
    {
      res.status = false;
      camTobaseT_mtx_.unlock();
      return true;
    }
    
    res.frame_id = odomFrameId_;
    res.child_frame_id = frameId_;
    // sending transformation from base frame to camera frame
    //
    // NOTE: a different naming convention is used for RTabmap vs ROS message variables
    //
    //       RTabmap:  AtoBTransform means:   Point_B = AtoBTransform * Point_A
    //       ROS Msg:  T_b_a means:           Point_b = T_b_a * Point_a
    //
    transformToGeometryMsg(camTobaseTransform_.inverse(), res.T_cam_base);

    camTobaseT_mtx_.unlock();

    res.status = true;

    return  true;
  }

  // void CoreWrapper::aliveEventCb(const ros::TimerEvent& event)
  // {
  //   std_msgs::Empty alive_msg;
  //   alive_publisher_.publish(alive_msg);
  // }

  void CoreWrapper::heartbeatEventThread()
  {
    float heartbeat_time_delay = 1.0;
    ros::Rate rate(1.0 / heartbeat_time_delay);

    while (ros::ok())
    {
      std_msgs::Empty alive_msg;
      alive_publisher_.publish(alive_msg);

      rate.sleep();
    }
  }

  // JHUAPL section end

  PLUGINLIB_EXPORT_CLASS(rtabmap_ros::CoreWrapper, nodelet::Nodelet);
}
