/*
Modified by: Johns Hopkins University Applied Physics Laboratory
			
Original by:
Copyright (c) 2010-2016, Mathieu Labbe - IntRoLab - Universite de Sherbrooke
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
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

#include "rtabmap_ros/MapsManager.h"

#include <rtabmap/utilite/ULogger.h>
#include <rtabmap/utilite/UTimer.h>
#include <rtabmap/utilite/UStl.h>
#include <rtabmap/utilite/UConversion.h>
#include <rtabmap/core/util3d_mapping.h>
#include <rtabmap/core/util3d_filtering.h>
#include <rtabmap/core/util3d_transforms.h>
#include <rtabmap/core/util2d.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Graph.h>
#include <rtabmap/core/Version.h>
#include <rtabmap/core/OccupancyGrid.h>
#include <rtabmap/core/Link.h>
#include <rtabmap/core/Transform.h>

#include <pcl/search/kdtree.h>

#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl_conversions/pcl_conversions.h>

#ifdef WITH_OCTOMAP_MSGS
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#ifdef RTABMAP_OCTOMAP
#include <octomap/ColorOcTree.h>
#include <rtabmap/core/OctoMap.h>
#endif
#endif

#include <rtabmap_ros/utils_mapping.h>
#include <Eigen/Dense>

//system
#include <string>
#include <fmt/format.h>

using namespace rtabmap;

MapsManager::MapsManager() :
		cloudOutputVoxelized_(true),
		cloudSubtractFiltering_(false),
		cloudSubtractFilteringMinNeighbors_(2),
		mapFilterRadius_(0.0),
		mapFilterAngle_(30.0), // degrees
		mapCacheCleanup_(true),
		alwaysUpdateMap_(false),
		scanEmptyRayTracing_(true),
		assembledObstacles_(new pcl::PointCloud<pcl::PointXYZRGB>),
		assembledGround_(new pcl::PointCloud<pcl::PointXYZRGB>),
		occupancyGrid_(new OccupancyGrid),
		gridUpdated_(true),
		octomap_(0),
		semanticOctomap_(0),
		octomapTreeDepth_(16),
		octomapUpdated_(true),
		latching_(true),
		semanticSegmentationEnable_(false),
		publishSemanticMask_(false),
		gridMaxObstacleHeight_(std::numeric_limits<double>::max()),
		octomapRayTracing_(false)
{
}

void MapsManager::init(ros::NodeHandle & nh, ros::NodeHandle & pnh, const std::string & name, bool usePublicNamespace)
{
	// common map stuff
	pnh.param("map_filter_radius", mapFilterRadius_, mapFilterRadius_);
	pnh.param("map_filter_angle", mapFilterAngle_, mapFilterAngle_);
	pnh.param("map_cleanup", mapCacheCleanup_, mapCacheCleanup_);

	if(pnh.hasParam("map_negative_poses_ignored"))
	{
		ROS_WARN("Parameter \"map_negative_poses_ignored\" has been "
				"removed. Use \"map_always_update\" instead.");
		if(!pnh.hasParam("map_always_update"))
		{
			bool negPosesIgnored;
			pnh.getParam("map_negative_poses_ignored", negPosesIgnored);
			alwaysUpdateMap_ = !negPosesIgnored;
		}
	}
	pnh.param("map_always_update", alwaysUpdateMap_, alwaysUpdateMap_);

	if(pnh.hasParam("map_negative_scan_empty_ray_tracing"))
	{
		ROS_WARN("Parameter \"map_negative_scan_empty_ray_tracing\" has been "
				"removed. Use \"map_empty_ray_tracing\" instead.");
		if(!pnh.hasParam("map_empty_ray_tracing"))
		{
			pnh.getParam("map_negative_scan_empty_ray_tracing", scanEmptyRayTracing_);
		}
	}
	pnh.param("map_empty_ray_tracing", scanEmptyRayTracing_, scanEmptyRayTracing_);

	if(pnh.hasParam("scan_output_voxelized"))
	{
		ROS_WARN("Parameter \"scan_output_voxelized\" has been "
				"removed. Use \"cloud_output_voxelized\" instead.");
		if(!pnh.hasParam("cloud_output_voxelized"))
		{
			pnh.getParam("scan_output_voxelized", cloudOutputVoxelized_);
		}
	}
	pnh.param("cloud_output_voxelized", cloudOutputVoxelized_, cloudOutputVoxelized_);
	pnh.param("cloud_subtract_filtering", cloudSubtractFiltering_, cloudSubtractFiltering_);
	pnh.param("cloud_subtract_filtering_min_neighbors", cloudSubtractFilteringMinNeighbors_, cloudSubtractFilteringMinNeighbors_);

	ROS_INFO("%s(maps): map_filter_radius          = %f", name.c_str(), mapFilterRadius_);
	ROS_INFO("%s(maps): map_filter_angle           = %f", name.c_str(), mapFilterAngle_);
	ROS_INFO("%s(maps): map_cleanup                = %s", name.c_str(), mapCacheCleanup_?"true":"false");
	ROS_INFO("%s(maps): map_always_update          = %s", name.c_str(), alwaysUpdateMap_?"true":"false");
	ROS_INFO("%s(maps): map_empty_ray_tracing      = %s", name.c_str(), scanEmptyRayTracing_?"true":"false");
	ROS_INFO("%s(maps): cloud_output_voxelized     = %s", name.c_str(), cloudOutputVoxelized_?"true":"false");
	ROS_INFO("%s(maps): cloud_subtract_filtering   = %s", name.c_str(), cloudSubtractFiltering_?"true":"false");
	ROS_INFO("%s(maps): cloud_subtract_filtering_min_neighbors = %d", name.c_str(), cloudSubtractFilteringMinNeighbors_);
	
	// JHUAPL section
	pnh.param("Grid/EnableSemanticSegmentation", semanticSegmentationEnable_, semanticSegmentationEnable_);
	pnh.param("model_classes_file_path", semanticSegmentationModelFilePath_, semanticSegmentationModelFilePath_);
	ROS_INFO("%s(maps): Grid/EnableSemanticSegmentation = %s", name.c_str(), semanticSegmentationEnable_?"true":"false");
	ROS_INFO("%s(maps): model_classes_file_path = %s", name.c_str(), semanticSegmentationModelFilePath_.empty()?"NOT_PATH":
				semanticSegmentationModelFilePath_.c_str());
	pnh.param("publish_semantic_mask", publishSemanticMask_, publishSemanticMask_);
	if(publishSemanticMask_)
	{
		ros::NodeHandle semantic_nh(nh, "semantic");
		image_transport::ImageTransport semantic_it(semantic_nh);
		semanticMaskPub_ = semantic_it.advertise("image_mask", 1);
	}

	std::string multiLevelCellSizeStr = "0.5 0.5 0.5 0.5 0.5";
	pnh.param("Grid/MultiLevelCellSize", multiLevelCellSizeStr, multiLevelCellSizeStr);
	ROS_INFO("%s(maps): Grid/MultiLevelCellSize = %s", name.c_str(), multiLevelCellSizeStr.c_str());
	std::list<std::string> multiLevelCellSizeStrList = uSplit(multiLevelCellSizeStr, ' ');
	std::vector<float> multiLevelCellSize(multiLevelCellSizeStrList.size());

	unsigned int index = 0;
	for(std::list<std::string>::iterator jter = multiLevelCellSizeStrList.begin(); jter != multiLevelCellSizeStrList.end(); ++jter)
	{
		multiLevelCellSize.at(index) = uStr2Float(*jter);
		++index;
	}

	std::string multiLevelTreeNameStr = "static movable dynamic";
	pnh.param("Grid/MultiLevelTreeName", multiLevelTreeNameStr, multiLevelTreeNameStr);
	ROS_INFO("%s(maps): Grid/MultiLevelTreeName = %s", name.c_str(), multiLevelTreeNameStr.c_str());
	std::list<std::string> multiLevelTreeName = uSplit(multiLevelTreeNameStr, ' ');

	int octoMapNumThreads = 2;
	pnh.param("Grid/OctoMapNumThreads", octoMapNumThreads, octoMapNumThreads);
	ROS_INFO("Grid/OctoMapNumThreads = %d", octoMapNumThreads);
	std::string gridMaxObstacleHeight;
	pnh.param("Grid/MaxObstacleHeight", gridMaxObstacleHeight, gridMaxObstacleHeight);
	gridMaxObstacleHeight_ = std::stof(gridMaxObstacleHeight);
	ROS_INFO("(mapManager) Grid/MaxObstacleHeight = %f", gridMaxObstacleHeight_);
	octomapRayTracing_ = false;
	pnh.param("octomap_raytracing", octomapRayTracing_, octomapRayTracing_);
	ROS_INFO("octomap_raytracing = %s", octomapRayTracing_?"True":"False");

	// JHUAPL section end

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	// JHUAPL section
	if(semanticSegmentationEnable_)
	{
		semanticOctomap_ = new SemanticOctoMap(multiLevelCellSize, multiLevelTreeName, 0.5, occupancyGrid_->isFullUpdate(), occupancyGrid_->getUpdateError(), octoMapNumThreads);
	
		// set the model class map if available
		if(!semanticSegmentationModelFilePath_.empty())
		{
			std::map<std::string, std::map<unsigned int, std::string>> networkModelMap;
			std::map<unsigned int, cv::Point3f> modelMaskIdColorMap;
			
			if(!utils::parseModelConfig(semanticSegmentationModelFilePath_, networkModelMap, modelMaskIdColorMap))
			{
				ROS_WARN("parsing network model Config FAILED to parse the semantic segmentation ocupacy to label id to name association file");
			}

			// occupancy grid (3D) needs to create label to occupancy association
			occupancyGrid_->setModelAssociationMap(networkModelMap);
			occupancyGrid_->updateOccupancyMapStruct();
			occupancyGrid_->setMaskidColorMap(modelMaskIdColorMap);

			// semantic octomap needs to create label to occupancy association
			semanticOctomap_->setModelNameIdMap(networkModelMap);
			semanticOctomap_->updateOccupancyMapStruct();
			// configure association of labels id with multi-level map
			semanticOctomap_->setMultiLevelOctreeConfig(multiLevelCellSize.size());

			if(modelMaskIdColorMap.empty())
				semanticOctomap_->generateMaskIdColorMap();
			else
				semanticOctomap_->setMaskidColorMap(modelMaskIdColorMap);

			std::map<unsigned int, int> labelId2octreeId = semanticOctomap_->getObjectIdToOctreeId();
			occupancyGrid_->setObjectIdToOctreeId(labelId2octreeId);
		}
		else
		{
			ROS_ERROR("file with Label Name to Id association for semantic segmentation model Not FOUND!!");
		}
	}
	// JHUAPL section end
	else
	{
		octomap_ = new OctoMap(occupancyGrid_->getCellSize(), 0.5, occupancyGrid_->isFullUpdate(), occupancyGrid_->getUpdateError());
	}
	pnh.param("octomap_tree_depth", octomapTreeDepth_, octomapTreeDepth_);
	if(octomapTreeDepth_ > 16)
	{
		ROS_WARN("octomap_tree_depth maximum is 16");
		octomapTreeDepth_ = 16;
	}
	else if(octomapTreeDepth_ < 0)
	{
		ROS_WARN("octomap_tree_depth cannot be negative, set to 16 instead");
		octomapTreeDepth_ = 16;
	}
	ROS_INFO("%s(maps): octomap_tree_depth         = %d", name.c_str(), octomapTreeDepth_);
#endif
#endif

	// If true, the last message published on
	// the map topics will be saved and sent to new subscribers when they
	// connect
	pnh.param("latch", latching_, latching_);

	// mapping topics
	ros::NodeHandle * nht;
	if(usePublicNamespace)
	{
		nht = &nh;
	}
	else
	{
		nht = &pnh;
	}
	latched_.clear();
	gridMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("grid_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&gridMapPub_, false));
	gridProbMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("grid_prob_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&gridProbMapPub_, false));
	cloudMapPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&cloudMapPub_, false));
	cloudObstaclesPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_obstacles", 1, latching_);
	latched_.insert(std::make_pair((void*)&cloudObstaclesPub_, false));
	cloudGroundPub_ = nht->advertise<sensor_msgs::PointCloud2>("cloud_ground", 1, latching_);
	latched_.insert(std::make_pair((void*)&cloudGroundPub_, false));

	// deprecated
	projMapPub_ = nht->advertise<nav_msgs::OccupancyGrid>("proj_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&projMapPub_, false));
	scanMapPub_ = nht->advertise<sensor_msgs::PointCloud2>("scan_map", 1, latching_);
	latched_.insert(std::make_pair((void*)&scanMapPub_, false));


#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octoMapPubFull_ = nht->advertise<octomap_msgs::Octomap>("octomap_full", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapPubFull_, false));
	octoMapPubBin_ = nht->advertise<octomap_msgs::Octomap>("octomap_binary", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapPubBin_, false));
	octoMapCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_occupied_space_cloud", 1, latching_);
	latched_.insert(std::make_pair((void*)&octoMapCloud_, false));

	if(semanticSegmentationEnable_)
	{
		octoMapFullGroundPub_ = nht->advertise<octomap_msgs::Octomap>("octomap_full_ground", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapFullGroundPub_, false));
		octoMapFullCeilingPub_ = nht->advertise<octomap_msgs::Octomap>("octomap_full_ceiling", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapFullCeilingPub_, false));
		octoMapFullStaticPub_ = nht->advertise<octomap_msgs::Octomap>("octomap_full_static", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapFullStaticPub_, false));
		octoMapFullMovablePub_ = nht->advertise<octomap_msgs::Octomap>("octomap_full_movable", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapFullMovablePub_, false));
		octoMapFullDynamicPub_ = nht->advertise<octomap_msgs::Octomap>("octomap_full_dynamic", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapFullDynamicPub_, false));
	}
	else 
	{
		octoMapFrontierCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_global_frontier_space", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapFrontierCloud_, false));
		octoMapObstacleCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_obstacles", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapObstacleCloud_, false));
		octoMapGroundCloud_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_ground", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapGroundCloud_, false));
		octoMapEmptySpace_ = nht->advertise<sensor_msgs::PointCloud2>("octomap_empty_space", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapEmptySpace_, false));
		octoMapProj_ = nht->advertise<nav_msgs::OccupancyGrid>("octomap_grid", 1, latching_);
		latched_.insert(std::make_pair((void*)&octoMapProj_, false));
	}

#endif
#endif
}

MapsManager::~MapsManager() {
	clear();

	delete occupancyGrid_;

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	if(semanticOctomap_)
	{
		delete semanticOctomap_;
		semanticOctomap_ = 0;
	}
	if(octomap_)
	{
		delete octomap_;
		octomap_ = 0;
	}
#endif
#endif
}

void parameterMoved(
		ros::NodeHandle & nh,
		const std::string & rosName,
		const std::string & parameterName,
		ParametersMap & parameters)
{
	if(nh.hasParam(rosName))
	{
		ParametersMap::const_iterator iter = Parameters::getDefaultParameters().find(parameterName);
		if(iter != Parameters::getDefaultParameters().end())
		{
			ROS_WARN("Parameter \"%s\" has moved from "
					 "rtabmap_ros to rtabmap library. Use "
					 "parameter \"%s\" instead. The value is still "
					 "copied to new parameter name.",
					 rosName.c_str(),
					 parameterName.c_str());
			std::string type = Parameters::getType(parameterName);
			if(type.compare("float") || type.compare("double"))
			{
				double v = uStr2Double(iter->second);
				nh.getParam(rosName, v);
				parameters.insert(ParametersPair(parameterName, uNumber2Str(v)));
			}
			else if(type.compare("int") || type.compare("unsigned int"))
			{
				int v = uStr2Int(iter->second);
				nh.getParam(rosName, v);
				parameters.insert(ParametersPair(parameterName, uNumber2Str(v)));
			}
			else if(type.compare("bool"))
			{
				bool v = uStr2Bool(iter->second);
				nh.getParam(rosName, v);
				if(rosName.compare("grid_incremental") == 0)
				{
					v = !v; // new parameter is called kGridGlobalFullUpdate(), which is the inverse
				}
				parameters.insert(ParametersPair(parameterName, uNumber2Str(v)));

			}
			else
			{
				ROS_ERROR("Not handled type \"%s\" for parameter \"%s\"", type.c_str(), parameterName.c_str());
			}
		}
		else
		{
			ROS_ERROR("Parameter \"%s\" not found in default parameters.", parameterName.c_str());
		}
	}
}

void MapsManager::backwardCompatibilityParameters(ros::NodeHandle & pnh, ParametersMap & parameters) const
{
	// removed
	if(pnh.hasParam("cloud_frustum_culling"))
	{
		ROS_WARN("Parameter \"cloud_frustum_culling\" has been removed. OctoMap topics "
				"already do it. You can remove it from your launch file.");
	}

	// moved
	parameterMoved(pnh, "cloud_decimation", Parameters::kGridDepthDecimation(), parameters);
	parameterMoved(pnh, "cloud_max_depth", Parameters::kGridRangeMax(), parameters);
	parameterMoved(pnh, "cloud_min_depth", Parameters::kGridRangeMin(), parameters);
	parameterMoved(pnh, "cloud_voxel_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(pnh, "cloud_floor_culling_height", Parameters::kGridMaxGroundHeight(), parameters);
	parameterMoved(pnh, "cloud_ceiling_culling_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(pnh, "cloud_noise_filtering_radius", Parameters::kGridNoiseFilteringRadius(), parameters);
	parameterMoved(pnh, "cloud_noise_filtering_min_neighbors", Parameters::kGridNoiseFilteringMinNeighbors(), parameters);
	parameterMoved(pnh, "scan_decimation", Parameters::kGridScanDecimation(), parameters);
	parameterMoved(pnh, "scan_voxel_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(pnh, "proj_max_ground_angle", Parameters::kGridMaxGroundAngle(), parameters);
	parameterMoved(pnh, "proj_min_cluster_size", Parameters::kGridMinClusterSize(), parameters);
	parameterMoved(pnh, "proj_max_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(pnh, "proj_max_obstacles_height", Parameters::kGridMaxObstacleHeight(), parameters);
	parameterMoved(pnh, "proj_max_ground_height", Parameters::kGridMaxGroundHeight(), parameters);

	parameterMoved(pnh, "proj_detect_flat_obstacles", Parameters::kGridFlatObstacleDetected(), parameters);
	parameterMoved(pnh, "proj_map_frame", Parameters::kGridMapFrameProjection(), parameters);
	parameterMoved(pnh, "grid_unknown_space_filled", Parameters::kGridScan2dUnknownSpaceFilled(), parameters);
	parameterMoved(pnh, "grid_cell_size", Parameters::kGridCellSize(), parameters);
	parameterMoved(pnh, "grid_incremental", Parameters::kGridGlobalFullUpdate(), parameters);
	parameterMoved(pnh, "grid_size", Parameters::kGridGlobalMinSize(), parameters);
	parameterMoved(pnh, "grid_eroded", Parameters::kGridGlobalEroded(), parameters);
	parameterMoved(pnh, "grid_footprint_radius", Parameters::kGridGlobalFootprintRadius(), parameters);

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	parameterMoved(pnh, "octomap_ground_is_obstacle", Parameters::kGridGroundIsObstacle(), parameters);
	parameterMoved(pnh, "octomap_occupancy_thr", Parameters::kGridGlobalOccupancyThr(), parameters);
#endif
#endif
}

void MapsManager::setParameters(const rtabmap::ParametersMap & parameters)
{
	// JHUAPL section

	Parameters::parse(parameters, Parameters::kGridEnableSemanticSegmentation(), semanticSegmentationEnable_);

	std::string multiLevelCellSizeStr;
	Parameters::parse(parameters, Parameters::kGridMultiLevelCellSize(), multiLevelCellSizeStr);
	std::list<std::string> mlCellSizeStrList = uSplit(multiLevelCellSizeStr, ' ');

	//JHUAPL section end

	parameters_ = parameters;
	occupancyGrid_->parseParameters(parameters_);

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomap_mtx_.lock();

	if(semanticOctomap_)
	{
		delete semanticOctomap_;
		semanticOctomap_ = 0;
	}
	if(octomap_)
	{
		delete octomap_;
		octomap_ = 0;
	}
	// JHUAPL section
	if(semanticSegmentationEnable_)
	{
		semanticOctomap_ = new SemanticOctoMap(parameters_);

		// set the model class map if available
		if(!semanticSegmentationModelFilePath_.empty())
		{
			std::map<std::string, std::map<unsigned int, std::string>> networkModelMap;
			std::map<unsigned int, cv::Point3f> modelMaskIdColorMap;

			if(!utils::parseModelConfig(semanticSegmentationModelFilePath_, networkModelMap, modelMaskIdColorMap))
			{
				ROS_WARN("parseModelConfig {network model config} FAILED to parse the semantic semantation name architecture file");
			}

			// occupancy grid (3D) needs to create label to occupancy association
			occupancyGrid_->setModelAssociationMap(networkModelMap);
			occupancyGrid_->updateOccupancyMapStruct();
			occupancyGrid_->setMaskidColorMap(modelMaskIdColorMap);

			// semantic octomap needs to create label to occupancy association
			semanticOctomap_->setModelNameIdMap(networkModelMap);
			semanticOctomap_->updateOccupancyMapStruct();
			// configure association of labels id with multi-level map
			semanticOctomap_->setMultiLevelOctreeConfig(mlCellSizeStrList.size());

			if(modelMaskIdColorMap.empty())
				semanticOctomap_->generateMaskIdColorMap();
			else
				semanticOctomap_->setMaskidColorMap(modelMaskIdColorMap);
			
			std::map<unsigned int, int> labelId2octreeId = semanticOctomap_->getObjectIdToOctreeId();
			occupancyGrid_->setObjectIdToOctreeId(labelId2octreeId);
		}
		else
		{
			ROS_ERROR("file with Label Name to Id association for semantic segmentation model Not FOUND!!");
		}
	}
	// JHUAPL section end
	else
	{
		octomap_ = new OctoMap(parameters_);
	}
	octomap_mtx_.unlock();
#endif
#endif
}

void MapsManager::set2DMap(
		const cv::Mat & map,
		float xMin,
		float yMin,
		float cellSize,
		const std::map<int, rtabmap::Transform> & poses,
		const rtabmap::Memory * memory)
{
	occupancyGrid_->setMap(map, xMin, yMin, cellSize, poses);
	//update cache in case the map should be updated
	if(memory)
	{
		for(std::map<int, rtabmap::Transform>::const_iterator iter=poses.lower_bound(1); iter!=poses.end(); ++iter)
		{
			std::map<int, std::pair< std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator jter = gridMaps_.find(iter->first);
			if(!uContains(gridMaps_, iter->first))
			{
				rtabmap::SensorData data;
				data = memory->getNodeData(iter->first, false, false, false, true);
				if(data.gridCellSize() == 0.0f)
				{
					ROS_WARN("Local occupancy grid doesn't exist for node %d", iter->first);
				}
				else
				{
					cv::Mat ground, obstacles, emptyCells;
					data.uncompressData(
							0,
							0,
							0,
							0,
							&ground,
							&obstacles,
							&emptyCells);

					uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
					uInsert(gridMapsViewpoints_, std::make_pair(iter->first, data.gridViewPoint()));
					occupancyGrid_->addToCache(iter->first, ground, obstacles, emptyCells);
				}
			}
			else
			{
				occupancyGrid_->addToCache(iter->first, jter->second.first.first, jter->second.first.second, jter->second.second);
			}
		}
	}
}

void MapsManager::clear()
{
	grid_mtx_.lock();
	gridAPLMaps_.clear();
	gridMaps_.clear();
	gridMapsViewpoints_.clear();
	assembledGround_->clear();
	assembledObstacles_->clear();
	assembledGroundPoses_.clear();
	assembledObstaclePoses_.clear();
	assembledGroundIndex_.release();
	assembledObstacleIndex_.release();
	groundClouds_.clear();
	obstacleClouds_.clear();
	occupancyGrid_->clear();
	grid_mtx_.unlock();
#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomap_mtx_.lock();
	if(semanticOctomap_)
	{
		semanticOctomap_->clear();
	}
	if(octomap_)
	{
		octomap_->clear();
	}
	octomap_mtx_.unlock();
#endif
#endif
	for(std::map<void*, bool>::iterator iter=latched_.begin(); iter!=latched_.end(); ++iter)
	{
		iter->second = false;
	}
}

bool MapsManager::hasSubscribers() const
{
	return  cloudMapPub_.getNumSubscribers() != 0 ||
			cloudObstaclesPub_.getNumSubscribers() != 0 ||
			cloudGroundPub_.getNumSubscribers() != 0 ||
			projMapPub_.getNumSubscribers() != 0 ||
			gridMapPub_.getNumSubscribers() != 0 ||
			gridProbMapPub_.getNumSubscribers() != 0 ||
			scanMapPub_.getNumSubscribers() != 0 ||
			octoMapPubBin_.getNumSubscribers() != 0 ||
			octoMapPubFull_.getNumSubscribers() != 0 ||
			octoMapCloud_.getNumSubscribers() != 0 ||
			octoMapFrontierCloud_.getNumSubscribers() != 0 ||
			octoMapObstacleCloud_.getNumSubscribers() != 0 ||
			octoMapGroundCloud_.getNumSubscribers() != 0 ||
			octoMapEmptySpace_.getNumSubscribers() != 0 ||
			octoMapProj_.getNumSubscribers() != 0 ||
			octoMapFullGroundPub_.getNumSubscribers() != 0 ||
			octoMapFullCeilingPub_.getNumSubscribers() != 0 ||
			octoMapFullStaticPub_.getNumSubscribers() != 0 ||
			octoMapFullMovablePub_.getNumSubscribers() != 0 ||
			octoMapFullDynamicPub_.getNumSubscribers() != 0;
}

std::map<int, Transform> MapsManager::getFilteredPoses(const std::map<int, Transform> & poses)
{
	if(mapFilterRadius_ > 0.0)
	{
		// filter nodes
		double angle = mapFilterAngle_ == 0.0?CV_PI+0.1:mapFilterAngle_*CV_PI/180.0;
		return rtabmap::graph::radiusPosesFiltering(poses, mapFilterRadius_, angle);
	}
	return std::map<int, Transform>();
}

std::map<int, rtabmap::Transform> MapsManager::updateMapCaches(
		const std::map<int, rtabmap::Transform> & posesIn,
		const rtabmap::Memory * memory,
		bool updateGrid,
		bool updateOctomap,
		UMutex& memory_mtx,
		const std::map<int, rtabmap::Signature> & signaturesIn)
{
	bool updateGridCache = updateGrid || updateOctomap;
	if(!updateGrid && !updateOctomap)
	{	
		//  all false, update only those where we have subscribers
		updateOctomap =
				octoMapPubBin_.getNumSubscribers() != 0 ||
				octoMapPubFull_.getNumSubscribers() != 0 ||
				octoMapCloud_.getNumSubscribers() != 0 ||
				octoMapFrontierCloud_.getNumSubscribers() != 0 ||
				octoMapObstacleCloud_.getNumSubscribers() != 0 ||
				octoMapGroundCloud_.getNumSubscribers() != 0 ||
				octoMapEmptySpace_.getNumSubscribers() != 0 ||
				octoMapProj_.getNumSubscribers() != 0 ||
				octoMapFullGroundPub_.getNumSubscribers() != 0 ||
				octoMapFullCeilingPub_.getNumSubscribers() != 0 ||
				octoMapFullStaticPub_.getNumSubscribers() != 0 ||
				octoMapFullMovablePub_.getNumSubscribers() != 0 ||
				octoMapFullDynamicPub_.getNumSubscribers() != 0;

		updateGrid = projMapPub_.getNumSubscribers() != 0 ||
				gridMapPub_.getNumSubscribers() != 0 ||
				gridProbMapPub_.getNumSubscribers() != 0;

		updateGridCache = updateOctomap || updateGrid ||
				cloudMapPub_.getNumSubscribers() != 0 ||
				cloudObstaclesPub_.getNumSubscribers() != 0 ||
				cloudGroundPub_.getNumSubscribers() != 0 ||
				scanMapPub_.getNumSubscribers() != 0;
	}

#ifndef WITH_OCTOMAP_MSGS
	updateOctomap = false;
#endif
#ifndef RTABMAP_OCTOMAP
	updateOctomap = false;
#endif

	octomap_u_mtx_.lock();
	gridUpdated_ = updateGrid;
	octomapUpdated_ = updateOctomap;
	octomap_u_mtx_.unlock();

	UDEBUG("Updating map caches...");

	if(!memory && signaturesIn.size() == 0)
	{
		ROS_ERROR("Memory and signatures should not be both null!?");
		return std::map<int, rtabmap::Transform>();
	}
	
	// process only nodes (exclude landmarks)
	std::map<int, rtabmap::Transform> poses;
	if(posesIn.begin()->first < 0)
	{
		poses.insert(posesIn.lower_bound(0), posesIn.end());
	}
	else
	{
		poses = posesIn;
	}
	std::map<int, rtabmap::Transform> filteredPoses;
	
	// update cache
	if(updateGridCache)
	{
		// filter nodes
		if(mapFilterRadius_ > 0.0)
		{
			UDEBUG("Filter nodes...");
			double angle = mapFilterAngle_ == 0.0?CV_PI+0.1:mapFilterAngle_*CV_PI/180.0;
			filteredPoses = rtabmap::graph::radiusPosesFiltering(poses, mapFilterRadius_, angle);
			if(poses.find(0) != poses.end())
			{
				// make sure to keep latest data
				filteredPoses.insert(*poses.find(0));
			}
		}
		else
		{
			filteredPoses = poses;
		}
		
		/// JHUAPL modification:  filteredPoses is receiving the latest added sensor data and occupancy
		// if(!alwaysUpdateMap_)
		// {
		// 	filteredPoses.erase(0);
		// }


		bool occupancySavedInDB = memory && uStrNumCmp(memory->getDatabaseVersion(), "0.11.10")>=0?true:false;

		// JHUAPL section start
		// Copy all the needed signature data up-front so that we only have to lock one time;
		// the locks for this same data in the code further below should then never be reached/triggered
		std::map<int, rtabmap::Signature> signatures = signaturesIn;
		memory_mtx.lock();
		for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				if(updateGridCache && (iter->first == 0 || !uContains(gridAPLMaps_, iter->first)))
				{
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter == signatures.end())
					{
						rtabmap::SensorData tmpData = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, !occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, false, true);
						signatures.insert(std::make_pair(iter->first, Signature(iter->first, -1, 0, tmpData.stamp(), "", Transform(), Transform(), tmpData)));
					}
				
				}
			}
		}
		memory_mtx.unlock();
		// JHUAPL section end

		bool longUpdate = false;
		UTimer longUpdateTimer;
		grid_mtx_.lock();
		octomap_mtx_.lock();
		if(filteredPoses.size() > 20)
		{
			if(updateGridCache && !semanticSegmentationEnable_ && gridMaps_.size() < 5)
			{
				ROS_WARN("Many occupancy grids should be loaded (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-gridMaps_.size()));
				longUpdate = true;
			}
#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
			if(semanticSegmentationEnable_)
			{
				if(updateOctomap && semanticOctomap_ && semanticOctomap_->addedNodes().size() < 5)
				{
					ROS_WARN("Many clouds should be added to octomap (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-semanticOctomap_->addedNodes().size()));
					longUpdate = true;
				}
			}
			else
			{
				if(updateOctomap && octomap_ && octomap_->addedNodes().size() < 5)
				{
					ROS_WARN("Many clouds should be added to octomap (~%d), this may take a while to update the map(s)...", int(filteredPoses.size()-octomap_->addedNodes().size()));
					longUpdate = true;
				}
			}
#endif
#endif
		}
		
		// JHUAPL moved this to up above
		//bool occupancySavedInDB = memory && uStrNumCmp(memory->getDatabaseVersion(), "0.11.10")>=0?true:false;

		for(std::map<int, rtabmap::Transform>::iterator iter=filteredPoses.begin(); iter!=filteredPoses.end(); ++iter)
		{
			if(!iter->second.isNull())
			{
				rtabmap::SensorData data;
				if(updateGridCache && semanticSegmentationEnable_ && (iter->first == 0 || !uContains(gridAPLMaps_, iter->first)))
				{
					UDEBUG("Data required for %d", iter->first);
					// the argument signature is a local copy, and does not need to be lock for
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter != signatures.end())
					{
						data = findIter->second.sensorData();
						UDEBUG("	data from signature -> id: %d; stamp:%f; grid: %f", data.id(), data.stamp(), data.gridCellSizes().at(0));
					}
					else if(memory)
					{
						memory_mtx.lock();
						data = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, !occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, false, true);
						memory_mtx.unlock();
					}
					
					UDEBUG("Adding grid map %d to cache...", iter->first);
					cv::Point3f viewPoint;
					std::map<unsigned int, cv::Mat> obstaclesCellsMap;
					std::map<unsigned int, cv::Mat> emptyCellsMap;
					
					cv::Mat rgb, depth, semanticMask;
					bool generateGrid = true;
					if(data.gridCellSizes().size() > 0) 
					{
						// just looking that there is at least one octree with grid size greater than zero
						generateGrid = data.gridCellSizes().at(0) == 0.0f;
					}
					static bool warningShown = false;
					if(occupancySavedInDB && generateGrid && !warningShown)
					{
						warningShown = true;
						UWARN("Occupancy grid for location %d should be added to global map (e..g, a ROS node is subscribed to "
								"any occupancy grid output) but it cannot be found "
								"in memory. For convenience, the occupancy "
								"grid is regenerated. Make sure parameter \"%s\" is true to "
								"avoid this warning for the next locations added to map. For older "
								"locations already in database without an occupancy grid map, you can use the "
								"\"rtabmap-databaseViewer\" to regenerate the missing occupancy grid maps and "
								"save them back in the database for next sessions. This warning is only shown once.",
								data.id(), Parameters::kRGBDCreateOccupancyGrid().c_str());
					}
					if(memory && occupancySavedInDB && generateGrid)
					{
						// if we are here, it is because we loaded a database with old nodes not having occupancy grid set
						// try reload again
						memory_mtx.lock();
						data = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth(), !occupancyGrid_->isGridFromDepth(), false, false);
						memory_mtx.unlock();
					}
					data.uncompressData(
							occupancyGrid_->isGridFromDepth() && generateGrid?&rgb:0,
							occupancyGrid_->isGridFromDepth() && generateGrid?&depth:0,
							semanticSegmentationEnable_ && generateGrid?&semanticMask:0,
							generateGrid?0:&obstaclesCellsMap,
							generateGrid?0:&emptyCellsMap);

					// empty cell map might not have all the layers since mat with no elements were not store.
					// if  the empty cells map is not the same size as the active occupancy layers
					// the missing one would be added with a mat with not elements.
					UDEBUG(" emptyCellsMap size=%d", (int)emptyCellsMap.size());
					if(emptyCellsMap.size() < occupancyGrid_->getLayerIdToLayerName().size()) 
					{
						UDEBUG(" empty cells map doesnot contain correct number of layers");
						std::map<int, std::string> layerIdToLayerNameMap = occupancyGrid_->getLayerIdToLayerName();
						for(auto ldToLNMapIter = layerIdToLayerNameMap.begin(); ldToLNMapIter != layerIdToLayerNameMap.end(); ++ldToLNMapIter)
						{
							auto emptyCellsMapIter = emptyCellsMap.find(ldToLNMapIter->first);
							if(emptyCellsMapIter == emptyCellsMap.end())
							{
								cv::Mat emptyCells;
								emptyCellsMap.insert({(unsigned int)ldToLNMapIter->first, emptyCells});
							}
						}
					}
					
					if(generateGrid)
					{
						UDEBUG("generateGrid");
						Signature tmp(data);
						tmp.setPose(iter->second);
						occupancyGrid_->createLocalMap(tmp, obstaclesCellsMap, emptyCellsMap, viewPoint);
						uInsert(gridAPLMaps_, std::make_pair(iter->first, std::make_pair(obstaclesCellsMap, emptyCellsMap)));
						uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
					}
					else
					{
						viewPoint = data.gridViewPoint();
						uInsert(gridAPLMaps_, std::make_pair(iter->first, std::make_pair(obstaclesCellsMap, emptyCellsMap)));
						uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
					}
				
				}
				else if(updateGridCache && !semanticSegmentationEnable_ && (iter->first == 0 || !uContains(gridMaps_, iter->first)))
				{
					UDEBUG("( Default MODE) Data required for %d", iter->first);
					std::map<int, rtabmap::Signature>::const_iterator findIter = signatures.find(iter->first);
					if(findIter != signatures.end())
					{
						data = findIter->second.sensorData();
					}
					else if(memory)
					{
						memory_mtx.lock();
						data = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, !occupancyGrid_->isGridFromDepth() && !occupancySavedInDB, false, true);
						memory_mtx.unlock();
					}

					UDEBUG("Adding grid map %d to cache...", iter->first);
					cv::Point3f viewPoint;
					cv::Mat ground, obstacles, emptyCells;
					if(iter->first > 0)
					{
						cv::Mat rgb, depth;
						LaserScan scan;
						bool generateGrid = data.gridCellSize() == 0.0f;
						static bool warningShown = false;
						if(occupancySavedInDB && generateGrid && !warningShown)
						{
							warningShown = true;
							UWARN("Occupancy grid for location %d should be added to global map (e..g, a ROS node is subscribed to "
									"any occupancy grid output) but it cannot be found "
									"in memory. For convenience, the occupancy "
									"grid is regenerated. Make sure parameter \"%s\" is true to "
									"avoid this warning for the next locations added to map. For older "
									"locations already in database without an occupancy grid map, you can use the "
									"\"rtabmap-databaseViewer\" to regenerate the missing occupancy grid maps and "
									"save them back in the database for next sessions. This warning is only shown once.",
									data.id(), Parameters::kRGBDCreateOccupancyGrid().c_str());
						}
						if(memory && occupancySavedInDB && generateGrid)
						{
							// if we are here, it is because we loaded a database with old nodes not having occupancy grid set
							// try reload again
							data = memory->getNodeData(iter->first, occupancyGrid_->isGridFromDepth(), !occupancyGrid_->isGridFromDepth(), false, false);
						}
						data.uncompressData(
								occupancyGrid_->isGridFromDepth() && generateGrid?&rgb:0,
								occupancyGrid_->isGridFromDepth() && generateGrid?&depth:0,
								!occupancyGrid_->isGridFromDepth() && generateGrid?&scan:0,
								0,
								generateGrid?0:&ground,
								generateGrid?0:&obstacles,
								generateGrid?0:&emptyCells);

						if(generateGrid)
						{
							Signature tmp(data);
							tmp.setPose(iter->second);
							occupancyGrid_->createLocalMap(tmp, ground, obstacles, emptyCells, viewPoint);
							uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}
						else
						{
							viewPoint = data.gridViewPoint();
							uInsert(gridMaps_, std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}
					}
					else
					{
						// generate tmp occupancy grid for latest id (assuming data is already uncompressed)
						// For negative laser scans, fill empty space?
						bool unknownSpaceFilled = Parameters::defaultGridScan2dUnknownSpaceFilled();
						Parameters::parse(parameters_, Parameters::kGridScan2dUnknownSpaceFilled(), unknownSpaceFilled);

						if(unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_)
						{
							ParametersMap parameters;
							parameters.insert(ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), uBool2Str(scanEmptyRayTracing_)));
							occupancyGrid_->parseParameters(parameters);
						}

						cv::Mat rgb, depth;
						LaserScan scan;
						bool generateGrid = data.gridCellSize() == 0.0f || (unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_);
						data.uncompressData(
							occupancyGrid_->isGridFromDepth() && generateGrid?&rgb:0,
							occupancyGrid_->isGridFromDepth() && generateGrid?&depth:0,
							!occupancyGrid_->isGridFromDepth() && generateGrid?&scan:0,
							0,
							generateGrid?0:&ground,
							generateGrid?0:&obstacles,
							generateGrid?0:&emptyCells);

						if(generateGrid)
						{
							Signature tmp(data);
							tmp.setPose(iter->second);
							occupancyGrid_->createLocalMap(tmp, ground, obstacles, emptyCells, viewPoint);
							uInsert(gridMaps_,  std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}
						else
						{
							viewPoint = data.gridViewPoint();
							uInsert(gridMaps_,  std::make_pair(iter->first, std::make_pair(std::make_pair(ground, obstacles), emptyCells)));
							uInsert(gridMapsViewpoints_, std::make_pair(iter->first, viewPoint));
						}

						// put back
						if(unknownSpaceFilled != scanEmptyRayTracing_ && scanEmptyRayTracing_)
						{
							ParametersMap parameters;
							parameters.insert(ParametersPair(Parameters::kGridScan2dUnknownSpaceFilled(), uBool2Str(unknownSpaceFilled)));
							occupancyGrid_->parseParameters(parameters);
						}
					}
				}
				
				if(updateGrid && !semanticSegmentationEnable_ &&
						(iter->first == 0 ||
						  occupancyGrid_->addedNodes().find(iter->first) == occupancyGrid_->addedNodes().end()))
				{
					std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator mter = gridMaps_.find(iter->first);
					if(mter != gridMaps_.end())
					{
						if(!mter->second.first.first.empty() || !mter->second.first.second.empty() || !mter->second.second.empty())
						{
							occupancyGrid_->addToCache(iter->first, mter->second.first.first, mter->second.first.second, mter->second.second);
						}
					}
				}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
				if(updateOctomap && semanticSegmentationEnable_ &&
					(iter->first == 0 || 
						semanticOctomap_->addedNodes().find(iter->first) == semanticOctomap_->addedNodes().end()))
				{
					std::map<int, std::pair< std::map<unsigned int, cv::Mat>, std::map<unsigned int, cv::Mat> > >::iterator mter = gridAPLMaps_.find(iter->first);
					std::map<int, cv::Point3f>::iterator pter = gridMapsViewpoints_.find(iter->first);
					if(mter != gridAPLMaps_.end() && pter != gridMapsViewpoints_.end())
					{
						if( (mter->second.first.begin()->second.empty() || mter->second.first.begin()->second.channels() > 2) &&
						   (mter->second.second.begin()->second.empty() || mter->second.second.begin()->second.channels() > 2) )
						{
							semanticOctomap_->addToCache(iter->first, mter->second.first, mter->second.second, pter->second);
						}
						else
						{
							ROS_WARN("Node %d: Cannot update octomap occupancy grids were not added to octomap cache! ", iter->first);
						}
					}
				}
				else if(updateOctomap && !semanticSegmentationEnable_ &&
							(iter->first == 0 ||
						  		octomap_->addedNodes().find(iter->first) == octomap_->addedNodes().end()))
				{
					std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator mter = gridMaps_.find(iter->first);
					std::map<int, cv::Point3f>::iterator pter = gridMapsViewpoints_.find(iter->first);
					if(mter != gridMaps_.end() && pter!=gridMapsViewpoints_.end())
					{
						if((mter->second.first.first.empty() || mter->second.first.first.channels() > 2) &&
						   (mter->second.first.second.empty() || mter->second.first.second.channels() > 2) &&
						   (mter->second.second.empty() || mter->second.second.channels() > 2))
						{
							octomap_->addToCache(iter->first, mter->second.first.first, mter->second.first.second, mter->second.second, pter->second);
						}
						else if(!mter->second.first.first.empty() && !mter->second.first.second.empty() && !mter->second.second.empty())
						{
							ROS_WARN("Node %d: Cannot update octomap with 2D occupancy grids. "
									"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
									"all occupancy grid parameters.",
									iter->first);
						}
					}
				}
#endif
#endif
			}
			else
			{
				ROS_ERROR("Pose null for node %d", iter->first);
			}
		}

		if(updateGrid && !semanticSegmentationEnable_)
		{
			gridUpdated_ = occupancyGrid_->update(filteredPoses);
		}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
		octomap_u_mtx_.lock();
		if(updateOctomap && semanticSegmentationEnable_)
		{
			if(octomapRayTracing_) {
				std::list<std::string> rayTraceLayers;
				rayTraceLayers.push_back("static");

				UTimer time;
				octomapUpdated_ = semanticOctomap_->update(filteredPoses, true, &rayTraceLayers);
				UINFO("\nSemanticOctomap update time = %fs", time.ticks());
			}
			else {
				UTimer time;
				octomapUpdated_ = semanticOctomap_->update(filteredPoses);
				UINFO("\nSemanticOctomap update time = %fs", time.ticks());
			}
			
		}
		else if(updateOctomap)
		{
			UTimer time;
			octomapUpdated_ = octomap_->update(filteredPoses);
			UINFO("\nOctomap update time = %fs", time.ticks());
		}
		octomap_u_mtx_.unlock();
#endif
#endif
		octomap_mtx_.unlock();


		if(!semanticSegmentationEnable_)
		{
			for(std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator iter=gridMaps_.begin();
				iter!=gridMaps_.end();)
			{	
				if(!uContains(poses, iter->first))
				{
					UASSERT(gridMapsViewpoints_.erase(iter->first) != 0);
					gridMaps_.erase(iter++);
				}
				else
				{
					++iter;
				}
			}

			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=groundClouds_.begin();
				iter!=groundClouds_.end();)
			{
				if(!uContains(poses, iter->first))
				{
					groundClouds_.erase(iter++);
				}
				else
				{
					++iter;
				}
			}

			for(std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator iter=obstacleClouds_.begin();
				iter!=obstacleClouds_.end();)
			{
				if(!uContains(poses, iter->first))
				{
					obstacleClouds_.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}
		else
		{
			// In Semantic Segmentation Mode
			for(std::map<int, std::pair< std::map<unsigned int, cv::Mat>, std::map<unsigned int, cv::Mat> > >::iterator iter=gridAPLMaps_.begin(); iter!=gridAPLMaps_.end();)
			{
				if(!uContains(poses, iter->first))
				{
					UASSERT(gridMapsViewpoints_.erase(iter->first) != 0);
					gridAPLMaps_.erase(iter++);
				}
				else
				{
					++iter;
				}
			}
		}
		grid_mtx_.unlock();
		
		if(longUpdate)
		{
			ROS_WARN("Map(s) updated! (%f s)", longUpdateTimer.ticks());
		}
	}

	return filteredPoses;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractFiltering(
		const pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud,
		const rtabmap::FlannIndex & substractCloudIndex,
		float radiusSearch,
		int minNeighborsInRadius)
{
	UASSERT(minNeighborsInRadius > 0);
	UASSERT(substractCloudIndex.indexedFeatures());

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output(new pcl::PointCloud<pcl::PointXYZRGB>);
	output->resize(cloud->size());
	int oi = 0; // output iterator
	for(unsigned int i=0; i<cloud->size(); ++i)
	{
		std::vector<std::vector<size_t> > kIndices;
		std::vector<std::vector<float> > kDistances;
		cv::Mat pt = (cv::Mat_<float>(1, 3) << cloud->at(i).x, cloud->at(i).y, cloud->at(i).z);
		substractCloudIndex.radiusSearch(pt, kIndices, kDistances, radiusSearch, minNeighborsInRadius, 32, 0, false);
		if(kIndices.size() == 1 && kIndices[0].size() < minNeighborsInRadius)
		{
			output->at(oi++) = cloud->at(i);
		}
	}
	output->resize(oi);
	return output;
}

void MapsManager::publishMaps(
		const std::map<int, rtabmap::Transform> & poses,
		const ros::Time & stamp,
		const std::string & mapFrameId)
{
	UDEBUG("Publishing maps...");

	grid_mtx_.lock();
	// publish maps
	if(cloudMapPub_.getNumSubscribers() ||
	   scanMapPub_.getNumSubscribers() ||
	   cloudObstaclesPub_.getNumSubscribers() ||
	   cloudGroundPub_.getNumSubscribers())
	{
		// generate the assembled cloud!
		UTimer time;

		if(scanMapPub_.getNumSubscribers())
		{
			if(parameters_.find(Parameters::kGridFromDepth()) != parameters_.end() &&
				uStr2Bool(parameters_.at(Parameters::kGridFromDepth())))
			{
				ROS_WARN("/scan_map topic is deprecated! Subscribe to /cloud_map topic "
						"instead with <param name=\"%s\" type=\"string\" value=\"false\"/>. "
						"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
						"all occupancy grid parameters.",
						Parameters::kGridFromDepth().c_str());
			}
			else
			{
				ROS_WARN("/scan_map topic is deprecated! Subscribe to /cloud_map topic instead.");
			}
		}

		// detect if the graph has changed, if so, recreate the clouds
		bool graphGroundOptimized = false;
		bool graphObstacleOptimized = false;
		bool updateGround = cloudMapPub_.getNumSubscribers() ||
				   scanMapPub_.getNumSubscribers() ||
				   cloudGroundPub_.getNumSubscribers();
		bool updateObstacles = cloudMapPub_.getNumSubscribers() ||
				   scanMapPub_.getNumSubscribers() ||
				   cloudObstaclesPub_.getNumSubscribers();
		bool graphGroundChanged = updateGround;
		bool graphObstacleChanged = updateObstacles;

		for(std::map<int, Transform>::const_iterator iter=poses.lower_bound(0); iter!=poses.end(); ++iter)
		{
			std::map<int, Transform>::const_iterator jter;
			if(updateGround)
			{
				jter = assembledGroundPoses_.find(iter->first);
				if(jter != assembledGroundPoses_.end())
				{
					graphGroundChanged = false;
					UASSERT(!iter->second.isNull() && !jter->second.isNull());
					if(iter->second.getDistanceSquared(jter->second) > 0.0001)
					{
						graphGroundOptimized = true;
					}
				}
			}
			if(updateObstacles)
			{
				jter = assembledObstaclePoses_.find(iter->first);
				if(jter != assembledObstaclePoses_.end())
				{
					graphObstacleChanged = false;
					UASSERT(!iter->second.isNull() && !jter->second.isNull());
					if(iter->second.getDistanceSquared(jter->second) > 0.0001)
					{
						graphObstacleOptimized = true;
					}
				}
			}
		}
		int countObstacles = 0;
		int countGrounds = 0;
		int previousIndexedGroundSize = assembledGroundIndex_.indexedFeatures();
		int previousIndexedObstacleSize = assembledObstacleIndex_.indexedFeatures();
		if(graphGroundOptimized || graphGroundChanged)
		{
			int previousSize = assembledGround_->size();
			assembledGround_->clear();
			assembledGround_->reserve(previousSize);
			assembledGroundPoses_.clear();
			assembledGroundIndex_.release();
		}
		if(graphObstacleOptimized || graphObstacleChanged)
		{
			int previousSize = assembledObstacles_->size();
			assembledObstacles_->clear();
			assembledObstacles_->reserve(previousSize);
			assembledObstaclePoses_.clear();
			assembledObstacleIndex_.release();
		}

		if(graphGroundOptimized || graphObstacleOptimized)
		{
			ROS_INFO("Graph has changed, updating clouds...");
			UTimer t;
			cv::Mat tmpGroundPts;
			cv::Mat tmpObstaclePts;
			for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
			{
				if(iter->first > 0)
				{
					if(updateGround  &&
					   (graphGroundOptimized || assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end()))
					{
						assembledGroundPoses_.insert(*iter);
						std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator kter=groundClouds_.find(iter->first);
						if(kter != groundClouds_.end() && kter->second->size())
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(kter->second, iter->second);
							*assembledGround_+=*transformed;
							if(cloudSubtractFiltering_)
							{
								for(unsigned int i=0; i<transformed->size(); ++i)
								{
									if(tmpGroundPts.empty())
									{
										tmpGroundPts = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
										tmpGroundPts.reserve(previousIndexedGroundSize>0?previousIndexedGroundSize:100);
									}
									else
									{
										cv::Mat pt = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
										tmpGroundPts.push_back(pt);
									}
								}
							}
							++countGrounds;
						}
					}
					if(updateObstacles  &&
					   (graphObstacleOptimized || assembledObstaclePoses_.find(iter->first) == assembledObstaclePoses_.end()))
					{
						assembledObstaclePoses_.insert(*iter);
						std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr >::iterator kter=obstacleClouds_.find(iter->first);
						if(kter != obstacleClouds_.end() && kter->second->size())
						{
							pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::transformPointCloud(kter->second, iter->second);
							*assembledObstacles_+=*transformed;
							if(cloudSubtractFiltering_)
							{
								for(unsigned int i=0; i<transformed->size(); ++i)
								{
									if(tmpObstaclePts.empty())
									{
										tmpObstaclePts = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
										tmpObstaclePts.reserve(previousIndexedObstacleSize>0?previousIndexedObstacleSize:100);
									}
									else
									{
										cv::Mat pt = (cv::Mat_<float>(1, 3) << transformed->at(i).x, transformed->at(i).y, transformed->at(i).z);
										tmpObstaclePts.push_back(pt);
									}
								}
							}
							++countObstacles;
						}
					}
				}
			}
			double addingPointsTime = t.ticks();

			if(graphGroundOptimized && !tmpGroundPts.empty())
			{
				assembledGroundIndex_.buildKDTreeSingleIndex(tmpGroundPts, 15);
			}
			if(graphObstacleOptimized && !tmpObstaclePts.empty())
			{
				assembledObstacleIndex_.buildKDTreeSingleIndex(tmpObstaclePts, 15);
			}
			double indexingTime = t.ticks();
			UINFO("Graph optimized! Time recreating clouds (%d ground, %d obstacles) = %f s (indexing %fs)", countGrounds, countObstacles, addingPointsTime+indexingTime, indexingTime);
		}
		else if(graphGroundChanged || graphObstacleChanged)
		{
			UWARN("Graph has changed! The whole cloud is regenerated.");
		}

		for(std::map<int, Transform>::const_iterator iter = poses.begin(); iter!=poses.end(); ++iter)
		{
			std::map<int, std::pair<std::pair<cv::Mat, cv::Mat>, cv::Mat> >::iterator jter = gridMaps_.find(iter->first);
			if(updateGround  && assembledGroundPoses_.find(iter->first) == assembledGroundPoses_.end())
			{
				if(iter->first > 0)
				{
					assembledGroundPoses_.insert(*iter);
				}
				if(jter!=gridMaps_.end() && jter->second.first.first.cols)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(jter->second.first.first), iter->second, 0, 255, 0);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractedCloud = transformed;
					if(cloudSubtractFiltering_)
					{
						if(assembledGroundIndex_.indexedFeatures())
						{
							subtractedCloud = subtractFiltering(transformed, assembledGroundIndex_, occupancyGrid_->getCellSize(), cloudSubtractFilteringMinNeighbors_);
						}
						if(subtractedCloud->size())
						{
							UDEBUG("Adding ground %d pts=%d/%d (index=%d)", iter->first, subtractedCloud->size(), transformed->size(), assembledGroundIndex_.indexedFeatures());
							cv::Mat pts(subtractedCloud->size(), 3, CV_32FC1);
							for(unsigned int i=0; i<subtractedCloud->size(); ++i)
							{
								pts.at<float>(i, 0) = subtractedCloud->at(i).x;
								pts.at<float>(i, 1) = subtractedCloud->at(i).y;
								pts.at<float>(i, 2) = subtractedCloud->at(i).z;
							}
							if(!assembledGroundIndex_.isBuilt())
							{
								assembledGroundIndex_.buildKDTreeSingleIndex(pts, 15);
							}
							else
							{
								assembledGroundIndex_.addPoints(pts);
							}
						}
					}
					if(iter->first>0)
					{
						groundClouds_.insert(std::make_pair(iter->first, util3d::transformPointCloud(subtractedCloud, iter->second.inverse())));
					}
					if(subtractedCloud->size())
					{
						*assembledGround_+=*subtractedCloud;
					}
					++countGrounds;
				}
			}
			if(updateObstacles  && assembledObstaclePoses_.find(iter->first) == assembledObstaclePoses_.end())
			{
				if(iter->first > 0)
				{
					assembledObstaclePoses_.insert(*iter);
				}
				if(jter!=gridMaps_.end() && jter->second.first.second.cols)
				{
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed = util3d::laserScanToPointCloudRGB(LaserScan::backwardCompatibility(jter->second.first.second), iter->second, 255, 0, 0);
					pcl::PointCloud<pcl::PointXYZRGB>::Ptr subtractedCloud = transformed;
					if(cloudSubtractFiltering_)
					{
						if(assembledObstacleIndex_.indexedFeatures())
						{
							subtractedCloud = subtractFiltering(transformed, assembledObstacleIndex_, occupancyGrid_->getCellSize(), cloudSubtractFilteringMinNeighbors_);
						}
						if(subtractedCloud->size())
						{
							UDEBUG("Adding obstacle %d pts=%d/%d (index=%d)", iter->first, subtractedCloud->size(), transformed->size(), assembledObstacleIndex_.indexedFeatures());
							cv::Mat pts(subtractedCloud->size(), 3, CV_32FC1);
							for(unsigned int i=0; i<subtractedCloud->size(); ++i)
							{
								pts.at<float>(i, 0) = subtractedCloud->at(i).x;
								pts.at<float>(i, 1) = subtractedCloud->at(i).y;
								pts.at<float>(i, 2) = subtractedCloud->at(i).z;
							}
							if(!assembledObstacleIndex_.isBuilt())
							{
								assembledObstacleIndex_.buildKDTreeSingleIndex(pts, 15);
							}
							else
							{
								assembledObstacleIndex_.addPoints(pts);
							}
						}
					}
					if(iter->first>0)
					{
						obstacleClouds_.insert(std::make_pair(iter->first, util3d::transformPointCloud(subtractedCloud, iter->second.inverse())));
					}
					if(subtractedCloud->size())
					{
						*assembledObstacles_+=*subtractedCloud;
					}
					++countObstacles;
				}
			}
		}

		if(cloudOutputVoxelized_)
		{
			UASSERT(occupancyGrid_->getCellSize() > 0.0);
			if(countGrounds && assembledGround_->size())
			{
				assembledGround_ = util3d::voxelize(assembledGround_, occupancyGrid_->getCellSize());
			}
			if(countObstacles && assembledObstacles_->size())
			{
				assembledObstacles_ = util3d::voxelize(assembledObstacles_, occupancyGrid_->getCellSize());
			}
		}

		ROS_INFO("Assembled %d obstacle and %d ground clouds (%d points, %fs)",
				countObstacles, countGrounds, (int)(assembledGround_->size() + assembledObstacles_->size()), time.ticks());

		if( countGrounds > 0 ||
			countObstacles > 0 ||
			!latching_ ||
			(assembledGround_->empty() && assembledObstacles_->empty()) ||
			(cloudGroundPub_.getNumSubscribers() && !latched_.at(&cloudGroundPub_)) ||
			(cloudObstaclesPub_.getNumSubscribers() && !latched_.at(&cloudObstaclesPub_)) ||
			(cloudMapPub_.getNumSubscribers() && !latched_.at(&cloudMapPub_)) ||
			(scanMapPub_.getNumSubscribers() && !latched_.at(&scanMapPub_)))
		{
			if(cloudGroundPub_.getNumSubscribers())
			{
				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledGround_, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;
				cloudGroundPub_.publish(cloudMsg);
				latched_.at(&cloudGroundPub_) = true;
			}
			if(cloudObstaclesPub_.getNumSubscribers())
			{
				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(*assembledObstacles_, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;
				cloudObstaclesPub_.publish(cloudMsg);
				latched_.at(&cloudObstaclesPub_) = true;
			}
			if(cloudMapPub_.getNumSubscribers() || scanMapPub_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloud = *assembledObstacles_ + *assembledGround_;
				sensor_msgs::PointCloud2::Ptr cloudMsg(new sensor_msgs::PointCloud2);
				pcl::toROSMsg(cloud, *cloudMsg);
				cloudMsg->header.stamp = stamp;
				cloudMsg->header.frame_id = mapFrameId;

				if(cloudMapPub_.getNumSubscribers())
				{
					cloudMapPub_.publish(cloudMsg);
					latched_.at(&cloudMapPub_) = true;
				}
				if(scanMapPub_.getNumSubscribers())
				{
					scanMapPub_.publish(cloudMsg);
					latched_.at(&scanMapPub_) = true;
				}
			}
		}
	}
	else if(mapCacheCleanup_)
	{
		assembledGround_->clear();
		assembledObstacles_->clear();
		assembledGroundPoses_.clear();
		assembledObstaclePoses_.clear();
		assembledGroundIndex_.release();
		assembledObstacleIndex_.release();
		groundClouds_.clear();
		obstacleClouds_.clear();
	}
	if(cloudMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&cloudMapPub_) = false;
	}
	if(scanMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&scanMapPub_) = false;
	}
	if(cloudGroundPub_.getNumSubscribers() == 0)
	{
		latched_.at(&cloudGroundPub_) = false;
	}
	if(cloudObstaclesPub_.getNumSubscribers() == 0)
	{
		latched_.at(&cloudObstaclesPub_) = false;
	}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomap_u_mtx_.lock();
	bool octomapUpdated = octomapUpdated_;
	octomap_u_mtx_.unlock();

	/// TODO: need to copy all needed octomap_ data under a single lock rather than
	///        seperate locks to ensure consistency of different parts of the data;
	///        using multiple locks spread out over time allows octree data to change between locks
	octomap_mtx_.lock();
	if( octomapUpdated ||
		!latching_ ||
		(octoMapPubBin_.getNumSubscribers() && !latched_.at(&octoMapPubBin_)) ||
		(octoMapPubFull_.getNumSubscribers() && !latched_.at(&octoMapPubFull_)) ||
		(octoMapCloud_.getNumSubscribers() && !latched_.at(&octoMapCloud_)) ||
		(octoMapFrontierCloud_.getNumSubscribers() && !latched_.at(&octoMapFrontierCloud_)) ||
		(octoMapObstacleCloud_.getNumSubscribers() && !latched_.at(&octoMapObstacleCloud_)) ||
		(octoMapGroundCloud_.getNumSubscribers() && !latched_.at(&octoMapGroundCloud_)) ||
		(octoMapEmptySpace_.getNumSubscribers() && !latched_.at(&octoMapEmptySpace_)) ||
		(octoMapProj_.getNumSubscribers() && !latched_.at(&octoMapProj_)))
	{	
		if(octoMapPubBin_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::binaryMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubBin_.publish(msg);
			latched_.at(&octoMapPubBin_) = true;
		}
		if(octoMapPubFull_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::fullMapToMsg(*octomap_->octree(), msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubFull_.publish(msg);
			latched_.at(&octoMapPubFull_) = true;
		}
		if(octoMapCloud_.getNumSubscribers() ||
		octoMapFrontierCloud_.getNumSubscribers() ||
		octoMapObstacleCloud_.getNumSubscribers() ||
		octoMapGroundCloud_.getNumSubscribers() ||
		octoMapEmptySpace_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 msg;
			pcl::IndicesPtr obstacleIndices(new std::vector<int>);
			pcl::IndicesPtr frontierIndices(new std::vector<int>);
			pcl::IndicesPtr emptyIndices(new std::vector<int>);
			pcl::IndicesPtr groundIndices(new std::vector<int>);
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = octomap_->createCloud(octomapTreeDepth_, obstacleIndices.get(), emptyIndices.get(), groundIndices.get(), true, frontierIndices.get());

			if(octoMapCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudOccupiedSpace;
				pcl::IndicesPtr indices = util3d::concatenate(obstacleIndices, groundIndices);
				pcl::copyPointCloud(*cloud, *indices, cloudOccupiedSpace);
				pcl::toROSMsg(cloudOccupiedSpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapCloud_.publish(msg);
				latched_.at(&octoMapCloud_) = true;
			}
			if(octoMapFrontierCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudFrontier;
				pcl::copyPointCloud(*cloud, *frontierIndices, cloudFrontier);
				pcl::toROSMsg(cloudFrontier, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapFrontierCloud_.publish(msg);
				latched_.at(&octoMapFrontierCloud_) = true;
			}
			if(octoMapObstacleCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudObstacles;
				pcl::copyPointCloud(*cloud, *obstacleIndices, cloudObstacles);
				pcl::toROSMsg(cloudObstacles, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapObstacleCloud_.publish(msg);
				latched_.at(&octoMapObstacleCloud_) = true;
			}
			if(octoMapGroundCloud_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudGround;
				pcl::copyPointCloud(*cloud, *groundIndices, cloudGround);
				pcl::toROSMsg(cloudGround, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapGroundCloud_.publish(msg);
				latched_.at(&octoMapGroundCloud_) = true;
			}
			if(octoMapEmptySpace_.getNumSubscribers())
			{
				pcl::PointCloud<pcl::PointXYZRGB> cloudEmptySpace;
				pcl::copyPointCloud(*cloud, *emptyIndices, cloudEmptySpace);
				pcl::toROSMsg(cloudEmptySpace, msg);
				msg.header.frame_id = mapFrameId;
				msg.header.stamp = stamp;
				octoMapEmptySpace_.publish(msg);
				latched_.at(&octoMapEmptySpace_) = true;
			}
			
		}
		if(octoMapProj_.getNumSubscribers())
		{
			// create the projection map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = octomap_->createProjectionMap(xMin, yMin, gridCellSize, occupancyGrid_->getMinMapSize(), octomapTreeDepth_);

			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
				map.info.resolution = gridCellSize;
				map.info.origin.position.x = 0.0;
				map.info.origin.position.y = 0.0;
				map.info.origin.position.z = 0.0;
				map.info.origin.orientation.x = 0.0;
				map.info.origin.orientation.y = 0.0;
				map.info.origin.orientation.z = 0.0;
				map.info.origin.orientation.w = 1.0;

				map.info.width = pixels.cols;
				map.info.height = pixels.rows;
				map.info.origin.position.x = xMin;
				map.info.origin.position.y = yMin;
				map.data.resize(map.info.width * map.info.height);

				memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);

				map.header.frame_id = mapFrameId;
				map.header.stamp = stamp;

				octoMapProj_.publish(map);
				latched_.at(&octoMapProj_) = true;
			}
			else if(poses.size())
			{
				ROS_WARN("Octomap projection map is empty! (poses=%d octomap nodes=%d). "
						"Make sure you activated \"%s\" and \"%s\" to true. "
						"See \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" for more info.",
						(int)poses.size(), (int)octomap_->octree()->size(),
						Parameters::kGrid3D().c_str(), Parameters::kGridFromDepth().c_str());
			}
		}
	}

	if( mapCacheCleanup_ &&
		octoMapPubBin_.getNumSubscribers() == 0 &&
		octoMapPubFull_.getNumSubscribers() == 0 &&
		octoMapCloud_.getNumSubscribers() == 0 &&
		octoMapFrontierCloud_.getNumSubscribers() == 0 &&
		octoMapObstacleCloud_.getNumSubscribers() == 0 &&
		octoMapGroundCloud_.getNumSubscribers() == 0 &&
		octoMapEmptySpace_.getNumSubscribers() == 0 &&
		octoMapProj_.getNumSubscribers() == 0 )
	{
		octomap_->clear();
	}

	if(octoMapPubBin_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapPubBin_) = false;
	}
	if(octoMapPubFull_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapPubFull_) = false;
	}
	if(octoMapCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapCloud_) = false;
	}
	if(octoMapFrontierCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFrontierCloud_) = false;
	}
	if(octoMapObstacleCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapObstacleCloud_) = false;
	}
	if(octoMapGroundCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapGroundCloud_) = false;
	}
	if(octoMapEmptySpace_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapEmptySpace_) = false;
	}
	if(octoMapProj_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapProj_) = false;
	}

	octomap_mtx_.unlock();
	
#endif
#endif

	bool gridUpdated = gridUpdated_;

	if( gridUpdated ||
		!latching_ ||
		(gridMapPub_.getNumSubscribers() && !latched_.at(&gridMapPub_)) ||
		(projMapPub_.getNumSubscribers() && !latched_.at(&projMapPub_)) ||
		(gridProbMapPub_.getNumSubscribers() && !latched_.at(&gridProbMapPub_)))
	{
		if(projMapPub_.getNumSubscribers())
		{
			if(parameters_.find(Parameters::kGridFromDepth()) != parameters_.end() &&
				!uStr2Bool(parameters_.at(Parameters::kGridFromDepth())))
			{
				ROS_WARN("/proj_map topic is deprecated! Subscribe to /grid_map topic "
						"instead with <param name=\"%s\" type=\"string\" value=\"true\"/>. "
						"Do \"$ rosrun rtabmap_ros rtabmap --params | grep Grid\" to see "
						"all occupancy grid parameters.",
						Parameters::kGridFromDepth().c_str());
			}
			else
			{
				ROS_WARN("/proj_map topic is deprecated! Subscribe to /grid_map topic instead.");
			}
		}

		if(gridProbMapPub_.getNumSubscribers())
		{
			// create the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = this->getGridProbMap(xMin, yMin, gridCellSize);

			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
				map.info.resolution = gridCellSize;
				map.info.origin.position.x = 0.0;
				map.info.origin.position.y = 0.0;
				map.info.origin.position.z = 0.0;
				map.info.origin.orientation.x = 0.0;
				map.info.origin.orientation.y = 0.0;
				map.info.origin.orientation.z = 0.0;
				map.info.origin.orientation.w = 1.0;

				map.info.width = pixels.cols;
				map.info.height = pixels.rows;
				map.info.origin.position.x = xMin;
				map.info.origin.position.y = yMin;
				map.data.resize(map.info.width * map.info.height);

				memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);

				map.header.frame_id = mapFrameId;
				map.header.stamp = stamp;

				if(gridProbMapPub_.getNumSubscribers())
				{
					gridProbMapPub_.publish(map);
					latched_.at(&gridProbMapPub_) = true;
				}
			}
			else if(poses.size())
			{
				ROS_WARN("Grid map is empty! (local maps=%d)", (int)gridMaps_.size());
			}
		}
		if(gridMapPub_.getNumSubscribers() || projMapPub_.getNumSubscribers())
		{
			// create the grid map
			float xMin=0.0f, yMin=0.0f, gridCellSize = 0.05f;
			cv::Mat pixels = this->getGridMap(xMin, yMin, gridCellSize);

			if(!pixels.empty())
			{
				//init
				nav_msgs::OccupancyGrid map;
				map.info.resolution = gridCellSize;
				map.info.origin.position.x = 0.0;
				map.info.origin.position.y = 0.0;
				map.info.origin.position.z = 0.0;
				map.info.origin.orientation.x = 0.0;
				map.info.origin.orientation.y = 0.0;
				map.info.origin.orientation.z = 0.0;
				map.info.origin.orientation.w = 1.0;

				map.info.width = pixels.cols;
				map.info.height = pixels.rows;
				map.info.origin.position.x = xMin;
				map.info.origin.position.y = yMin;
				map.data.resize(map.info.width * map.info.height);

				memcpy(map.data.data(), pixels.data, map.info.width * map.info.height);

				map.header.frame_id = mapFrameId;
				map.header.stamp = stamp;

				if(gridMapPub_.getNumSubscribers())
				{
					gridMapPub_.publish(map);
					latched_.at(&gridMapPub_) = true;
				}
				if(projMapPub_.getNumSubscribers())
				{
					projMapPub_.publish(map);
					latched_.at(&projMapPub_) = true;
				}
			}
			else if(poses.size())
			{
				ROS_WARN("Grid map is empty! (local maps=%d)", (int)gridMaps_.size());
			}
		}
	}

	if(gridMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&gridMapPub_) = false;
	}
	if(projMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&projMapPub_) = false;
	}
	if(gridProbMapPub_.getNumSubscribers() == 0)
	{
		latched_.at(&gridProbMapPub_) = false;
	}

	if(!this->hasSubscribers() && mapCacheCleanup_)
	{
		gridMaps_.clear();
		gridMapsViewpoints_.clear();		
	}
	grid_mtx_.unlock();
}

cv::Mat MapsManager::getGridMap(
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = occupancyGrid_->getCellSize();
	return occupancyGrid_->getMap(xMin, yMin);
}

cv::Mat MapsManager::getGridProbMap(
		float & xMin,
		float & yMin,
		float & gridCellSize)
{
	gridCellSize = occupancyGrid_->getCellSize();
	return occupancyGrid_->getProbMap(xMin, yMin);
}

// JHUAPL section

void MapsManager::publishAPLMaps(
		const ros::Time & stamp,
		const std::string & mapFrameId)
{
	UDEBUG("Publishing APL maps...");

	if(!semanticSegmentationEnable_)
	{
		UERROR("THIS FUNCTION SHOULD HAVE \"semanticSegmentationEnable_\" SET TO TRUE !!!");
	}

#ifdef WITH_OCTOMAP_MSGS
#ifdef RTABMAP_OCTOMAP
	octomap_u_mtx_.lock();
	bool octomapUpdated = octomapUpdated_;
	octomap_u_mtx_.unlock();

	if(octomapUpdated || 
		!latching_ ||
		(octoMapPubBin_.getNumSubscribers() && !latched_.at(&octoMapPubBin_)) ||
		(octoMapPubFull_.getNumSubscribers() && !latched_.at(&octoMapPubFull_)) ||
		(octoMapFullGroundPub_.getNumSubscribers() && !latched_.at(&octoMapFullGroundPub_)) ||
		(octoMapFullCeilingPub_.getNumSubscribers() && !latched_.at(&octoMapFullCeilingPub_)) ||
		(octoMapFullStaticPub_.getNumSubscribers() && !latched_.at(&octoMapFullStaticPub_)) ||
		(octoMapFullMovablePub_.getNumSubscribers() && !latched_.at(&octoMapFullMovablePub_)) ||
		(octoMapFullDynamicPub_.getNumSubscribers() && !latched_.at(&octoMapFullDynamicPub_)) ||
		(octoMapCloud_.getNumSubscribers() && !latched_.at(&octoMapCloud_)))
	{
		octomap_mtx_.lock();
		std::map<std::string, int> octreeName2OctreeId = semanticOctomap_->getOctreeName2OctreeId();
		std::map<int, std::string> octreeId2OctreeName = semanticOctomap_->getOctreeId2OctreeName();	
		// octoMapFullGroundb_ publishes ground layer
		SemanticOctoMap::MultiLevelOctrees mlOctrees = semanticOctomap_->multiLevelOctrees();

		// make a local copy the octres that need to be publish in its corresponding index layer
		SemanticOctoMap::MultiLevelOctrees mlOctreesTemp;
		for(auto nter = mlOctrees.begin(); nter != mlOctrees.end(); ++nter)
		{
			int layerId = nter->first;
			RtabmapAPLColorOcTree* octreePtr = nter->second;
			RtabmapAPLColorOcTree* newOcTree = new RtabmapAPLColorOcTree(octreePtr->getResolution());
			std::string octreeName = octreePtr->getOctTreeName();
			
			// only copy the layers that would be used for publishing.
			bool copyOctree = false;
			auto octreeId2OctreeNamePtr = octreeId2OctreeName.find(layerId);
			UASSERT(octreeId2OctreeNamePtr != octreeId2OctreeName.end());

			if(octoMapFullGroundPub_.getNumSubscribers() > 0 && octreeId2OctreeNamePtr->second == "ground")
				copyOctree = true;
			else if(octoMapFullCeilingPub_.getNumSubscribers() > 0 && octreeId2OctreeNamePtr->second == "ceiling")
				copyOctree = true;
			else if(octoMapFullStaticPub_.getNumSubscribers() > 0 && octreeId2OctreeNamePtr->second == "static")
				copyOctree = true;
			else if(octoMapFullMovablePub_.getNumSubscribers() > 0 && octreeId2OctreeNamePtr->second == "movable")
				copyOctree = true;
			else if(octoMapFullDynamicPub_.getNumSubscribers() > 0 && octreeId2OctreeNamePtr->second == "dynamic")
				copyOctree = true;
			else if(octoMapCloud_.getNumSubscribers() > 0)
				copyOctree = true;

			if(copyOctree)
			{
				float maxValue = std::numeric_limits<float>::max();
				float minValue = -1*std::numeric_limits<float>::max();
				Eigen::Vector3f maxBoundMap(maxValue, maxValue, maxValue);
				Eigen::Vector3f minBoundMap(minValue, minValue, minValue);

				UDEBUG("copying %d", layerId);
				// copy the whole octree into a local tree temporally
				std::list<std::string> multiLevelOctreeName = {octreeId2OctreeNamePtr->second};
				semanticOctomap_->multiOctreesToMergeOctree(newOcTree, multiLevelOctreeName, minBoundMap, maxBoundMap);
			}
			newOcTree->setOctTreeName(octreeName);

			mlOctreesTemp.insert({layerId, newOcTree});
		}

		RtabmapAPLColorOcTree m_octree(0.05);
		if(octoMapPubFull_.getNumSubscribers() > 0 || octoMapPubBin_.getNumSubscribers() > 0)
		{	
			m_octree.setOccupancyThres(0.5);
			m_octree.setProbHit(0.7);
			m_octree.setProbMiss(0.4);
			m_octree.setClampingThresMin(0.1192);
			m_octree.setClampingThresMax(0.971);
			std::string octreeName = "Dynamic_Map";	// map named expected by RTK
			m_octree.setOctTreeName(octreeName);

			std::list<std::string> multiLevelOctreeName = {"static","movable","dynamic"};

			float maxValue = std::numeric_limits<float>::max();
			float minValue = -1*std::numeric_limits<float>::max();
			Eigen::Vector3f maxBoundMap(maxValue, maxValue, gridMaxObstacleHeight_);
			Eigen::Vector3f minBoundMap(minValue, minValue, minValue);

			// multiOctreesToMergeOctree uses the actual map, needs to be locked
			semanticOctomap_->multiOctreesToMergeOctree(&m_octree, multiLevelOctreeName, minBoundMap, maxBoundMap);
		}
		octomap_mtx_.unlock();

		auto octreeName2OctreeIdPtr = octreeName2OctreeId.find("ground");
		UASSERT(octreeName2OctreeIdPtr != octreeName2OctreeId.end());
		if(mlOctreesTemp[octreeName2OctreeIdPtr->second] && octoMapFullGroundPub_.getNumSubscribers() > 0)
		{
			octomap_msgs::Octomap msg;
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;

			if(octomap_msgs::fullMapToMsg(*mlOctreesTemp[octreeName2OctreeIdPtr->second], msg))
				octoMapFullGroundPub_.publish(msg);
			else
				ROS_ERROR("ERROR serializing Octomap (%d)", 0);
		
			latched_.at(&octoMapFullGroundPub_) = true;
		}
		// octoMapFullCeilingPub_ publishes ceiling layer
		octreeName2OctreeIdPtr = octreeName2OctreeId.find("ceiling");
		UASSERT(octreeName2OctreeIdPtr != octreeName2OctreeId.end());
		if(mlOctreesTemp[octreeName2OctreeIdPtr->second] && octoMapFullCeilingPub_.getNumSubscribers() > 0)
		{
			octomap_msgs::Octomap msg;
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;

			if(octomap_msgs::fullMapToMsg(*mlOctreesTemp[octreeName2OctreeIdPtr->second], msg))
				octoMapFullCeilingPub_.publish(msg);
			else
				ROS_ERROR("ERROR serializing Octomap (%d)", 1);

			latched_.at(&octoMapFullCeilingPub_) = true;
		}
		// octoMapFullStaticPub_ publishes static layer
		octreeName2OctreeIdPtr = octreeName2OctreeId.find("static");
		UASSERT(octreeName2OctreeIdPtr != octreeName2OctreeId.end());
		if(mlOctreesTemp[octreeName2OctreeIdPtr->second] && octoMapFullStaticPub_.getNumSubscribers() > 0)
		{
			octomap_msgs::Octomap msg;
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			
			if(octomap_msgs::fullMapToMsg(*mlOctreesTemp[octreeName2OctreeIdPtr->second], msg))
				octoMapFullStaticPub_.publish(msg);
			else
				ROS_ERROR("ERROR serializing Octomap (%d)", 2);

			latched_.at(&octoMapFullStaticPub_) = true;
		}
		// octoMapFullMovablePub_ publishes movable layer
		octreeName2OctreeIdPtr = octreeName2OctreeId.find("movable");
		UASSERT(octreeName2OctreeIdPtr != octreeName2OctreeId.end());
		if(mlOctreesTemp[octreeName2OctreeIdPtr->second] && octoMapFullMovablePub_.getNumSubscribers() > 0)
		{
			octomap_msgs::Octomap msg;
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			
			if(octomap_msgs::fullMapToMsg(*mlOctreesTemp[octreeName2OctreeIdPtr->second], msg))
				octoMapFullMovablePub_.publish(msg);
			else
				ROS_ERROR("ERROR serializing Octomap (%d)", 3);
			
			latched_.at(&octoMapFullMovablePub_) = true;
		}
		// octoMapFullDynamicPub_ publishes dynamic layer
		octreeName2OctreeIdPtr = octreeName2OctreeId.find("dynamic");
		UASSERT(octreeName2OctreeIdPtr != octreeName2OctreeId.end());
		if(mlOctreesTemp[octreeName2OctreeIdPtr->second] && octoMapFullDynamicPub_.getNumSubscribers() > 0)
		{
			octomap_msgs::Octomap msg;
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
		
			if(octomap_msgs::fullMapToMsg(*mlOctreesTemp[octreeName2OctreeIdPtr->second], msg))
				octoMapFullDynamicPub_.publish(msg);
			else
				ROS_ERROR("ERROR serializing Octomap (%d)", 4);
			
			latched_.at(&octoMapFullDynamicPub_) = true;
		}
		// octoMapCloud_ publishes all layers as single point cloud (include node's data)
		if(octoMapCloud_.getNumSubscribers())
		{
			sensor_msgs::PointCloud2 msg;
			std::list<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds;
			
			///
			/// TODO: not ideal to have data in separate locks, data mayb change in between locks.   
			///
			octomap_mtx_.lock();
			for (auto n : octreeName2OctreeId) {
				pcl::IndicesPtr occupiedIndices(new std::vector<int>);
				pcl::IndicesPtr emptyIndices(new std::vector<int>);

				std::string layerName = n.first;
				UDEBUG("octree name: \"%s\"", layerName.c_str());
				pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = semanticOctomap_->createCloud(layerName, octomapTreeDepth_, occupiedIndices.get(), emptyIndices.get(), true);

				clouds.push_back(cloud);
			}
			octomap_mtx_.unlock();
			
			pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOccupiedSpacePtr;
			cloudOccupiedSpacePtr = util3d::concatenateClouds(clouds);
			
			pcl::toROSMsg(*cloudOccupiedSpacePtr, msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapCloud_.publish(msg);
			latched_.at(&octoMapCloud_) = true;
		}
		// octoMapPubFull_ publishes layers {static,movable,dynamic} as OcTreeDist full map (include node's data)
		if(octoMapPubFull_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::fullMapToMsg(m_octree, msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubFull_.publish(msg);
			latched_.at(&octoMapPubFull_) = true;

		}
		// octoMapPubBin_ publishes layers {static,movable,dynamic} as OcTreeDist Binary map
		if(octoMapPubBin_.getNumSubscribers())
		{
			octomap_msgs::Octomap msg;
			octomap_msgs::binaryMapToMsg(m_octree, msg);
			msg.header.frame_id = mapFrameId;
			msg.header.stamp = stamp;
			octoMapPubBin_.publish(msg);
			latched_.at(&octoMapPubBin_) = true;
		}

		// remove local copy of octree from memory
		for(auto nter = mlOctreesTemp.begin(); nter != mlOctreesTemp.end(); ++nter)
		{
			delete nter->second;
		}
		mlOctreesTemp.clear();
	}
	
	if( mapCacheCleanup_ &&
		octoMapPubBin_.getNumSubscribers() == 0 &&
		octoMapPubFull_.getNumSubscribers() == 0 &&
		octoMapCloud_.getNumSubscribers() == 0 &&
		octoMapFullGroundPub_.getNumSubscribers() == 0 &&
		octoMapFullCeilingPub_.getNumSubscribers() == 0 &&
		octoMapFullStaticPub_.getNumSubscribers() == 0 &&
		octoMapFullMovablePub_.getNumSubscribers() == 0 &&
		octoMapFullDynamicPub_.getNumSubscribers() == 0 )
	{
		octomap_mtx_.lock();
		semanticOctomap_->clear();
		octomap_mtx_.unlock();
	}

	if(octoMapPubBin_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapPubBin_) = false;
	}
	if(octoMapPubFull_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapPubFull_) = false;
	}
	if(octoMapFullGroundPub_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFullGroundPub_) = false;
	}
	if(octoMapFullCeilingPub_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFullCeilingPub_) = false;
	}
	if(octoMapFullStaticPub_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFullStaticPub_) = false;
	}
	if(octoMapFullMovablePub_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFullMovablePub_) = false;
	}
	if(octoMapFullDynamicPub_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapFullDynamicPub_) = false;
	}
	if(octoMapCloud_.getNumSubscribers() == 0)
	{
		latched_.at(&octoMapCloud_) = false;
	}

#endif
#endif

	if(!this->hasSubscribers() && mapCacheCleanup_)
	{
		grid_mtx_.lock();
		gridAPLMaps_.clear();
		gridMapsViewpoints_.clear();
		grid_mtx_.unlock();
	}
}

void MapsManager::publishSemenaticMaskImage(const rtabmap::SensorData & data)
{
	if(semanticMaskPub_.getNumSubscribers())
	{
		cv::Mat rgb, depth;
		cv::Mat semanticMask;
		if(!data.imageSemanticMaskRaw().empty())
		{
			semanticMask = data.imageSemanticMaskRaw();
		}
		else 
		{
			data.uncompressDataConst(&rgb, &depth, &semanticMask);
		}

		if(semanticMask.empty()) 
		{
			ROS_ERROR("semantic mask is empty!! ");
			return;
		}

		if(semanticMask.channels() != 1)
		{
			ROS_ERROR("semantic mask should have the label number in only one channel");
			return;
		}

		std::vector<rtabmap::CameraModel> cameraModels = data.cameraModels();
		// This function only supports for one camera model
		if (cameraModels.size() > 1)
		{
			ROS_ERROR("publishing semantic mask only supports one camera model");
			return;
		}

		std::map<unsigned int, cv::Point3f> classIDSemanticMaskMap;
		semanticOctomap_->getMaskIdColorMap(classIDSemanticMaskMap);

		// apply maskcolor to semantic mask image
		cv::Mat semanticMaskImage = cv::Mat::zeros(semanticMask.rows, semanticMask.cols, CV_8UC3);

		for(int h = 0; h < semanticMaskImage.rows; ++h) {
			for(int w = 0; w < semanticMaskImage.cols; ++w) {
				std::map<unsigned int, cv::Point3f>::iterator classIDSemanticMaskMapIter = 
					classIDSemanticMaskMap.find(semanticMask.at<uint8_t>(h, w));
				if(classIDSemanticMaskMapIter != classIDSemanticMaskMap.end())
		 		{
					// classIDSemanticMaskMap is in RGB but ROS needs it in BGR
					// (b,g,r) = (r,g,b) <==> (z,y,x) <==> (x,y,z)
					semanticMaskImage.at<cv::Vec3b>(h, w)[0] =  classIDSemanticMaskMapIter->second.z; // blue
					semanticMaskImage.at<cv::Vec3b>(h, w)[1] =  classIDSemanticMaskMapIter->second.y; // green
					semanticMaskImage.at<cv::Vec3b>(h, w)[2] =  classIDSemanticMaskMapIter->second.x; // red
				}
			}
		}

		sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", semanticMaskImage).toImageMsg();
		semanticMaskPub_.publish(msg);
	}
}

std::map<unsigned int, std::string> MapsManager::getOccupancyAssociation() 
{
	std::map<unsigned int, std::string> occupancyAssociationMap;
	// data gets written to argument
	occupancyGrid_->getOccupancyAssociation(&occupancyAssociationMap);

	return std::move(occupancyAssociationMap);
}

// JHUAPL section end
