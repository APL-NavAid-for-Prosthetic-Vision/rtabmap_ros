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

#ifndef MAPSMANAGER_H_
#define MAPSMANAGER_H_

#include <rtabmap/core/Signature.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/FlannIndex.h>
#include <rtabmap/core/Landmark.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/time.h>
#include <ros/publisher.h>

#include <image_transport/image_transport.h>

// JHUAPL section

//boost
#include <boost/thread/mutex.hpp>

// JHUAPL section end

namespace rtabmap {
class OctoMap;
class SemanticOctoMap;
class Memory;
class OccupancyGrid;

}  // namespace rtabmap

class MapsManager {
public:
	MapsManager();
	virtual ~MapsManager();
	void init(ros::NodeHandle & nh, ros::NodeHandle & pnh, const std::string & name, bool usePublicNamespace);
	void clear();
	bool hasSubscribers() const;
	void backwardCompatibilityParameters(ros::NodeHandle & pnh, rtabmap::ParametersMap & parameters) const;
	void setParameters(const rtabmap::ParametersMap & parameters);
	void set2DMap(const cv::Mat & map, float xMin, float yMin, float cellSize, const std::map<int, rtabmap::Transform> & poses, const rtabmap::Memory * memory = 0);

	std::map<int, rtabmap::Transform> getFilteredPoses(
			const std::map<int, rtabmap::Transform> & poses);

	std::map<int, rtabmap::Transform> updateMapCaches(
			const std::map<int, rtabmap::Transform> & poses,
			const rtabmap::Memory * memory,
			bool updateGrid,
			bool updateOctomap,
			const std::map<int, rtabmap::Signature> & signatures = std::map<int, rtabmap::Signature>());

	void publishMaps(
			const std::map<int, rtabmap::Transform> & poses,
			const ros::Time & stamp,
			const std::string & mapFrameId);

	cv::Mat getGridMap(
			float & xMin,
			float & yMin,
			float & gridCellSize);

	cv::Mat getGridProbMap(
			float & xMin,
			float & yMin,
			float & gridCellSize);

	const rtabmap::OctoMap * getOctomap() const {return octomap_;}
	const rtabmap::SemanticOctoMap * getSemanticOctomap() const {return semanticOctomap_;}
	const rtabmap::OccupancyGrid * getOccupancyGrid() const {return occupancyGrid_;}

	// JHUAPL section

	bool isSemanticSegmentationEnabled() { return semanticSegmentationEnable_; }

	void publishAPLMaps(const ros::Time & stamp,
						const std::string & mapFrameId);

	void publishSemanticMask(const rtabmap::SensorData & data);
	
	// JHUAPL section end

private:
	// mapping stuff
	bool cloudOutputVoxelized_;
	bool cloudSubtractFiltering_;
	int cloudSubtractFilteringMinNeighbors_;
	double mapFilterRadius_;
	double mapFilterAngle_;
	bool mapCacheCleanup_;
	bool alwaysUpdateMap_;
	bool scanEmptyRayTracing_;

	ros::Publisher cloudMapPub_;
	ros::Publisher cloudGroundPub_;
	ros::Publisher cloudObstaclesPub_;
	ros::Publisher projMapPub_;
	ros::Publisher gridMapPub_;
	ros::Publisher gridProbMapPub_;
	ros::Publisher scanMapPub_;

	ros::Publisher octoMapPubBin_;
	ros::Publisher octoMapPubFull_;
	ros::Publisher octoMapCloud_;
	ros::Publisher octoMapFrontierCloud_;
	ros::Publisher octoMapGroundCloud_;
	ros::Publisher octoMapObstacleCloud_;
	ros::Publisher octoMapEmptySpace_;
	ros::Publisher octoMapProj_;

	// JHUAPL section
	
	ros::Publisher octoMapFullGroundPub_;
	ros::Publisher octoMapFullCeilingPub_;
	ros::Publisher octoMapFullStaticPub_;
	ros::Publisher octoMapFullMovablePub_;
	ros::Publisher octoMapFullDynamicPub_;

	// JHUAPL end section

	std::map<int, rtabmap::Transform> assembledGroundPoses_;
	std::map<int, rtabmap::Transform> assembledObstaclePoses_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledObstacles_;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr assembledGround_;
	rtabmap::FlannIndex assembledGroundIndex_;
	rtabmap::FlannIndex assembledObstacleIndex_;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > groundClouds_;
	std::map<int, pcl::PointCloud<pcl::PointXYZRGB>::Ptr > obstacleClouds_;

	std::map<int, rtabmap::Transform> gridPoses_;
	cv::Mat gridMap_;
	std::map<int, std::pair< std::pair<cv::Mat, cv::Mat>, cv::Mat> > gridMaps_; // < <ground, obstacles>, empty cells >
	std::map<int, cv::Point3f> gridMapsViewpoints_;

	rtabmap::OccupancyGrid * occupancyGrid_;
	bool gridUpdated_;

	rtabmap::OctoMap * octomap_;
	int octomapTreeDepth_;
	bool octomapUpdated_;

	rtabmap::ParametersMap parameters_;

	bool latching_;
	std::map<void*, bool> latched_;

	// JHUAPL section

	rtabmap::SemanticOctoMap * semanticOctomap_;
	bool semanticSegmentationEnable_;
	std::map<int, std::pair< std::map<unsigned int, cv::Mat>, cv::Mat > > gridAPLMaps_; // < , < map<class_label_id, object_cells>, empty_cells > >
	std::string semanticSegmentationModelFilePath_;
	bool publishSemanticMask_;
	image_transport::Publisher semanticMaskPub_;

	mutable boost::mutex octomap_mtx_;
	mutable boost::mutex octomap_u_mtx_;
	mutable boost::mutex grid_mtx_;

	// JHUAPL section end
};

#endif /* MAPSMANAGER_H_ */
