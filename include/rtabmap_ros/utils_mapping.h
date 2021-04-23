/*
*   Copyright 2021 The Johns Hopkins University
*   Applied Physics Laboratory.  All rights reserved.
*/


#ifndef UTILS_MAPPING_H_
#define UTILS_MAPPING_H_

// system
#include <map>
#include <string>

// opencv
#include <opencv2/core.hpp>

#ifdef WITH_YAMLCPP
#include <yaml-cpp/yaml.h> 
#endif

namespace utils
{
    ///
    /// @brief helper function parse yaml file into a node
    ///
    bool parseConfig(std::string filePath, YAML::Node & node);

    ///
    /// @brief helper function for parsing out model class id, type, and color mask
    ///
    bool parseModelConfig(std::string filePath,
                        std::map< std::string, std::map<unsigned int, std::string>> & modelMap, 
                        std::map<unsigned int, cv::Point3f> & modelMaskIdColorMap);

    ///
    /// @brief edge filder for depth image, based on removing depths with neights distance too far out,
    ///         identifying end of plane 
    ///
    void depthEdgeFilter(cv::Mat& depthImg);

    ///
    /// @brief calculates the median of cv::Mat matrix with a single channel with up to a range of 65536
    ///
    double medianMat(cv::Mat& input);

}


#endif /* UTILS_MAPPING_H_ */