/*
*   Johns Hopkins University Applied Physics Laboratory
*/


#ifndef UTILS_MAPPING_H_
#define UTILS_MAPPING_H_

// system
#include <map>
#include <string>

// opencv
#include <opencv2/core.hpp>

namespace utils
{

    /*
    *
    * */
    bool parseModelConfig(std::string filePath, std::map< std::string, std::map<unsigned int, std::string> > &modelMap);

    /*
    *
    * */
    void depthEdgeFilter(cv::Mat& depthImg);

    /*
    * @brief calculates the median of cv::Mat matrix with a single channel with up to a range of 65536
    * 
    */
    double medianMat(cv::Mat& input);

}


#endif /* UTILS_MAPPING_H_ */