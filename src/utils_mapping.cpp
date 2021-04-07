/*
*   Johns Hopkins University Applied Physics Laboratory
*/

#include "rtabmap_ros/utils_mapping.h"

#include <ros/ros.h>

// system
#include <fstream>
#include <iostream>
#include <cmath>

#ifdef WITH_YAMLCPP
#include <yaml-cpp/yaml.h> 
#endif

//opencv
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/fast_math.hpp>


namespace utils
{

    bool parseModelConfig(std::string filePath, std::map< std::string, std::map<unsigned int, std::string> > & modelMap)
    {
        std::ifstream finput;
    	finput.open(filePath, std::ifstream::in);

        if(!finput.is_open())
        {
            ROS_ERROR(" failed to open yaml file");
            return false;
        }

#ifdef WITH_YAMLCPP
        YAML::Node modelNamesConfig;
        try 
        {
            modelNamesConfig = YAML::Load(finput);
        }
        catch (const YAML::Exception& e) 
        {
            ROS_ERROR_STREAM("yaml load error: " << e.what() << "\n");
        }
             
        for(YAML::const_iterator it=modelNamesConfig.begin(); it!=modelNamesConfig.end(); ++it) 
        {
            std::string type = it->first.as<std::string>();

            std::map<std::string, unsigned int> typeObjMap = it->second.as<std::map<std::string, unsigned int>>();
            std::map<unsigned int, std::string> ObjIdMap;
            for(std::map<std::string, unsigned int>::iterator itMap=typeObjMap.begin(); itMap!=typeObjMap.end();  ++itMap)
            {
                // update the map based on the object id (key) and name (value)
                //unsigned int labelId = itMap->second;
                ObjIdMap.insert({itMap->second, itMap->first});
            }
            modelMap.insert({type, ObjIdMap});
        }   
#endif    

        finput.close();

        // check if the modelMap was populated
        if(modelMap.empty())
        {
            return false;
        }
        else
        {
           return true;
        }           
    }

    void depthEdgeFilter(cv::Mat& depthImg)
    {
        int ksize = 5;
        int ddepth = CV_32F;
        cv::Mat cv_image_dx, cv_image_dy, grad_mag;

        // computes the horizontal changes
        cv::Sobel(depthImg, cv_image_dx, ddepth, 1, 0, ksize);
        // computes the vertical cahnges
        cv::Sobel(depthImg, cv_image_dy, ddepth, 0, 1, ksize);

        // take the magnitude of the gradients 
        cv::Mat cv_image_dx_sqr, cv_image_dy_sqr;
        cv::pow(cv_image_dx, 2, cv_image_dx_sqr);
        cv::pow(cv_image_dy, 2, cv_image_dy_sqr);   
        cv::sqrt(cv_image_dx_sqr + cv_image_dy_sqr, grad_mag);

        double medianValue = utils::medianMat(grad_mag);
        
        // filtering out depth elements 
        for(int h = 0; h < depthImg.rows; ++h) 
        {
            for(int w = 0; w < depthImg.cols; ++w)
            {
                float grad_mag_hw = grad_mag.at<float>(h,w);
                if(grad_mag_hw > medianValue*2)
                {
                    depthImg.at<unsigned short>(h,w) = 0;
                }      
            }
        }
    }

    /// Based on the 
    /// https://github.com/arnaudgelas/OpenCVExamples/blob/master/cvMat/Statistics/Median/Median.cpp
    /// https://gist.github.com/heisters/9cd68181397fbd35031b
    /// 
    ///
    /// 
    double medianMat(cv::Mat& input)
    {
        int histSize = 65536;
        cv::Mat inputMat;
        if(input.type() != CV_32F)
        {
            input.convertTo(inputMat, CV_32F);
        }
        else
        {
            input.copyTo(inputMat);
        }
        
        float range[] = { 0, (float) histSize };
        const float* histRange = { range };
        bool uniform = true; 
        bool accumulate = false;
        cv::Mat hist;

        cv::calcHist(&inputMat, 1, 0, cv::Mat(), hist, 1, &histSize, &histRange, uniform, accumulate);

        int bin = 0;
        double med = -1.0;
        double m = (inputMat.rows*inputMat.cols)/2;

        for( int i = 0; i < histSize && med < 0.0; ++i )
        {
            bin += cvRound( hist.at< float >( i ) );
            if ( bin > m && med < 0.0 )
                med = i;
        }

        return med;
    }

} // end namespace utils