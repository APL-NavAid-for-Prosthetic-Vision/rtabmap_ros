/*
*   Johns Hopkins University Applied Physics Laboratory
*/

#include <rtabmap_ros/utils_mapping.h>

#include <ros/ros.h>

#ifdef WITH_YAMLCPP
#include <fstream>
#include <iostream>
#include <yaml-cpp/yaml.h> 
#endif

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
}