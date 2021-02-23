/*
*   Johns Hopkins University Applied Physics Laboratory
*/


#ifndef UTILS_MAPPING_H_
#define UTILS_MAPPING_H_

// system
#include <map>
#include <string>

namespace utils
{

    /*
    *
    * */
    bool parseModelConfig(std::string filePath, std::map< std::string, std::map<unsigned int, std::string> > &modelMap);


}


#endif /* UTILS_MAPPING_H_ */