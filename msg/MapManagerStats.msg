#  Johns Hopkins University Applied Physics Laboratory
#
# Map Manager statitics message : system functional parameters manly for system performances. 
#

# transmitting message header , the time message was published.
std_msgs/Header header

# map_manager_update_cache_time: total time elapsed to update the updateMapCaches and passing in data. 
#                                this essentially the time it takes to process the Map Manager thread in seconds.
float32 map_manager_update_cache_time

# is_octomap_data: it's set to true when statitics were captured, othewise it is set to false. 
# octomap_update_time: it's the total time it takes to update the octomap update function.
bool is_octomap_data
float32 octomap_update_time