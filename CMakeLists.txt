cmake_minimum_required(VERSION 2.8.3)
project(rtabmap_ros)

SET(RTABMAP_ROS_PATH "${PROJECT_SOURCE_DIR}/third_party/rtabmap_ros")

# Policy CMP0043 introduced in cmake version 3.0 IGNORES the use of COMPILE_DEFINITIONS suffixed variables, e.g. COMPILE_DEFINITIONS_DEBUG
# Set to OLD behavior until minimum cmake version >= 2.8.10 (version that COMPILE_DEFINITIONS can be set by generator expressions instead)
if (POLICY CMP0043)
    cmake_policy(SET CMP0043 OLD)
endif (POLICY CMP0043)
# Policy CMP0042 introduced in cmake version 3.0 enables the use of @rpath in an install name via MACOSX_RPATH by default
# Set to OLD behavior so that all versions use the same behavior, or until minimum cmake version >= 2.8.12 (version where @rpath is available)
if (POLICY CMP0042)
    cmake_policy(SET CMP0042 OLD)
endif (POLICY CMP0042)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
             cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
             image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
             pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs
             genmsg stereo_msgs move_base_msgs image_geometry pluginlib objectrecognition
)

set(CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} /usr/local/rtabmap_apl/lib/cmake/rtabmap_apl)

# Optional components
find_package(costmap_2d)
find_package(octomap_msgs)
find_package(apriltag_ros)
find_package(rviz)
find_package(find_object_2d)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(rtabmap_apl 0.20.0 REQUIRED)
message(STATUS "rtabmap_apl_INCLUDE_DIRS=${rtabmap_apl_INCLUDE_DIRS}")
message(STATUS "rtabmap_apl_LIBRARIES=${rtabmap_apl_LIBRARIES}")

find_package(OpenCV 3.4.10 REQUIRED)
message(STATUS "OpenCV_INCLUDE_DIRS=${OpenCV_INCLUDE_DIRS}")
message(STATUS "OpenCV_LIBRARIES=${OpenCV_LIBRARIES}")

find_package(fmt REQUIRED)

option(RTABMAP_GUI "rtabmap_apl library librtabmap_gui.so" OFF)

IF(RTABMAP_GUI)
FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation visualization)
ELSE()
FIND_PACKAGE(PCL 1.7 REQUIRED QUIET COMPONENTS common io kdtree search surface filters registration sample_consensus segmentation )
ENDIF()
add_definitions(${PCL_DEFINITIONS}) # To include -march=native if set

IF(WIN32)
add_compile_options(-bigobj)
ENDIF(WIN32)

option(RTABMAP_SYNC_MULTI_RGBD "Build with multi RGBD camera synchronization support"  OFF)
option(RTABMAP_SYNC_USER_DATA "Build with input user data support"  OFF)
MESSAGE(STATUS "RTABMAP_SYNC_MULTI_RGBD = ${RTABMAP_SYNC_MULTI_RGBD}")
MESSAGE(STATUS "RTABMAP_SYNC_USER_DATA  = ${RTABMAP_SYNC_USER_DATA}")
IF(RTABMAP_SYNC_MULTI_RGBD)
add_definitions("-DRTABMAP_SYNC_MULTI_RGBD")
ENDIF(RTABMAP_SYNC_MULTI_RGBD)
IF(RTABMAP_SYNC_USER_DATA)
add_definitions("-DRTABMAP_SYNC_USER_DATA")
ENDIF(RTABMAP_SYNC_USER_DATA)

option(OPENCV_CUDA "Build with OPENCV CUDA support" OFF)
IF(OPENCV_CUDA)
add_definitions("-DOPENCV_CUDA")
ENDIF(OPENCV_CUDA)

option(WITH_YAMLCPP "Include Yaml-cpp for parsing model file class architecture"  OFF)

if(WITH_YAMLCPP)
   find_package(yaml-cpp QUIET)  
   if(yaml-cpp_FOUND)
      find_library(YAML_CPP_LIBRARIES_PATH
         ${YAML_CPP_LIBRARIES}
      )
      message(STATUS "FOUND yaml-cpp: ${YAML_CPP_LIBRARIES_PATH}")
      add_definitions("-DWITH_YAMLCPP")
   endif(yaml-cpp_FOUND)
endif(WITH_YAMLCPP)

#Qt stuff
# If librtabmap_gui.so is found, rtabmapviz will be built
# If rviz is found, plugins will be built
IF(RTABMAP_GUI OR rviz_FOUND)
   IF(RTABMAP_QT_VERSION EQUAL 4)
      FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
      INCLUDE(${QT_USE_FILE})
   ELSE()
      IF(RTABMAP_GUI)
         FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui REQUIRED)
      ELSE()
         # For rviz plugins, look for Qt5 before Qt4
         FIND_PACKAGE(Qt5 COMPONENTS Widgets Core Gui QUIET)
         IF(NOT Qt5_FOUND)
            FIND_PACKAGE(Qt4 COMPONENTS QtCore QtGui REQUIRED)
            INCLUDE(${QT_USE_FILE})
         ENDIF(NOT Qt5_FOUND)
      ENDIF()
   ENDIF()   
ENDIF(RTABMAP_GUI OR rviz_FOUND)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

#######################################
## Declare ROS messages and services ##
#######################################

## Generate messages in the 'msg' folder
add_message_files(
   DIRECTORY ${RTABMAP_ROS_PATH}/msg
   FILES
   Info.msg
   KeyPoint.msg
   GlobalDescriptor.msg
   ScanDescriptor.msg
   MapData.msg
   MapGraph.msg
   NodeData.msg
   Link.msg
   OdomInfo.msg
   Point2f.msg
   Point3f.msg
   Goal.msg
   RGBDImage.msg
   UserData.msg
   GPS.msg
   Path.msg
)

add_message_files(
   DIRECTORY msg
   FILES
   RGBDSemanticDetection.msg
   SemanticClassIdElement.msg 
   SemanticClassIdMap.msg
   RegisteredData.msg
   Landmark.msg
   Landmarks.msg
)

## Generate services in the 'srv' folder
 add_service_files(
   DIRECTORY ${RTABMAP_ROS_PATH}/srv
   FILES
   GetMap.srv
   ListLabels.srv
   PublishMap.srv
   ResetPose.srv
   SetGoal.srv
   SetLabel.srv
   GetPlan.srv
   AddLink.srv
   GetNodeData.srv
 )

 add_service_files(
   DIRECTORY srv
   FILES
   LandmarksQuery.srv
   LandmarksRemove.srv
   LandmarksInsert.srv
   SemanticDataAssociation.srv
   SelectedLandmarks.srv
 )

## Generate added messages and services with any dependencies listed here
generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   sensor_msgs
   nav_msgs
)

#add dynamic reconfigure api
generate_dynamic_reconfigure_options(${RTABMAP_ROS_PATH}/cfg/Camera.cfg)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

SET(optional_dependencies "")
IF(costmap_2d_FOUND)
   SET(optional_dependencies ${optional_dependencies} costmap_2d)
ENDIF(costmap_2d_FOUND)
IF(octomap_msgs_FOUND)
   SET(optional_dependencies ${optional_dependencies} octomap_msgs)
ENDIF(octomap_msgs_FOUND)
IF(rviz_FOUND)
   SET(optional_dependencies ${optional_dependencies} rviz)
ENDIF(rviz_FOUND)
IF(find_object_2d_FOUND)
   SET(optional_dependencies ${optional_dependencies} find_object_2d)
ENDIF(find_object_2d_FOUND)

catkin_package(
  INCLUDE_DIRS ${RTABMAP_ROS_PATH}/include include
  LIBRARIES rtabmap_ros rtabmap_sync rtabmap_plugins
  CATKIN_DEPENDS cv_bridge roscpp rospy sensor_msgs std_msgs std_srvs nav_msgs geometry_msgs visualization_msgs
                 image_transport tf tf_conversions tf2_ros eigen_conversions laser_geometry pcl_conversions 
                 pcl_ros nodelet dynamic_reconfigure message_filters class_loader rosgraph_msgs pluginlib
                 stereo_msgs move_base_msgs image_geometry ${optional_dependencies} objectrecognition 
  DEPENDS rtabmap_apl OpenCV
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
   include
   ${RTABMAP_ROS_PATH}/include
   ${rtabmap_apl_INCLUDE_DIRS}          
   ${OpenCV_INCLUDE_DIRS}
   ${PCL_INCLUDE_DIRS}
   ${catkin_INCLUDE_DIRS}
)

# libraries
SET(Libraries
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
   ${catkin_LIBRARIES}
   ${rtabmap_apl_LIBRARIES}
   fmt::fmt   
)
 
SET(rtabmap_sync_lib_src
   #${RTABMAP_ROS_PATH}/src/CommonDataSubscriber.cpp
   src/CommonDataSubscriber.cpp
   ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberDepth.cpp
   ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberStereo.cpp
   ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberRGB.cpp
   ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberRGBD.cpp
   ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberScan.cpp
   ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberOdom.cpp
   src/impl/CommonDataSubscriberRGBDSemanticDetection.cpp
   #${RTABMAP_ROS_PATH}/src/CoreWrapper.cpp # we put CoreWrapper here instead of plugins lib to avoid long compilation time on plugins lib
   src/CoreWrapper.cpp
)
IF(RTABMAP_SYNC_MULTI_RGBD)
  SET(rtabmap_sync_lib_src
    ${rtabmap_sync_lib_src}
    ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberRGBD2.cpp
    ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberRGBD3.cpp
    ${RTABMAP_ROS_PATH}/src/impl/CommonDataSubscriberRGBD4.cpp
  )
ENDIF(RTABMAP_SYNC_MULTI_RGBD)
  
SET(rtabmap_ros_lib_src
   #${RTABMAP_ROS_PATH}/src/MsgConversion.cpp
   #${RTABMAP_ROS_PATH}/src/MapsManager.cpp
   src/MsgConversion.cpp
   src/MapsManager.cpp
   ${RTABMAP_ROS_PATH}/src/OdometryROS.cpp
   ${RTABMAP_ROS_PATH}/src/PluginInterface.cpp
   src/utils_mapping.cpp
)
  
SET(rtabmap_plugins_lib_src
   ${RTABMAP_ROS_PATH}/src/nodelets/rgbd_odometry.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/stereo_odometry.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/rgbdicp_odometry.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/icp_odometry.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/data_throttle.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/stereo_throttle.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/data_odom_sync.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/point_cloud_xyzrgb.cpp 
   ${RTABMAP_ROS_PATH}/src/nodelets/point_cloud_xyz.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/disparity_to_depth.cpp 
   ${RTABMAP_ROS_PATH}/src/nodelets/pointcloud_to_depthimage.cpp 
   ${RTABMAP_ROS_PATH}/src/nodelets/obstacles_detection.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/obstacles_detection_old.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/point_cloud_aggregator.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/point_cloud_assembler.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/undistort_depth.cpp
   ${RTABMAP_ROS_PATH}/src/nodelets/imu_to_tf.cpp
)

IF(${cv_bridge_VERSION_MAJOR} GREATER 1 OR ${cv_bridge_VERSION_MINOR} GREATER 10)
   SET(rtabmap_plugins_lib_src 
   ${rtabmap_plugins_lib_src} 
   ${RTABMAP_ROS_PATH}/src/nodelets/rgbd_sync.cpp 
   ${RTABMAP_ROS_PATH}/src/nodelets/stereo_sync.cpp 
   ${RTABMAP_ROS_PATH}/src/nodelets/rgbd_relay.cpp
   src/nodelets/rgbd_semantic_detection_sync.cpp)
ELSE()
   ADD_DEFINITIONS("-DCV_BRIDGE_HYDRO")
ENDIF()   

# If octomap is found, add definition
IF(octomap_msgs_FOUND)
MESSAGE(STATUS "WITH octomap_msgs")
include_directories(
  ${octomap_msgs_INCLUDE_DIRS}
)
SET(Libraries
  ${octomap_msgs_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_OCTOMAP_MSGS")
ENDIF(octomap_msgs_FOUND)

# If apriltag_ros is found, add definition
IF(apriltag_ros_FOUND)
MESSAGE(STATUS "WITH apriltag_ros")
include_directories(
  ${apriltag_ros_INCLUDE_DIRS}
)
SET(Libraries
  ${apriltag_ros_LIBRARIES}
  ${Libraries}
)
ADD_DEFINITIONS("-DWITH_APRILTAG_ROS")
ENDIF(apriltag_ros_FOUND)

if(yaml-cpp_FOUND)
   include_directories(
      ${YAML_CPP_INCLUDE_DIR}
   )
   set(Libraries
      ${YAML_CPP_LIBRARIES}
      ${Libraries}
   )
endif(yaml-cpp_FOUND)

############################
## Declare a cpp library
############################
add_library(rtabmap_sync
   ${rtabmap_sync_lib_src}
)
add_library(rtabmap_ros
   ${rtabmap_ros_lib_src}
)
add_library(rtabmap_plugins
   ${rtabmap_plugins_lib_src}
)

target_link_libraries(rtabmap_sync
  rtabmap_ros
)
target_link_libraries(rtabmap_ros
  ${Libraries}
)
target_link_libraries(rtabmap_plugins
  rtabmap_ros
)
add_dependencies(rtabmap_ros ${${PROJECT_NAME}_EXPORTED_TARGETS})
add_dependencies(rtabmap_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(rtabmap ${RTABMAP_ROS_PATH}/src/CoreNode.cpp)
target_link_libraries(rtabmap ${Libraries})

add_executable(rtabmap_rgbd_odometry ${RTABMAP_ROS_PATH}/src/RGBDOdometryNode.cpp)
target_link_libraries(rtabmap_rgbd_odometry ${Libraries})
set_target_properties(rtabmap_rgbd_odometry PROPERTIES OUTPUT_NAME "rgbd_odometry")

add_executable(rtabmap_stereo_odometry ${RTABMAP_ROS_PATH}/src/StereoOdometryNode.cpp)
target_link_libraries(rtabmap_stereo_odometry ${Libraries})
set_target_properties(rtabmap_stereo_odometry PROPERTIES OUTPUT_NAME "stereo_odometry")

add_executable(rtabmap_rgbdicp_odometry ${RTABMAP_ROS_PATH}/src/RGBDICPOdometryNode.cpp)
target_link_libraries(rtabmap_rgbdicp_odometry ${Libraries})
set_target_properties(rtabmap_rgbdicp_odometry PROPERTIES OUTPUT_NAME "rgbdicp_odometry")

add_executable(rtabmap_icp_odometry ${RTABMAP_ROS_PATH}/src/ICPOdometryNode.cpp)
target_link_libraries(rtabmap_icp_odometry ${Libraries})
set_target_properties(rtabmap_icp_odometry PROPERTIES OUTPUT_NAME "icp_odometry")

add_executable(rtabmap_rgbd_sync ${RTABMAP_ROS_PATH}/src/RGBDSyncNode.cpp)
target_link_libraries(rtabmap_rgbd_sync ${Libraries})
set_target_properties(rtabmap_rgbd_sync PROPERTIES OUTPUT_NAME "rgbd_sync")

add_executable(rtabmap_rgbd_relay ${RTABMAP_ROS_PATH}/src/RGBDRelayNode.cpp)
target_link_libraries(rtabmap_rgbd_relay ${Libraries})
set_target_properties(rtabmap_rgbd_relay PROPERTIES OUTPUT_NAME "rgbd_relay")

add_executable(rtabmap_map_optimizer ${RTABMAP_ROS_PATH}/src/MapOptimizerNode.cpp)
target_link_libraries(rtabmap_map_optimizer rtabmap_ros)
set_target_properties(rtabmap_map_optimizer PROPERTIES OUTPUT_NAME "map_optimizer")

# add_executable(rtabmap_map_assembler ${RTABMAP_ROS_PATH}/src/MapAssemblerNode.cpp)
# target_link_libraries(rtabmap_map_assembler rtabmap_ros)
# set_target_properties(rtabmap_map_assembler PROPERTIES OUTPUT_NAME "map_assembler")

add_executable(rtabmap_imu_to_tf ${RTABMAP_ROS_PATH}/src/ImuToTFNode.cpp)
target_link_libraries(rtabmap_imu_to_tf ${Libraries})
set_target_properties(rtabmap_imu_to_tf PROPERTIES OUTPUT_NAME "imu_to_tf")

#IF(NOT WIN32)
#add_executable(rtabmap_wifi_signal_pub ${RTABMAP_ROS_PATH}/src/WifiSignalPubNode.cpp)
#target_link_libraries(rtabmap_wifi_signal_pub rtabmap_ros)
#set_target_properties(rtabmap_wifi_signal_pub PROPERTIES OUTPUT_NAME "wifi_signal_pub")
#ENDIF(NOT WIN32)
#add_executable(rtabmap_wifi_signal_sub ${RTABMAP_ROS_PATH}/src/WifiSignalSubNode.cpp)
#target_link_libraries(rtabmap_wifi_signal_sub rtabmap_ros)
#set_target_properties(rtabmap_wifi_signal_sub PROPERTIES OUTPUT_NAME "wifi_signal_sub")

# If find_object_2d is found, add save objects example
IF(find_object_2d_FOUND)
    MESSAGE(STATUS "WITH find_object_2d")
    include_directories(${find_object_2d_INCLUDE_DIRS})
    add_executable(rtabmap_save_objects_example  ${RTABMAP_ROS_PATH}/src/SaveObjectsExample.cpp)
    target_link_libraries(rtabmap_save_objects_example ${Libraries} rtabmap_ros ${find_object_2d_LIBRARIES})
    set_target_properties(rtabmap_save_objects_example PROPERTIES OUTPUT_NAME "save_objects_example")
ENDIF(find_object_2d_FOUND)

add_executable(rtabmap_external_loop_detectionexample  ${RTABMAP_ROS_PATH}/src/ExternalLoopDetectionExample.cpp)
target_link_libraries(rtabmap_external_loop_detectionexample ${Libraries} rtabmap_ros)
set_target_properties(rtabmap_external_loop_detectionexample PROPERTIES OUTPUT_NAME "external_loop_detection_example")

add_executable(rtabmap_camera  ${RTABMAP_ROS_PATH}/src/CameraNode.cpp)
add_dependencies(rtabmap_camera ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(rtabmap_camera ${Libraries})
set_target_properties(rtabmap_camera PROPERTIES OUTPUT_NAME "camera")

add_executable(rtabmap_stereo_camera  ${RTABMAP_ROS_PATH}/src/StereoCameraNode.cpp)
target_link_libraries(rtabmap_stereo_camera rtabmap_ros)
set_target_properties(rtabmap_stereo_camera PROPERTIES OUTPUT_NAME "stereo_camera")

IF(RTABMAP_GUI)
    add_executable(rtabmapviz  ${RTABMAP_ROS_PATH}/src/GuiNode.cpp  ${RTABMAP_ROS_PATH}/src/GuiWrapper.cpp  ${RTABMAP_ROS_PATH}/src/PreferencesDialogROS.cpp)
    target_link_libraries(rtabmapviz rtabmap_sync ${QT_LIBRARIES})
    IF(Qt5_FOUND)
        QT5_USE_MODULES(rtabmapviz Widgets Core Gui)
    ENDIF()
ELSE()
    MESSAGE(WARNING "Found RTAB-Map built without its GUI library. Node rtabmapviz will not be built!")
ENDIF()

add_executable(rtabmap_data_player  ${RTABMAP_ROS_PATH}/src/DbPlayerNode.cpp)
target_link_libraries(rtabmap_data_player rtabmap_ros)
set_target_properties(rtabmap_data_player PROPERTIES OUTPUT_NAME "data_player")

add_executable(rtabmap_odom_msg_to_tf  ${RTABMAP_ROS_PATH}/src/OdomMsgToTFNode.cpp)
target_link_libraries(rtabmap_odom_msg_to_tf rtabmap_ros)
set_target_properties(rtabmap_odom_msg_to_tf PROPERTIES OUTPUT_NAME "odom_msg_to_tf")

add_executable(rtabmap_pointcloud_to_depthimage  ${RTABMAP_ROS_PATH}/src/PointCloudToDepthImageNode.cpp)
target_link_libraries(rtabmap_pointcloud_to_depthimage ${Libraries})
set_target_properties(rtabmap_pointcloud_to_depthimage PROPERTIES OUTPUT_NAME "pointcloud_to_depthimage")

add_executable(rtabmap_point_cloud_assembler  ${RTABMAP_ROS_PATH}/src/PointCloudAssemblerNode.cpp)
target_link_libraries(rtabmap_point_cloud_assembler ${Libraries})
set_target_properties(rtabmap_point_cloud_assembler PROPERTIES OUTPUT_NAME "point_cloud_assembler")

# If rviz is found, add plugins
IF(rviz_FOUND)

    ## We also use Ogre for rviz plugins
    include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
    pkg_check_modules(OGRE OGRE)
    include_directories( ${OGRE_INCLUDE_DIRS} )
    link_directories( ${OGRE_LIBRARY_DIRS} )

    MESSAGE(STATUS "WITH rviz")
    include_directories(
      ${rviz_INCLUDE_DIRS}
    )
    SET(Libraries
      ${Libraries}
      ${rviz_LIBRARIES}
      ${rviz_DEFAULT_PLUGIN_LIBRARIES}
    )
    
    ## RVIZ plugin
    IF(QT4_FOUND)
	  IF(WIN32)
        qt4_wrap_cpp(MOC_FILES
            ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.h
            ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.h
            ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.h
        )
	  ELSE()
	    qt4_wrap_cpp(MOC_FILES
         ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/OrbitOrientedViewController.h
        )
	  ENDIF()
    ELSE()
	  IF(WIN32)
        qt5_wrap_cpp(MOC_FILES
         ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.h
        )
	  ELSE()
	    qt5_wrap_cpp(MOC_FILES
         ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.h
         ${RTABMAP_ROS_PATH}/src/rviz/OrbitOrientedViewController.h
        )
	  ENDIF()
    ENDIF()
    
    # tf:message_filters, mixing boost and Qt signals
	IF(WIN32)
      set_property(
        SOURCE  ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.cpp  ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.cpp  ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.cpp
        PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
      )
	ELSE()
	  set_property(
        SOURCE  ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.cpp ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.cpp ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.cpp ${RTABMAP_ROS_PATH}/src/rviz/OrbitOrientedViewController.cpp
        PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
      )
	ENDIF()
	SET(SRC_FILES 
      ${RTABMAP_ROS_PATH}/src/rviz/MapCloudDisplay.cpp
      ${RTABMAP_ROS_PATH}/src/rviz/MapGraphDisplay.cpp
      ${RTABMAP_ROS_PATH}/src/rviz/InfoDisplay.cpp
      ${MOC_FILES}
	)
	IF(NOT WIN32)
	  SET(SRC_FILES 
	    ${SRC_FILES} 
       ${RTABMAP_ROS_PATH}/src/rviz/OrbitOrientedViewController.cpp
	  )
   ENDIF(NOT WIN32)
   
   IF(octomap_msgs_FOUND)
   IF(QT4_FOUND)
      qt4_wrap_cpp(MOC_FILES_APL
         include/rtabmap_ros/rviz/ObjectLabelOccupancyGridDisplay.h 
         include/rtabmap_ros/rviz/TableSemanticsPanel.h 
      )
   ELSE()
      qt5_wrap_cpp(MOC_FILES_APL
         include/rtabmap_ros/rviz/ObjectLabelOccupancyGridDisplay.h
         include/rtabmap_ros/rviz/TableSemanticsPanel.h  
      )
   ENDIF()

   set_property(
      SOURCE  src/rviz/ObjectLabelOccupancyGridDisplay.cpp src/rviz/TablePanel.cpp 
      PROPERTY COMPILE_DEFINITIONS QT_NO_KEYWORDS
   )

   SET(SRC_FILES
      ${SRC_FILES}
      src/rviz/ObjectLabelOccupancyGridDisplay.cpp
      src/rviz/TableSemanticsPanel.cpp
      ${MOC_FILES_APL}
   )
   ENDIF(octomap_msgs_FOUND)

   add_library(rtabmap_rviz_plugins
      ${SRC_FILES}
   )
   target_link_libraries(rtabmap_rviz_plugins
      rtabmap_ros
      ${Libraries}
   )

   IF(Qt5_FOUND)
      QT5_USE_MODULES(rtabmap_rviz_plugins Widgets Core Gui)
   ENDIF(Qt5_FOUND)
   add_dependencies(rtabmap_rviz_plugins ${${PROJECT_NAME}_EXPORTED_TARGETS})

ENDIF(rviz_FOUND)

# If costmap_2d is found, add the plugins
IF(costmap_2d_FOUND)
    MESSAGE(STATUS "WITH costmap_2d")
    IF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
      ADD_DEFINITIONS("-DCOSTMAP_2D_POINTCLOUD2")
    ENDIF(${costmap_2d_VERSION_MAJOR} GREATER 1 OR ${costmap_2d_VERSION_MINOR} GREATER 15)
    include_directories(${costmap_2d_INCLUDE_DIRS})
    add_library(rtabmap_costmap_plugins
      ${RTABMAP_ROS_PATH}/src/costmap_2d/static_layer.cpp
    )
    add_library(rtabmap_costmap_plugins2
      ${RTABMAP_ROS_PATH}/src/costmap_2d/voxel_layer.cpp
    )
    target_link_libraries(rtabmap_costmap_plugins
      ${costmap_2d_LIBRARIES}
    )
    target_link_libraries(rtabmap_costmap_plugins2
      ${costmap_2d_LIBRARIES}
    )
ENDIF(costmap_2d_FOUND)

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
catkin_install_python(PROGRAMS
   ${RTABMAP_ROS_PATH}/scripts/patrol.py
   ${RTABMAP_ROS_PATH}/scripts/objects_to_tags.py
   ${RTABMAP_ROS_PATH}/scripts/point_to_tf.py
   ${RTABMAP_ROS_PATH}/scripts/transform_to_tf.py
   ${RTABMAP_ROS_PATH}/scripts/yaml_to_camera_info.py
   scripts/insert_landmarks_test.py
   scripts/store_landmarks.py
   scripts/insert_landmarks_saved.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark executables and/or libraries for installation
install(TARGETS 
   rtabmap_sync
   rtabmap_ros
   rtabmap_plugins 
   rtabmap 
   rtabmap_rgbd_odometry 
   rtabmap_icp_odometry
   rtabmap_rgbdicp_odometry 
   rtabmap_stereo_odometry
   # rtabmap_map_assembler
   rtabmap_map_optimizer
   rtabmap_data_player
   rtabmap_odom_msg_to_tf
   rtabmap_pointcloud_to_depthimage
   rtabmap_point_cloud_assembler
   rtabmap_camera
   rtabmap_rgbd_sync
   rtabmap_rgbd_relay
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
IF(RTABMAP_GUI)
    install(TARGETS 
       rtabmapviz
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
ENDIF(RTABMAP_GUI)

## Mark cpp header files for installation
install(DIRECTORY ${RTABMAP_ROS_PATH}/include/${PROJECT_NAME} include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(FILES
   launch/rtabmap.launch
   #${RTABMAP_ROS_PATH}/launch/rgbd_mapping.launch
   #${RTABMAP_ROS_PATH}/launch/stereo_mapping.launch
   #${RTABMAP_ROS_PATH}/launch/data_recorder.launch
   #${RTABMAP_ROS_PATH}/launch/rgbd_mapping_kinect2.launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY 
   launch/config
   #${RTABMAP_ROS_PATH}/launch/data
   #${RTABMAP_ROS_PATH}/launch/demo
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

## install plugins/nodelets xml
install(FILES
   nodelet_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
IF(rviz_FOUND)
    install(TARGETS 
       rtabmap_rviz_plugins
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
    install(FILES
      rviz_plugins.xml
      DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
ENDIF(rviz_FOUND)

IF(costmap_2d_FOUND)
    install(TARGETS 
       rtabmap_costmap_plugins
       rtabmap_costmap_plugins2
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
     )
    install(FILES
      ${RTABMAP_ROS_PATH}/costmap_plugins.xml
       DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )
ENDIF(costmap_2d_FOUND)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_rtabmap.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
