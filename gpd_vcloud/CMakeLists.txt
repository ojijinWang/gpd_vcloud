cmake_minimum_required(VERSION 2.8.3)
project(gpd_vcloud)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
  cmake_modules
  eigen_conversions
  geometry_msgs
  message_generation
  roscpp
  sensor_msgs
  std_msgs
  visualization_msgs
  sensor_msgs
  pcl_ros
  pcl_conversions
  nav_msgs
  std_msgs
  std_srvs
  octomap_ros
  octomap_msgs
  dynamic_reconfigure
  cv_bridge
  image_transport
  eigen_conversions
  tf_conversions
)

# PCL
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# OpenCV
find_package(OpenCV)

# custom messages
find_package(message_generation)

find_library(GENERATOR_LIB grasp_candidates_generator)
find_path(GENERATOR_LIB_INCLUDE_DIR gpg/grasp.h)
include_directories(${GENERATOR_LIB_INCLUDE_DIR})

#octomap
find_package(octomap REQUIRED)
add_definitions(-DOCTOMAP_NODEBUGOUT)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

# CAFFE
# IF(NOT EXISTS ${CAFFE_DIR})
  # SET(CAFFE_DIR "$ENV{CAFFE_DIR}")
  # SET(CAFFE_DIR "/home/andreas/software/caffe/build")
# ENDIF()
# MESSAGE("CAFFE_DIR: " ${CAFFE_DIR})
find_package(Caffe)
include_directories(${Caffe_INCLUDE_DIRS})
add_definitions(${Caffe_DEFINITIONS})

## Set compiler optimization flags
set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp")
# set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp -flto -Wno-deprecated -Wenum-compare")
# set(CMAKE_CXX_FLAGS "-DNDEBUG -O3 -fopenmp -flto -mavx -mfma -Wno-deprecated -Wenum-compare")


## Generate messages in the 'msg' folder
add_message_files(
  FILES 
  CloudIndexed.msg 
  CloudSamples.msg 
  CloudSources.msg 
  GraspConfig.msg 
  GraspConfigList.msg 
  SamplesMsg.msg
)

## Generate services in the 'srv' folder
#add_service_files(DIRECTORY srv
# FILES
#)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES 
  geometry_msgs sensor_msgs std_msgs
)


################################################
## Declare ROS dynamic reconfigure parameters ##
################################################
catkin_package(
INCLUDE_DIRS include
CATKIN_DEPENDS cmake_modules eigen_conversions geometry_msgs message_runtime roscpp sensor_msgs std_msgs
DEPENDS Eigen OpenCV PCL octomap
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

## Declare a C++ library
add_library(${PROJECT_NAME}_caffe_classifier src/${PROJECT_NAME}/caffe_classifier.cpp)
add_library(${PROJECT_NAME}_clustering src/${PROJECT_NAME}/clustering.cpp)
add_library(${PROJECT_NAME}_data_generator src/${PROJECT_NAME}/data_generator.cpp)
add_library(${PROJECT_NAME}_grasp_detector src/${PROJECT_NAME}/grasp_detector.cpp)
add_library(${PROJECT_NAME}_learning src/${PROJECT_NAME}/learning.cpp)
add_library(${PROJECT_NAME}_sequential_importance_sampling src/${PROJECT_NAME}/sequential_importance_sampling.cpp)
add_library(${PROJECT_NAME}_grasp_image src/${PROJECT_NAME}/grasp_image.cpp)
add_library(${PROJECT_NAME}_grasp_image_15_channels src/${PROJECT_NAME}/grasp_image_15_channels.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_caffe_classifier
                      ${Caffe_LIBRARIES}
                      ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_clustering
                      ${GENERATOR_LIB})
target_link_libraries(${PROJECT_NAME}_data_generator
                      ${PROJECT_NAME}_learning
                      ${GENERATOR_LIB}
                      ${Caffe_LIBRARIES}
                      ${catkin_LIBRARIES}
                      ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_detector
                      ${PROJECT_NAME}_caffe_classifier
                      ${PROJECT_NAME}_clustering
                      ${PROJECT_NAME}_learning
                      ${GENERATOR_LIB}
                      ${catkin_LIBRARIES}
                      ${Caffe_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_image
                      ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_grasp_image_15_channels
                      ${PROJECT_NAME}_grasp_image)
target_link_libraries(${PROJECT_NAME}_learning
                      ${PROJECT_NAME}_grasp_image_15_channels
                      ${GENERATOR_LIB})
target_link_libraries(${PROJECT_NAME}_sequential_importance_sampling
                      ${PROJECT_NAME}_grasp_detector)
target_link_libraries(${PROJECT_NAME}_learning
                      ${GENERATOR_LIB}
                      ${catkin_LIBRARIES}
                      ${Caffe_LIBRARIES})

## Specify additional locations of header files
add_executable(gpd_vcloud_node
		src/gpd_vcloud_node.cpp
		src/vCloud_generator.h
		src/grasp_detector_vcloud.h
		src/grasp_detector_vcloud.cpp
		src/grasp_visual_markers.h
		src/pointcloud_downsize.h
		src/sub_plot.h
)
target_link_libraries(gpd_vcloud_node ${PROJECT_NAME}_caffe_classifier ${PROJECT_NAME}_clustering ${PROJECT_NAME}_learning ${GENERATOR_LIB} ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OCTOMAP_LIBRARIES} ${Caffe_LIBRARIES})
