cmake_minimum_required(VERSION 2.8.3)
project(radar)
##set ( CMAKE_CXX_FLAGS "-ggdb -g")
## find catkin macros and libraries
## if components list like find_package(catkin required components xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  pcl_ros
  pcl_conversions
  visualization_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
  sensor_msgs
  stereo_msgs 
  tf2
  cv_bridge
  eigen_conversions
  image_geometry
  image_transport  
  rosbag
  laser_geometry
)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Eigen3 QUIET)
if(NOT EIGEN3_FOUND)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
endif()
find_package(OpenCV REQUIRED)
include_directories(include ${BOOST_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

add_message_files(FILES ppl.msg radar_fusion.msg)

## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs
   geometry_msgs
   actionlib_msgs
 )


 generate_dynamic_reconfigure_options(
  	cfg/radar.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES cameleon_ros_driver
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs
#  DEPENDS 
)

# include_directories(include)
include_directories(
  INCLUDE_DIRS include
  ${catkin_INCLUDE_DIRS}
)
## Declare a C++ library

## Declare a C++ executable
add_executable(radarology src/radarology.cpp)
add_executable(radar_align src/radar_align.cpp)
add_executable(ground_truth src/ground_truth.cpp)
add_executable(legFilter src/legFilter.cpp)
add_executable(radar_detector_ol src/radar_detector_ol.cpp src/svm.cpp)
add_executable(laser_labelling src/laser_labelling.cpp)
add_executable(evaluator src/evaluator.cpp)
## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(radarology ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(ground_truth ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(evaluator ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(legFilter ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(radar_detector_ol ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(laser_labelling ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})
add_dependencies(radar_align ${PROJECT_NAME}_generate_messages_cpp ${PROJECT_NAME}_gencfg ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(radarology ${catkin_LIBRARIES} )
target_link_libraries(ground_truth ${catkin_LIBRARIES} )
target_link_libraries(evaluator ${catkin_LIBRARIES} )
target_link_libraries(legFilter ${catkin_LIBRARIES} )
target_link_libraries(radar_detector_ol ${catkin_LIBRARIES} )
target_link_libraries(laser_labelling ${catkin_LIBRARIES} )
target_link_libraries(radar_align ${catkin_LIBRARIES} )
