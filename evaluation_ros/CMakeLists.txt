cmake_minimum_required(VERSION 2.8.3)
project(evaluation_ros)

add_compile_options(-std=c++11 -Wall )
set ( CMAKE_CXX_FLAGS "-ggdb -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    pcl_ros
    pcl_conversions
    geometry_msgs
    radar
)
catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
      roscpp
      sensor_msgs
      std_msgs
      message_generation
      pcl_ros
      pcl_conversions
      geometry_msgs
      radar
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
add_executable(sensor_fusion src/sensor_fusion.cpp ${SRCS})
target_link_libraries(sensor_fusion ${catkin_LIBRARIES} )
