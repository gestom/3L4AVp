cmake_minimum_required(VERSION 2.8.3)
project(person_splitter)

add_compile_options(-std=c++11 -Wall )

find_package(catkin REQUIRED COMPONENTS
    roscpp
    sensor_msgs
    std_msgs
    message_generation
    pcl_ros
    pcl_conversions
    geometry_msgs
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
)

###########
## Build ##
###########

include_directories(${catkin_INCLUDE_DIRS})
add_executable(ppl_spliter src/ppl_spliter.cpp ${SRCS})
target_link_libraries(ppl_spliter ${catkin_LIBRARIES} )
