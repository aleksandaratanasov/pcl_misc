cmake_minimum_required(VERSION 2.8.3)
project(pcl_misc)


find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES pcl_misc
 CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
 DEPENDS system_lib
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(cloud_publisher src/cloud_publisher.cpp)
target_link_libraries(cloud_publisher ${catkin_LIBRARIES})
target_link_libraries(cloud_publisher pcl_common pcl_io) #${PCL_LIBRARIES})

add_executable(cloud_subscriber src/cloud_subscriber.cpp)
target_link_libraries(cloud_subscriber ${catkin_LIBRARIES})
target_link_libraries(cloud_subscriber pcl_common pcl_io pcl_filters) #${PCL_LIBRARIES})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pcl_misc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
