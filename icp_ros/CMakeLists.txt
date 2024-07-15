cmake_minimum_required(VERSION 3.0.2)
project(icp_ros)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
  pcl_conversions
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp pcl_ros pcl_conversions
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(gicp_ros src/gicp_ros.cpp)
target_link_libraries(gicp_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES})
