cmake_minimum_required(VERSION 2.8.3)
project(map_localization)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  novatel_oem7_msgs
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs nav_msgs geometry_msgs
                 tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions novatel_oem7_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(mapLocalization src/map_localization_node.cpp)
add_dependencies(mapLocalization ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapLocalization ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(waypointTransform src/waypoint_transform_node.cpp)
add_dependencies(waypointTransform ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypointTransform ${catkin_LIBRARIES})
