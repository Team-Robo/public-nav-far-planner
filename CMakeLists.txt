cmake_minimum_required(VERSION 3.0.2)
project(far_planner)

set(CMAKE_CXX_STANDARD 14)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

## FAR planning modules + move_base global planner plugin (nav_core::BaseGlobalPlanner)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  nav_core
  costmap_2d
  pluginlib
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES far_planner_ros
  CATKIN_DEPENDS
    roscpp std_msgs sensor_msgs geometry_msgs nav_msgs visualization_msgs
    tf tf2 tf2_ros tf2_geometry_msgs pcl_ros pcl_conversions
    nav_core costmap_2d pluginlib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${OpenCV_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

## FAR planning modules (unchanged) + static definitions + plugin glue, one shared library for pluginlib
add_library(far_planner_ros SHARED
  src/utility.cpp
  src/contour_detector.cpp
  src/contour_graph.cpp
  src/dynamic_graph.cpp
  src/graph_planner.cpp
  src/map_handler.cpp
  src/scan_handler.cpp
  src/terrain_planner.cpp
  src/planner_visualizer.cpp
  src/far_globals.cpp
  src/far_planner_ros.cpp
)
add_dependencies(far_planner_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(far_planner_ros ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBS})

install(TARGETS far_planner_ros
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(FILES far_planner_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
