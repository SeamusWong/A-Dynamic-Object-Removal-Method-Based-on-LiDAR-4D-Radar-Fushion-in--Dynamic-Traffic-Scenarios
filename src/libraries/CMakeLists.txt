cmake_minimum_required(VERSION 3.0.2)
SET(CMAKE_BUILD_TYPE "Release")
project(libraries)

add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(${INC_DIR1}) # 指定头文件的搜索路径，相当于指定 gcc 的 - I 参数
link_directories(INC_DIR1)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package(

)

include_directories(
  usr/include
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(libs_system_all
  include/libs_system_all.h
  src/libs_system_all.cpp
)

add_library(lib_detection_whx
  include/lib_detection_whx.h
  src/lib_detection_whx.cpp
)

add_library(lib_io_pointcloud_whx
  include/lib_io_pointcloud_whx.h
  src/lib_io_pointcloud_whx.cpp
)

add_library(lib_io_boundingbox_whx
  include/lib_io_boundingbox_whx.h
  src/lib_io_boundingbox_whx.cpp
)

add_library(lib_io_calib_whx
  include/lib_io_calib_whx.h
  src/lib_io_calib_whx.cpp
)

add_library(lib_io_radar_whx
  include/lib_io_radar_whx.h
  src/lib_io_radar_whx.cpp
)

add_library(lib_io_label_whx
  include/lib_io_label_whx.h
  src/lib_io_label_whx.cpp
)

add_library(lib_ndt_whx
  include/lib_ndt_whx.h
  src/lib_ndt_whx.cpp
)

add_library(lib_io_flow_whx
  include/lib_io_flow_whx.h
  src/lib_io_flow_whx.cpp
)

add_library(lib_EKF_whx
  include/lib_EKF_whx.h
  src/lib_EKF_whx.cpp
)

add_library(lib_hungarian_whx
  include/lib_hungarian_whx.h
  src/lib_hungarian_whx.cpp
)

add_library(lib_others_whx
  include/lib_others_whx.h
  src/lib_others_whx.cpp
)

add_library(lib_tracking_whx
  include/lib_tracking_whx.h
  src/lib_tracking_whx.cpp
)

add_library(lib_cluster_box
  include/lib_cluster_box.h
  src/lib_cluster_box.cpp
)

add_library(lib_cluster_kdtree
  include/lib_cluster_kdtree.h
  src/lib_cluster_kdtree.cpp
)

add_library(lib_cluster_lidar
  include/lib_cluster_lidar.h
  src/lib_cluster_lidar.cpp
)

add_library(lib_cluster_processPointClouds
  include/lib_cluster_processPointClouds.h
  src/lib_cluster_processPointClouds.cpp
)

add_library(lib_cluster_render
  include/lib_cluster_render.h
  src/lib_cluster_render.cpp
)

add_library(lib_cluster_whx
  include/lib_cluster_whx.h
  src/lib_cluster_whx.cpp
)

add_library(lib_io_pose_whx
  include/lib_io_pose_whx.h
  src/lib_io_pose_whx.cpp
)

add_library(lib_path_whx
  include/lib_path_whx.h
  src/lib_path_whx.cpp
)

add_library(lib_ground_whx
  include/lib_ground_whx.h
  src/lib_ground_whx.cpp
)
#################################################################
add_dependencies(lib_io_boundingbox_whx
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_boundingbox_whx
  libs_system_all 
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_detection_whx
  lib_io_boundingbox_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_detection_whx
  lib_io_boundingbox_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_io_pointcloud_whx
  lib_io_boundingbox_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_pointcloud_whx
  lib_io_boundingbox_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_io_radar_whx
  lib_io_boundingbox_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_radar_whx
  lib_io_boundingbox_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_io_label_whx
  lib_io_boundingbox_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_label_whx
  lib_io_boundingbox_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_io_calib_whx
  lib_io_boundingbox_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_calib_whx
  lib_io_boundingbox_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_ndt_whx
  lib_io_pointcloud_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_ndt_whx
  lib_io_pointcloud_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_io_flow_whx
  lib_io_pointcloud_whx
  lib_io_calib_whx
  lib_io_label_whx
  lib_ndt_whx
  lib_io_boundingbox_whx
  lib_io_radar_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_flow_whx
  lib_io_pointcloud_whx
  lib_io_calib_whx
  lib_io_label_whx
  lib_ndt_whx
  lib_io_boundingbox_whx
  lib_io_radar_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_hungarian_whx
  lib_io_boundingbox_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_hungarian_whx
  lib_io_boundingbox_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_others_whx
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_others_whx
  libs_system_all
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_tracking_whx
  libs_system_all
  lib_io_boundingbox_whx
  lib_hungarian_whx
  lib_others_whx
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_tracking_whx
  libs_system_all
  lib_io_boundingbox_whx
  lib_hungarian_whx
  lib_others_whx
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_cluster_box
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_cluster_box
  libs_system_all
  /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_cluster_kdtree
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_cluster_kdtree
  libs_system_all
  /usr/lib/x86_64-linux-gnu/libpcl_visualization.so
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_cluster_lidar
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_cluster_lidar
  libs_system_all
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_cluster_processPointClouds
  libs_system_all
  lib_cluster_box
  lib_cluster_kdtree
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_cluster_processPointClouds
  libs_system_all
  lib_cluster_box
  lib_cluster_kdtree
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_cluster_render
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_cluster_render
  libs_system_all
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_cluster_whx
  libs_system_all
  lib_cluster_lidar
  lib_cluster_processPointClouds
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_cluster_whx
  libs_system_all
  lib_cluster_lidar
  lib_cluster_processPointClouds
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_io_pose_whx
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_io_pose_whx
  libs_system_all
  jsoncpp
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_path_whx
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_path_whx
  libs_system_all
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
#################################################################
add_dependencies(lib_ground_whx
  libs_system_all
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(lib_ground_whx
  libs_system_all
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
)
#################################################################
