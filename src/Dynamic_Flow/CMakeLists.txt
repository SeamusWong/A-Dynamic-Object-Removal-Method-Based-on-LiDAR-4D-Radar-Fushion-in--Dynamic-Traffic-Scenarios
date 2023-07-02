cmake_minimum_required(VERSION 3.0.2)
SET(CMAKE_BUILD_TYPE "Release")
add_definitions(-std=c++14)

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS, "${CXX_FLAGS}")

project(Dynamic_Flow)

find_package(catkin REQUIRED COMPONENTS
  pcl_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

find_package(PCL 1.10 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")


catkin_package(

)

include_directories(
  include
  ../libraries/include
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}
)

# add_executable(Flow_demo src/Flow_demo.cpp)

# add_dependencies(Flow_demo
#   ${${PROJECT_NAME}_EXPORTED_TARGETS}
#   ${catkin_EXPORTED_TARGETS}
# )

# target_link_libraries(Flow_demo
#   lib_detection_whx
#   lib_io_pointcloud_whx
#   lib_io_boundingbox_whx
#   lib_io_calib_whx
#   lib_io_radar_whx
#   lib_io_label_whx
#   lib_io_flow_whx
#   ${catkin_LIBRARIES}
# )
######################################################################
add_executable(Dynamic_flow src/Dynamic_flow.cpp)

add_dependencies(Dynamic_flow
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(Dynamic_flow
  libs_system_all
  lib_io_pointcloud_whx
  lib_detection_whx
  lib_io_calib_whx
  lib_io_label_whx
  lib_io_boundingbox_whx
  lib_io_flow_whx
  lib_io_radar_whx
  lib_EKF_whx
  lib_others_whx
  lib_ground_whx
  ${catkin_LIBRARIES}
)

