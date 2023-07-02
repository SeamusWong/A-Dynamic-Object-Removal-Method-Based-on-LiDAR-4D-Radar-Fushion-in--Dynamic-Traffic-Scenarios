/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-07 14:19:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-04-20 14:37:26
 * @FilePath: /Dynamic_filter/src/libraries/include/libs_system_all.h
 * 
 * @Description: 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <thread>
/***********************/
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/ndt.h>               //NDT配准类对应头文件
#include <pcl/filters/approximate_voxel_grid.h> //滤波类对应头文件
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/extract_indices.h>
/*************/
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ctime>
#include <cmath>
#include <sys/io.h>
#include <boost/thread/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <json/json.h>
#include <ros/console.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
