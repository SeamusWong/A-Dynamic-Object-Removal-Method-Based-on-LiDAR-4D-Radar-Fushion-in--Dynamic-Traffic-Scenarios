/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-07 14:19:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-14 09:26:31
 * @FilePath: /Dynamic_filter/src/libraries/include/lib_io_pointcloud_whx.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef LIB_IO_POINTCLOUD_WHX_
#define LIB_IO_POINTCLOUD_WHX_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <thread>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ctime>
#include <sys/io.h>
#include <boost/thread/thread.hpp>

#include "lib_io_boundingbox_whx.h"

using namespace std;
using namespace LIB_IO_BOUNDINGBOX_WHX;

namespace LIB_IO_POINTCLOUD_WHX
{
    void ReadBinToPcdPtr(const std::string &in_file, const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_ptr);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReadBinToPcdPtr(const std::string &in_file);
    pcl::PointCloud<pcl::PointXYZI> ReadBinToPcd(const std::string &in_file);

    void CloudVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud, float leaf_size_square);
    pcl::PointCloud<pcl::PointXYZI>::Ptr CloudVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, float leaf_size_square);

    void PublishCloudToRviz(ros::Publisher &msg_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pub_cloud, const std::string frame_id = "odom");
    
    bool ComputePairNum(const std::string &pair1,const std::string &pair2);
    void GetFileNamesInFolder(const std::string &path, vector<std::string> &filenames);

    void RemoveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud, const vector<BoundingBox3DWHX> &boxes);
    pcl::PointCloud<pcl::PointXYZI>::Ptr RemoveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, const vector<BoundingBox3DWHX> &boxes);
    void ReserveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud, const vector<BoundingBox3DWHX> &boxes);
    pcl::PointCloud<pcl::PointXYZI>::Ptr ReserveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, const vector<BoundingBox3DWHX> &boxes);
    void RemoveRearPointFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud, float distance);
    pcl::PointCloud<pcl::PointXYZI>::Ptr RemoveRearPointFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, float distance);

    int PointInBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr &m_cloud,const BoundingBox3DWHX &m_box);
}

#endif