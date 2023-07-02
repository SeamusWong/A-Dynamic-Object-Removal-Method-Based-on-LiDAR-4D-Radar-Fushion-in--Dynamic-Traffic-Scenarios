/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-08 22:41:18
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-19 23:34:03
 * @FilePath: /Dynamic_filter/src/libraries/include/lib_cluster_whx.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef LIB_CLUSTER_WHX_
#define LIB_CLUSTER_WHX_

#include "lib_cluster_lidar.h"
#include "lib_cluster_processPointClouds.h"
#include "lib_io_flow_whx.h"
#include "lib_io_pointcloud_whx.h"
#include "lib_io_boundingbox_whx.h"
#include "libs_system_all.h"

using namespace std;
using namespace LIB_IO_POINTCLOUD_WHX;
using namespace LIB_IO_BOUNDINGBOX_WHX;

namespace LIB_CLUSTER_WHX
{
    void PointCloudCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud, BoxListWHX &box_list);
    void operator<<(BoxListWHX &box_list,const vector<Box> &boxes_Box);
}

void LIB_CLUSTER_WHX::PointCloudCluster(const pcl::PointCloud<pcl::PointXYZI>::Ptr &inputCloud, BoxListWHX &box_list)
{
    vector<Box> box_Box_list;
    box_list.clear();
    ProcessPointClouds<pcl::PointXYZI> pointCloudXYZIProcessor;

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointCloudXYZIProcessor.FilterCloud(inputCloud, 0.2, Eigen::Vector4f(-80, -30, -3, 2), Eigen::Vector4f(80, 30, 1, 2)); // TODO: try 1.5, 0.3, 0.5 | -10, -5, -2, 1, 30, 8, 1, 1
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = pointCloudXYZIProcessor.SegmentPlane(filteredCloud, 200, 0.3);      // TOTRY: 迭代次数、相邻点距离阈值
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointCloudXYZIProcessor.Clustering(segmentedCloud.first, 0.5,true,30,500);//点集群容差、聚类框最小、最大点数        // TODO: add min/max num points in plausible target, eg. 100-1600
    for (pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : obstacleClusters)
    {
        Box box = pointCloudXYZIProcessor.BoundingBox(cluster);
        box_Box_list.push_back(box);
    }
    box_list << box_Box_list;
}

void LIB_CLUSTER_WHX::operator<<(BoxListWHX &box_list, const vector<Box> &boxes_Box)
{
    box_list.clear();
    for (int count = 0; count < boxes_Box.size(); count++)
    {
        BoundingBox3DWHX box_single;
        Box box_Box_single = boxes_Box.at(count);
        box_single.center_x = (box_Box_single.x_max + box_Box_single.x_min) / 2;
        box_single.center_y = (box_Box_single.y_max + box_Box_single.y_min) / 2;
        box_single.center_z = (box_Box_single.z_max + box_Box_single.z_min) / 2;
        box_single.box_length = box_Box_single.x_max - box_Box_single.x_min;
        box_single.box_width = box_Box_single.y_max - box_Box_single.y_min;
        box_single.box_height = box_Box_single.z_max - box_Box_single.z_min;
        box_single.angle_yaw = 0;

        if(box_single.box_length < 0.1 || box_single.box_width < 0.1)
        {
            continue;
        }

        if(box_single.center_x < 0)
        {
            continue;
        }

        // if (std::abs(box_single.center_y) > std::abs(box_single.center_x))
        // {
        //     continue;
        // }

        if(std::sqrt((std::pow(box_single.center_x,2) + std::pow(box_single.center_y,2))) < 3)
        {
            continue;
        }
        box_list.push_back(box_single);
    }
}

#endif