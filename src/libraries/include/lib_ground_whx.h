/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-04-20 14:35:30
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-15 22:03:45
 * @FilePath: /Dynamic_filter/src/libraries/include/lib_ground_whx.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef LIB_GROUND_WHX_
#define LIB_GROUNG_WHX_

#include "libs_system_all.h"

#define CLIP_HEIGHT 3 // 截取掉高于雷达自身x米的点
#define MIN_DISTANCE 3
#define RADIAL_DIVIDER_ANGLE 0.18
#define SENSOR_HEIGHT 1.5

#define concentric_divider_distance_ 0.05 // 0.1 meters default
#define min_height_threshold_ 0.05
#define local_max_slope_ 3   // max slope of the ground between points, degree
#define general_max_slope_ 3 // max slope of the ground in entire point cloud, degree
#define reclass_distance_threshold_ 0.2

namespace LIB_GROUND_WHX
{
    class GroundWHX
    {
    private:

        pcl::PointCloud<pcl::PointXYZI>::Ptr ground_cloud_ptr;
        pcl::PointCloud<pcl::PointXYZI>::Ptr no_ground_cloud_ptr;

        struct PointXYZIRTColor
        {
            pcl::PointXYZI point;

            float radius; // cylindric coords on XY Plane
            float theta;  // angle deg on XY plane

            size_t radial_div;     // index of the radial divsion to which this point belongs to
            size_t concentric_div; // index of the concentric division to which this points belongs to

            size_t original_index; // index of this point in the source pointcloud
        };
        typedef std::vector<PointXYZIRTColor> PointCloudXYZIRTColor;

        size_t radial_dividers_num_;
        size_t concentric_dividers_num_;


        void ClipAbove(double clip_height, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

        void RemoveClosePt(double min_distance, const pcl::PointCloud<pcl::PointXYZI>::Ptr in, const pcl::PointCloud<pcl::PointXYZI>::Ptr out);

        void FromXYZIToRTZColor(const pcl::PointCloud<pcl::PointXYZI>::Ptr in_cloud,
                            PointCloudXYZIRTColor &out_organized_points,
                            std::vector<pcl::PointIndices> &out_radial_divided_indices,
                            std::vector<PointCloudXYZIRTColor> &out_radial_ordered_clouds);

        void ClassfyPc(std::vector<PointCloudXYZIRTColor> &in_radial_ordered_clouds,
                        pcl::PointIndices &out_ground_indices,
                        pcl::PointIndices &out_no_ground_indices);


    public:
        GroundWHX &PointCb(const pcl::PointCloud<pcl::PointXYZI>::Ptr &current_pc_ptr);
        pcl::PointCloud<pcl::PointXYZI>::Ptr CallbackGround();
        pcl::PointCloud<pcl::PointXYZI>::Ptr CallbackNoGround();
        GroundWHX();
        ~GroundWHX();
    };

    GroundWHX::GroundWHX()
    {
        this->ground_cloud_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
        this->no_ground_cloud_ptr = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>);
    }

    GroundWHX::~GroundWHX()
    {
    }
}
#endif