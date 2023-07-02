/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-07 14:19:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-05-11 21:04:25
 * @FilePath: /Dynamic_filter/src/libraries/include/lib_io_radar_whx.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef LIB_IO_RADAR_WHX_
#define LIB_IO_RADAR_WHX_

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
#include <pcl/common/transforms.h>
#include <ctime>
#include <sys/io.h>
#include <boost/thread/thread.hpp>

#include "lib_io_boundingbox_whx.h"
#include "lib_io_calib_whx.h"
#include "lib_io_pointcloud_whx.h"

using namespace std;
using namespace LIB_IO_BOUNDINGBOX_WHX;
using namespace LIB_IO_CALIB_WHX;
using namespace LIB_IO_POINTCLOUD_WHX;

namespace LIB_IO_RADAR_WHX
{
    struct RadarPointWHX
    {
        float point_x, point_y, point_z;
        float point_rcs;
        float v_relative, v_compensated;
        float id_time;
    };
    class RadarDataWHX
    {
    private:
        /* data */
        vector<RadarPointWHX> radar_cloud;
        pcl::PointXYZI ToPcl(int number);

    public:
        RadarDataWHX(/* args */);
        explicit RadarDataWHX(const std::string &file_path);
        ~RadarDataWHX();
        int Size();
        bool IsEmpty();
        void ReadFile(const std::string &file_path);

        virtual void Print();
        virtual void Print(int number);
        virtual void Print(const BoundingBox3DWHX &box_single);
        virtual void Print(const BoxListWHX &boxes);

        RadarDataWHX &Pushback(const RadarPointWHX &point);
        void PublishToRviz(ros::Publisher &msg_publisher, const std::string frame_id = "odom");
        pcl::PointCloud<pcl::PointXYZI>::Ptr ToPcl();
        float CallbackVelocity(BoundingBox3DWHX &box_single, bool if_v_compensated = false);
        RadarPointWHX CallbackPoint(int m_number_point);
        Eigen::Vector2f CallbackGroundVelo(const BoxListWHX &boxes, bool if_v_compensated = false);
        RadarDataWHX &VelocityCompensation(const Eigen::Vector4f &self_trans_vec);
        void AddBoxListVelo(BoxListWHX &boxes, bool if_v_compensated = false);
        void AddBoxListVelo(BoxListWHX &boxes, int box_number,bool if_v_compensated = false);
        RadarDataWHX &Transform(const Eigen::Matrix4f &mat);
        RadarDataWHX &TransToLidar(const CalibDataWHX &lidar_calib, const CalibDataWHX &radar_calib);
        RadarDataWHX &RemovePointInBoxlist(const BoxListWHX &boxlist);
        RadarDataWHX &RemovePointInBoxlist(const BoxListWHX &boxlist, RadarDataWHX &out);
        RadarDataWHX &RemovePointInGround(void);

        operator pcl::PointCloud<pcl::PointXYZI>::Ptr();
        friend RadarDataWHX &operator<<(RadarDataWHX &radar_cloud_1 ,const RadarDataWHX &radar_cloud_2);
    };

    RadarDataWHX::RadarDataWHX(/* args */)
    {
    }

    RadarDataWHX::RadarDataWHX(const std::string &file_path)
    {
        this->ReadFile(file_path);
    }

    RadarDataWHX::~RadarDataWHX()
    {
    }

    RadarDataWHX &operator<<(RadarDataWHX &radar_cloud_1, const RadarDataWHX &radar_cloud_2)
    {
        radar_cloud_1.radar_cloud.clear();
        radar_cloud_1.radar_cloud.assign(radar_cloud_2.radar_cloud.begin(), radar_cloud_2.radar_cloud.end());
        return radar_cloud_1;
    }
}

#endif