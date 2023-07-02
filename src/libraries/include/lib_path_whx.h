/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-30 19:11:43
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-05-12 15:59:27
 * @FilePath: /Dynamic_filter/src/libraries/include/lib_path_whx.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef LIB_PATH_WHX_
#define LIB_PATH_WHX_

#include "libs_system_all.h"
#include "lib_io_calib_whx.h"

using namespace std;
using namespace LIB_IO_CALIB_WHX;

namespace LIB_PATH_WHX
{
    class PathDataWHX
    {
    private:
        /* data */
        nav_msgs::Path m_path;
    public:
        PathDataWHX &Init(void);
        PathDataWHX &AddPointFromMatrix4f(Eigen::Matrix4f m_mat,ros::Time m_time = ros::Time::now());
        PathDataWHX &AddPointFromMatrix4fForPose(Eigen::Matrix4f m_mat_last, Eigen::Matrix4f m_mat_this, CalibDataWHX m_calib, ros::Time m_time = ros::Time::now());
        PathDataWHX &FAPP(Eigen::Matrix4f m_mat,ros::Time m_time = ros::Time::now());
        void PublishToRviz(ros::Publisher &m_publisher, const std::string m_frame_id = "odom");
        void PublishAllPoseStamped(ros::Publisher &m_publisher, const std::string m_frame_id = "odom");
        nav_msgs::PathConstPtr CallbackPath();
        PathDataWHX(/* args */);
        ~PathDataWHX();
    };
    
    PathDataWHX::PathDataWHX(/* args */)
    {
        this->Init();
    }
    
    PathDataWHX::~PathDataWHX()
    {
    }
    
}



#endif
