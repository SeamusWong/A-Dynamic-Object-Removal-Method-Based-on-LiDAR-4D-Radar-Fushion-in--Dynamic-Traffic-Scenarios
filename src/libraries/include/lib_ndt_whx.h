/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-07 14:19:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-05-11 19:05:33
 * @FilePath: /Dynamic_filter/src/libraries/include/lib_ndt_whx.h
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#ifndef LIB_NDT_WHX_
#define LIB_NDT_WHX_

#include "libs_system_all.h"
#include "lib_io_pointcloud_whx.h"
#include "lib_io_boundingbox_whx.h"
#include "lib_io_pose_whx.h"

using namespace std;
using namespace LIB_IO_POINTCLOUD_WHX;
using namespace LIB_IO_BOUNDINGBOX_WHX; 
using namespace LIB_IO_POSE_WHX;

namespace LIB_NDT_WHX
{

    class NDTCloudWHX
    {
    private:
        /* data */
        bool ComputePairNum(std::string pair1, std::string pair2);
        const Eigen::Matrix4f ZeroNDTMat(); 

    public:
        NDTCloudWHX(/* args */);
        ~NDTCloudWHX();
        Eigen::Matrix4f CallbackMatrix(const std::string &target_cloud_file, const std::string &source_cloud_file,Eigen::Matrix4f init_matrix = Eigen::Matrix4f::Zero());
        Eigen::Matrix4f CallbackMatrix(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, Eigen::Matrix4f init_matrix = Eigen::Matrix4f::Zero());
        Eigen::Matrix4f CallbackPoseMatrix(const PoseDataWHX &pose_otl_1,const PoseDataWHX &pose_otl_2);
        Eigen::Vector4f MatToEKFVector(const Eigen::Matrix4f &mat, float time);
        // Eigen::Matrix4f CallbackMatrix(const Eigen::Matrix4f &m_mat_last, const Eigen::Matrix4f &m_mat_this, const Eigen::Matrix4f &m_mat_source);
        
    };

    NDTCloudWHX::NDTCloudWHX(/* args */)
    {
    }
    
    NDTCloudWHX::~NDTCloudWHX()
    {
    }
    
}

#endif