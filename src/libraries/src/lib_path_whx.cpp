/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-30 19:13:36
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-05-12 18:43:14
 * @FilePath: /Dynamic_filter/src/libraries/src/lib_path_whx.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "lib_path_whx.h"

using namespace LIB_PATH_WHX;

PathDataWHX &PathDataWHX::Init(void)
{
    this->m_path.poses.clear();
    geometry_msgs::PoseStamped point_init;
    point_init.pose.position.x = 0.0;
    point_init.pose.position.y = 0.0;
    point_init.pose.position.z = 0.0;
    this->m_path.poses.push_back(point_init);
    return *this;
}

PathDataWHX &PathDataWHX::AddPointFromMatrix4f(Eigen::Matrix4f m_mat, ros::Time m_time)
{
    Eigen::Vector3f translation_vec = m_mat.block<3,1>(0,3);
    for(int count = 0;count < this->m_path.poses.size();count ++)
    {
        this->m_path.poses.at(count).pose.position.x += translation_vec(0);
        this->m_path.poses.at(count).pose.position.y += translation_vec(1);
        // this->m_path.poses.at(count).pose.position.z += translation_vec(2);
    }

    Eigen::Matrix3f rotation_mat = m_mat.block<3,3>(0,0);
    float yaw = std::atan2(rotation_mat(1, 0), rotation_mat(0, 0));
    for (int count = 0; count < this->m_path.poses.size(); count++)
    {
        this->m_path.poses.at(count).pose.position.x = this->m_path.poses.at(count).pose.position.x * std::cos(yaw) - this->m_path.poses.at(count).pose.position.y * std::sin(yaw);
        this->m_path.poses.at(count).pose.position.y = this->m_path.poses.at(count).pose.position.x * std::sin(yaw) + this->m_path.poses.at(count).pose.position.y * std::cos(yaw);
    }

    geometry_msgs::PoseStamped point_next;
    point_next.header.stamp = m_time;
    point_next.pose.position.x = 0;
    point_next.pose.position.y = 0;
    point_next.pose.position.z = 0;

    this->m_path.poses.push_back(point_next);
    return *this;
}

// PathDataWHX &PathDataWHX::AddPointFromMatrix4fForPose(Eigen::Matrix4f m_mat_last, Eigen::Matrix4f m_mat_this)
// {
//     Eigen::Vector3f translation_vec_last = m_mat_last.block<3, 1>(0, 3) / 1000;
//     Eigen::Vector3f translation_vec_this = m_mat_this.block<3, 1>(0, 3) / 1000;
//     Eigen::Matrix3f rotation_mat_last = m_mat_last.block<3, 3>(0, 0);
//     Eigen::Matrix3f rotation_mat_this = m_mat_this.block<3, 3>(0, 0);

//     float yaw_last = std::atan2(rotation_mat_last(1, 0), rotation_mat_last(0, 0));
//     float yaw_this = std::atan2(rotation_mat_this(1, 0), rotation_mat_this(0, 0));

//     // float yaw_last = std::atan2(rotation_mat_last(0, 2), sqrt(1 - rotation_mat_last(0, 2) * rotation_mat_last(0, 2) - rotation_mat_last(1, 2) * rotation_mat_last(1, 2)));
//     // float yaw_this = std::atan2(rotation_mat_this(0, 2), sqrt(1 - rotation_mat_this(0, 2) * rotation_mat_this(0, 2) - rotation_mat_this(1, 2) * rotation_mat_this(1, 2)));

//     Eigen::Vector3f translation_vec_between = translation_vec_this - translation_vec_last;
//     Eigen::Matrix3f rotatio_mat_between = rotation_mat_last.inverse() * rotation_mat_this;
//     float yaw = yaw_this - yaw_last;

//     for (int count = 0; count < this->m_path.poses.size(); count++)
//     {
//         this->m_path.poses.at(count).pose.position.x += translation_vec_between(1);
//         this->m_path.poses.at(count).pose.position.y += translation_vec_between(0);
//         this->m_path.poses.at(count).pose.position.z += translation_vec_between(2);
//     }

//     for (int count = 0; count < this->m_path.poses.size(); count++)
//     {
//         this->m_path.poses.at(count).pose.position.x = this->m_path.poses.at(count).pose.position.x * std::cos(yaw) - this->m_path.poses.at(count).pose.position.y * std::sin(yaw);
//         this->m_path.poses.at(count).pose.position.y = this->m_path.poses.at(count).pose.position.x * std::sin(yaw) + this->m_path.poses.at(count).pose.position.y * std::cos(yaw);
//     }

//     geometry_msgs::PoseStamped point_next;
//     point_next.pose.position.x = 0;
//     point_next.pose.position.y = 0;
//     point_next.pose.position.z = 0;

//     this->m_path.poses.push_back(point_next);
//     return *this;
// }

PathDataWHX &PathDataWHX::AddPointFromMatrix4fForPose(Eigen::Matrix4f m_mat_last, Eigen::Matrix4f m_mat_this, CalibDataWHX m_calib, ros::Time m_time)
{
    Eigen::Matrix4f mat_last_this = m_mat_this.inverse() * m_mat_last;
    // mat_last_this = m_calib.CallbackMatrix4f(CalibDataWHX::vtc_code).inverse() * mat_last_this;
    
    Eigen::Vector3f translation_vec = mat_last_this.block<3, 1>(0, 3);
    for (int count = 0; count < this->m_path.poses.size(); count++)
    {
        this->m_path.poses.at(count).pose.position.x += translation_vec(2);
        this->m_path.poses.at(count).pose.position.y += translation_vec(0);
        // this->m_path.poses.at(count).pose.position.z += translation_vec(1);
    }

    Eigen::Matrix3f rotation_mat = mat_last_this.block<3, 3>(0, 0);
    // float yaw = std::atan2(rotation_mat(1, 0), rotation_mat(0, 0));
    // float yaw = std::atan2(-rotation_mat(1, 0), sqrt(rotation_mat(0, 0) * rotation_mat(0, 0) + rotation_mat(2, 0) * rotation_mat(2, 0)));
    float yaw = std::atan2(rotation_mat(2, 0), rotation_mat(2, 2));
    for (int count = 0; count < this->m_path.poses.size(); count++)
    {
        this->m_path.poses.at(count).pose.position.x = this->m_path.poses.at(count).pose.position.x * std::cos(yaw) - this->m_path.poses.at(count).pose.position.y * std::sin(yaw);
        this->m_path.poses.at(count).pose.position.y = this->m_path.poses.at(count).pose.position.x * std::sin(yaw) + this->m_path.poses.at(count).pose.position.y * std::cos(yaw);
    }

    geometry_msgs::PoseStamped point_next;
    point_next.header.stamp = m_time;
    point_next.pose.position.x = 0;
    point_next.pose.position.y = 0;
    point_next.pose.position.z = 0;

    this->m_path.poses.push_back(point_next);
    return *this;
}

void PathDataWHX::PublishToRviz(ros::Publisher &m_publisher, const std::string m_frame_id)
{
    for (int count = 0; count < this->m_path.poses.size(); count++)
    {
        this->m_path.poses.at(count).header.frame_id = m_frame_id;
    }
    this->m_path.header.stamp =ros::Time::now();
    this->m_path.header.frame_id = m_frame_id;
    m_publisher.publish(this->m_path);
}

void PathDataWHX::PublishAllPoseStamped(ros::Publisher &m_publisher, const std::string m_frame_id)
{
    for (int count = 0; count < this->m_path.poses.size(); count++)
    {
        this->m_path.poses.at(count).header.frame_id = m_frame_id;
        m_publisher.publish(this->m_path.poses.at(count));
        ros::Duration sleep(0.01);
        sleep.sleep();
    }
}

nav_msgs::PathConstPtr PathDataWHX::CallbackPath()
{
    boost::shared_ptr<::nav_msgs::Path const> result(new nav_msgs::Path(this->m_path));
    return result;
}
