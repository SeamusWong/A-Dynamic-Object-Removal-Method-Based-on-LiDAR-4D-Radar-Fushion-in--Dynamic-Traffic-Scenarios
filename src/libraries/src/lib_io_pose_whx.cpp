/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-24 23:00:07
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-04-07 10:46:29
 * @FilePath: /Dynamic_filter/src/libraries/src/lib_io_pose_whx.cpp
 * @Description: 
 * 
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved. 
 */
#include "lib_io_pose_whx.h"

using namespace LIB_IO_POSE_WHX;

void PoseDataWHX::ReadFile(const std::string &file_path)
{
    ifstream stream_pose;
    stream_pose.open(file_path);
    if (stream_pose.is_open() == false)
    {
        std::cerr << "打开Pose文件错误!" << endl;
        exit(EXIT_FAILURE);
    }

    Json::Reader reader_pose;
    Json::Value root;

    while(stream_pose.eof() == false)
    {
        std::string line_pose;
        getline(stream_pose, line_pose);

        if (line_pose.find("odomToCamera") != std::string::npos)
        {
            reader_pose.parse(line_pose,root);
            const auto& line_odom_to_camera = root["odomToCamera"];
            if (line_odom_to_camera.isArray() && line_odom_to_camera.size() > 0)
            {
                for (int count_row = 0, count_num = 0; count_row < matrix_odom_to_camera.rows(); count_row++)
                {
                    for (int count_col = 0; count_col < matrix_odom_to_camera.cols(); count_col++)
                    {
                        matrix_odom_to_camera(count_row, count_col) = line_odom_to_camera[count_num++].asFloat();
                    }
                }
            }
            
        }

        if (line_pose.find("mapToCamera") != std::string::npos)
        {
            reader_pose.parse(line_pose, root);
            const auto &line_map_to_camera = root["mapToCamera"];
            if (line_map_to_camera.isArray() && line_map_to_camera.size() > 0)
            {
                for (int count_row = 0, count_num = 0; count_row < matrix_map_to_camera.rows(); count_row++)
                {
                    for (int count_col = 0; count_col < matrix_map_to_camera.cols(); count_col++)
                    {
                        matrix_map_to_camera(count_row, count_col) = line_map_to_camera[count_num++].asFloat();
                    }
                }
            }
        }

        if (line_pose.find("UTMToCamera") != std::string::npos)
        {
            reader_pose.parse(line_pose, root);
            const auto &line_utm_to_camera = root["UTMToCamera"];
            if (line_utm_to_camera.isArray() && line_utm_to_camera.size() > 0)
            {
                for (int count_row = 0, count_num = 0; count_row < matrix_utm_to_camera.rows(); count_row++)
                {
                    for (int count_col = 0; count_col < matrix_utm_to_camera.cols(); count_col++)
                    {
                        matrix_utm_to_camera(count_row, count_col) = line_utm_to_camera[count_num++].asFloat();
                    }
                }
            }
        }
    }
}

Eigen::Matrix4f PoseDataWHX::CallbackMatrix4f(int mat_code) const
{
    Eigen::Matrix4f result;
    switch (mat_code)
    {
    case this->code_odom_camera:
        result = this->matrix_odom_to_camera;
        break;

    case this->code_map_camera:
        result = this->matrix_map_to_camera;
        break;

    case this->code_utm_camera:
        result = this->matrix_utm_to_camera;
        break;

    default:
        break;
    }
    return result;
}
