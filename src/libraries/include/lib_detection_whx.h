#ifndef LIB_DETECTION_WHX_
#define LIB_DETECTION_WHX_

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
/***********************/
#include <thread>
#include <pcl/registration/ndt.h>               //NDT配准类对应头文件
#include <pcl/filters/approximate_voxel_grid.h> //滤波类对应头文件
#include <pcl/visualization/pcl_visualizer.h>
/*************/
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <ctime>
#include <sys/io.h>
#include <boost/thread/thread.hpp>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include "lib_io_boundingbox_whx.h"

using namespace std;
using namespace LIB_IO_BOUNDINGBOX_WHX;

namespace LIB_DETECTION_WHX
{
    class DetectionData3DWHX
    {
    private:
        /* data */
        vector<int> frame_number, target_type;
        vector<std::string> target_type_str;
        vector<float> useless_3, useless_4, useless_5, useless_6, useless_7;
        vector<float> center_x, center_y, center_z;
        vector<float> box_height, box_width, box_length;
        vector<float> angle_yaw;
        vector<float> confidence;
        BoundingBox3DWHX CallbackBoundingBox(int number);
    public:
        DetectionData3DWHX(/* args */);
        DetectionData3DWHX(const std::string &file_path);
        ~DetectionData3DWHX();
        bool IsEmpty();
        void ReadFile(const std::string &file_path);
        void CallbackBoxList(int frame, BoxListWHX &boxes);
    };

    DetectionData3DWHX::DetectionData3DWHX(/* args */)
    {
    }

    DetectionData3DWHX::~DetectionData3DWHX()
    {
    }

    DetectionData3DWHX::DetectionData3DWHX(const std::string &file_path)
    {
        this->ReadFile(file_path);
    }
}

#endif