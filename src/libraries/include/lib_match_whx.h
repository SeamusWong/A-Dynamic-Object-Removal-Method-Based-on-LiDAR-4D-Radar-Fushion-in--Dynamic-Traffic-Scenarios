#ifndef LIB_MATCH_WHX_
#define LIB_MATCH_WHX_

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

#include "lib_io_pointcloud_whx.h"
#include "lib_io_boundingbox_whx.h"

using namespace std;
using namespace LIB_IO_POINTCLOUD_WHX;
using namespace LIB_IO_BOUNDINGBOX_WHX;

namespace LIB_MATCH_WHX
{
    struct TargetSingleWHX
    {
        vector<int> frame_list;
        vector<BoundingBox3DWHX> box_list;
    };
    class TargetTrackingWHX
    {
    private:
        vector<TargetSingleWHX> targets_all;
    public:
        TargetTrackingWHX(/* args */);
        ~TargetTrackingWHX();
        void MatchTarget(int frame_number,const vector<BoundingBox3DWHX> &boxes_single_frame);
    };
    
    TargetTrackingWHX::TargetTrackingWHX(/* args */)
    {
    }
    
    TargetTrackingWHX::~TargetTrackingWHX()
    {
    }
    
}

#endif