#ifndef LIB_IO_POSE_WHX_
#define LIB_IO_POSE_WHX_

#include "libs_system_all.h"
#include "lib_io_calib_whx.h"

using namespace std;
using namespace LIB_IO_CALIB_WHX;

namespace LIB_IO_POSE_WHX
{
    class PoseDataWHX
    {
    private:
        Eigen::Matrix4f matrix_odom_to_camera;
        Eigen::Matrix4f matrix_map_to_camera;
        Eigen::Matrix4f matrix_utm_to_camera;

    public:
        static const int code_odom_camera = 0,code_map_camera = 1,code_utm_camera = 2;
        PoseDataWHX(/* args */);
        PoseDataWHX(const std::string &file_path);
        ~PoseDataWHX();
        void ReadFile(const std::string &file_path);
        Eigen::Matrix4f CallbackMatrix4f(int mat_code) const;
    };
    
    PoseDataWHX::PoseDataWHX(/* args */)
    {
    }

    PoseDataWHX::PoseDataWHX(const std::string &file_path)
    {
        this->ReadFile(file_path);
    }
    
    PoseDataWHX::~PoseDataWHX()
    {
    }
    
}

#endif