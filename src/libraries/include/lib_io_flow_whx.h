#ifndef LIB_IO_FLOW_WHX_
#define LIB_IO_FLOW_WHX_

#include "libs_system_all.h"
#include "lib_io_label_whx.h"
#include "lib_io_pointcloud_whx.h"
#include "lib_io_radar_whx.h"
#include "lib_ndt_whx.h"
#include "lib_io_label_whx.h"
#include "lib_io_calib_whx.h"
#include "lib_io_boundingbox_whx.h"
#include "lib_io_pose_whx.h"

using namespace std;
using namespace LIB_IO_LABEL_WHX;
using namespace LIB_IO_POINTCLOUD_WHX;
using namespace LIB_IO_RADAR_WHX;
using namespace LIB_NDT_WHX;
using namespace LIB_IO_CALIB_WHX;
using namespace LIB_IO_LABEL_WHX;
using namespace LIB_IO_BOUNDINGBOX_WHX;
using namespace LIB_IO_POSE_WHX;

namespace LIB_IO_FLOW_WHX
{
    class LabelDataFlowWHX
    {
    private:
        vector<std::string> label_data_files_all;

    public:
        LabelDataFlowWHX(/* args */);
        explicit LabelDataFlowWHX(const std::string &folder_path);
        ~LabelDataFlowWHX();
        bool IsEmpty();
        void ReadFolder(const std::string &folder_path);
        LabelDataWHX CallbackByFrame(int frame);
        int Size();
    };

    LabelDataFlowWHX::LabelDataFlowWHX(/* args */)
    {
    }

    LabelDataFlowWHX::LabelDataFlowWHX(const std::string &folder_path)
    {
        ReadFolder(folder_path);
    }

    LabelDataFlowWHX::~LabelDataFlowWHX()
    {
    }

    class RadarDataFlowWHX
    {
    private:
        vector<std::string> radar_data_files_all;

    public:
        RadarDataFlowWHX(/* args */);
        explicit RadarDataFlowWHX(const std::string &folder_path);
        ~RadarDataFlowWHX();
        bool IsEmpty();
        void ReadFolder(const std::string &folder_path);
        RadarDataWHX CallbackByFrame(int frame);
        int Size();
    };

    RadarDataFlowWHX::RadarDataFlowWHX(/* args */)
    {
    }

    RadarDataFlowWHX::RadarDataFlowWHX(const std::string &folder_path)
    {
        this->ReadFolder(folder_path);
    }

    RadarDataFlowWHX::~RadarDataFlowWHX()
    {
    }

    class PointCloudFlowWHX
    {
    private:
        vector<std::string> point_cloud_files_all;

    public:
        PointCloudFlowWHX(/* args */);
        explicit PointCloudFlowWHX(const std::string &folder_path);
        ~PointCloudFlowWHX();
        bool IsEmpty();
        void ReadFolder(const std::string &folder_path);
        pcl::PointCloud<pcl::PointXYZI> CallbackByFrame(int frame);
        pcl::PointCloud<pcl::PointXYZI>::Ptr CallbackPtrByFrame(int frame);
        int Size();
    };

    PointCloudFlowWHX::PointCloudFlowWHX(/* args */)
    {
    }

    PointCloudFlowWHX::PointCloudFlowWHX(const std::string &folder_path)
    {
        this->ReadFolder(folder_path);
    }

    PointCloudFlowWHX::~PointCloudFlowWHX()
    {
    }

    class BoundingBox3DFlowWHX
    {
    private:
        /* data */
        LabelDataFlowWHX label_data_all;
        CalibDataWHX calib_data_lidar;
        CalibDataWHX calib_data_radar;
        RadarDataFlowWHX radar_data_all;
        float dynamic_threshold = 0.0;

    public:
        BoundingBox3DFlowWHX(const LabelDataFlowWHX &label_flow);
        BoundingBox3DFlowWHX(const LabelDataFlowWHX &label_flow, const CalibDataWHX &calib_flow_lidar);
        BoundingBox3DFlowWHX(const LabelDataFlowWHX &label_flow, const CalibDataWHX &calib_flow_lidar, const CalibDataWHX &calib_flow_radar, const RadarDataFlowWHX &radar_flow);
        ~BoundingBox3DFlowWHX();
        BoundingBox3DFlowWHX &SetDynamicThreshold(float set_number);
        BoundingBox3DFlowWHX &VelocityCompensation(int frame,const Eigen::Vector4f &self_trans_vec);
        void CallbackBoxList(int frame, BoxListWHX &boxes_out,bool if_v_compensated = false);
    };

    BoundingBox3DFlowWHX::BoundingBox3DFlowWHX(const LabelDataFlowWHX &label_flow)
    {
        this->label_data_all = label_flow;
    }

    BoundingBox3DFlowWHX::BoundingBox3DFlowWHX(const LabelDataFlowWHX &label_flow, const CalibDataWHX &calib_flow_lidar)
    {
        this->label_data_all = label_flow;
        this->calib_data_lidar = calib_flow_lidar;
    }

    BoundingBox3DFlowWHX::BoundingBox3DFlowWHX(const LabelDataFlowWHX &label_flow, const CalibDataWHX &calib_flow_lidar, const CalibDataWHX &calib_flow_radar, const RadarDataFlowWHX &radar_flow)
    {
        this->label_data_all = label_flow;
        this->calib_data_lidar = calib_flow_lidar;
        this->calib_data_radar = calib_flow_radar;
        this->radar_data_all = radar_flow;
    }

    BoundingBox3DFlowWHX::~BoundingBox3DFlowWHX()
    {
    }

    class PoseFlowWHX
    {
    private:
        /* data */
        vector<std::string> pose_files_all;

    public:
        PoseFlowWHX(/* args */);
        PoseFlowWHX(const std::string &folder_path);
        ~PoseFlowWHX();
        void ReadFolder(const std::string &folder_path);
        bool IsEmpty();
        PoseDataWHX CallbackByFrame(int frame);
        int Size();
    };
    
    PoseFlowWHX::PoseFlowWHX(/* args */)
    {
    }
    
    PoseFlowWHX::PoseFlowWHX(const std::string &folder_path)
    {
        this->ReadFolder(folder_path);
    }
    
    PoseFlowWHX::~PoseFlowWHX()
    {
    }
    
}

#endif