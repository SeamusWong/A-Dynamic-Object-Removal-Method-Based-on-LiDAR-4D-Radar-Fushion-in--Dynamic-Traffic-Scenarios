#include "lib_io_flow_whx.h"

using namespace LIB_IO_FLOW_WHX;

void LabelDataFlowWHX::ReadFolder(const std::string &folder_path)
{
    GetFileNamesInFolder(folder_path, this->label_data_files_all);
}

LabelDataWHX LabelDataFlowWHX::CallbackByFrame(int frame)
{
    LabelDataWHX result(this->label_data_files_all.at(frame));
    return result;
}

int LabelDataFlowWHX::Size()
{
    return this->label_data_files_all.size();
}

bool LabelDataFlowWHX::IsEmpty()
{
    return this->label_data_files_all.empty();
}

void RadarDataFlowWHX::ReadFolder(const std::string &folder_path)
{
    GetFileNamesInFolder(folder_path, this->radar_data_files_all);
}

RadarDataWHX RadarDataFlowWHX::CallbackByFrame(int frame)
{
    RadarDataWHX result(this->radar_data_files_all.at(frame));
    return result;
}

int RadarDataFlowWHX::Size()
{
    return this->radar_data_files_all.size();
}

bool RadarDataFlowWHX::IsEmpty()
{
    return this->radar_data_files_all.empty();
}

void PointCloudFlowWHX::ReadFolder(const std::string &folder_path)
{
    GetFileNamesInFolder(folder_path, this->point_cloud_files_all);
}

pcl::PointCloud<pcl::PointXYZI> PointCloudFlowWHX::CallbackByFrame(int frame)
{
    pcl::PointCloud<pcl::PointXYZI> result = ReadBinToPcd(this->point_cloud_files_all.at(frame));
    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr PointCloudFlowWHX::CallbackPtrByFrame(int frame)
{
    return ReadBinToPcdPtr(this->point_cloud_files_all.at(frame));
}

int PointCloudFlowWHX::Size()
{
    return this->point_cloud_files_all.size();
}

bool PointCloudFlowWHX::IsEmpty()
{
    return this->point_cloud_files_all.empty();
}

/* 
Eigen::Matrix4f NDTDataFlowWHX::CallbackMatrixSequence(int frame)
{
    return CallbackMatrixSequence(frame, frame + 1);
}

Eigen::Matrix4f NDTDataFlowWHX::CallbackMatrixSequence(int frame_1, int frame_2)
{
    if (max(frame_1, frame_2) > this->ndt_mat_all.size())
    {
        std::cout << "NDT矩阵数据访问越界,访问前要先 PushMatrix() 过" << endl;
        exit(EXIT_FAILURE);
    }

    Eigen::Matrix4f result;
    result = this->ndt_mat_all.at(min(frame_1, frame_2));
    for (int count = min(frame_1, frame_2) + 1; count < max(frame_1, frame_2); count++)
    {
        result = this->ndt_mat_all.at(count) * result;
    }
    if (frame_1 > frame_2)
        return result;
    else if (frame_1 < frame_2)
        return result.inverse();
}
 */
BoundingBox3DFlowWHX &BoundingBox3DFlowWHX::SetDynamicThreshold(float set_number)
{
    this->dynamic_threshold = set_number;
    return *this;
}

void BoundingBox3DFlowWHX::CallbackBoxList(int frame, BoxListWHX &boxes_out, bool if_v_compensated)
{
    boxes_out.clear();
    if (this->label_data_all.IsEmpty() == false)
    {
        this->label_data_all.CallbackByFrame(frame).CallbackBoxList(boxes_out);
        if (this->calib_data_lidar.IsEmpty() == false)
        {
            this->calib_data_lidar.TransBoxList(boxes_out);
            // BoundingBox3DhwlaTohlwa(boxes_out);
            if(this->radar_data_all.IsEmpty() == false)
            {
                this->radar_data_all.CallbackByFrame(frame).TransToLidar(this->calib_data_lidar, this->calib_data_radar).AddBoxListVelo(boxes_out, if_v_compensated);
            }
        }
    }
    else
    {
        std::cout << "LabelFlow为空,读取BoundingBox出错" << endl;
        exit(EXIT_FAILURE);
    }
}
/* 
BoundingBox3DFlowWHX &BoundingBox3DFlowWHX::VelocityCompensation(int frame, const Eigen::Vector4f &self_trans_vec)
{
    for(int count = 0;count < this->boxes_flow.at(frame).size();count ++)
    {
        Eigen::Vector2f target_location_vec;
        Eigen::Vector2f self_velocity_vec;
        Eigen::Vector2f compensate_velocity_vec;
        target_location_vec(0) = this->boxes_flow.at(frame).at(count).center_x;
        target_location_vec(1) = this->boxes_flow.at(frame).at(count).center_y;

        self_velocity_vec(0) = self_trans_vec(2);
        self_velocity_vec(1) = self_trans_vec(3);
        compensate_velocity_vec = VectorProjection(self_velocity_vec,target_location_vec);
        if (self_velocity_vec.dot(target_location_vec) > 0)
        {
            this->boxes_flow.at(frame).at(count).velocity -= compensate_velocity_vec.norm();
        }
        else
        {
            this->boxes_flow.at(frame).at(count).velocity += compensate_velocity_vec.norm();
        }
    }
} 
*/

void PoseFlowWHX::ReadFolder(const std::string &folder_path)
{
    GetFileNamesInFolder(folder_path, this->pose_files_all);
}

int PoseFlowWHX::Size()
{
    return this->pose_files_all.size();
}

bool PoseFlowWHX::IsEmpty()
{
    return this->pose_files_all.empty();
}

PoseDataWHX PoseFlowWHX::CallbackByFrame(int frame)
{
    PoseDataWHX result(this->pose_files_all.at(frame));
    return result;
}