#include "lib_ndt_whx.h"

using namespace LIB_NDT_WHX;

const Eigen::Matrix4f NDTCloudWHX::ZeroNDTMat()
{
    Eigen::AngleAxisf init_rotation(0, Eigen::Vector3f::UnitZ()); // 旋转矩阵
    Eigen::Translation3f init_translation(0, 0, 0);
    Eigen::Matrix4f init_guess = (init_translation * init_rotation).matrix();
    return init_guess;
}

Eigen::Matrix4f NDTCloudWHX::CallbackMatrix(const std::string &target_cloud_file, const std::string &source_cloud_file, Eigen::Matrix4f init_matrix)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    ReadBinToPcdPtr(target_cloud_file, target_cloud);
    ReadBinToPcdPtr(source_cloud_file, source_cloud);

    Eigen::Matrix4f result_pose_matrix;

    result_pose_matrix = this->CallbackMatrix(target_cloud, source_cloud, init_matrix);

    return result_pose_matrix;
}

Eigen::Matrix4f NDTCloudWHX::CallbackMatrix(const pcl::PointCloud<pcl::PointXYZI>::Ptr &target_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &source_cloud, Eigen::Matrix4f init_matrix)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_source_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZI>);

    if(init_matrix.isZero() == true)
    {
        init_matrix = this->ZeroNDTMat();
    }

    CloudVoxelGrid(source_cloud, filtered_source_cloud, 0.2);
    
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    ndt.setTransformationEpsilon(0.01);        // 设置收敛 0.1- 0.001
    ndt.setStepSize(0.08);                      // 设置步长 0.1
    ndt.setResolution(1.5);                      // 设置栅格化大小1-2m最佳
    ndt.setMaximumIterations(100);              // 最大迭代步数50-200
    ndt.setInputSource(filtered_source_cloud); // 配准的输入 对准的输入
    ndt.setInputTarget(target_cloud);          // 配准的目标 对准的目标
    ndt.align(*output_cloud, init_matrix);

    Eigen::Matrix4f result_pose_matrix = ndt.getFinalTransformation();

    return result_pose_matrix;
}

bool NDTCloudWHX::ComputePairNum(std::string pair1, std::string pair2)
{
    return pair1 < pair2;
}

Eigen::Vector4f NDTCloudWHX::MatToEKFVector(const Eigen::Matrix4f &trans_mat, float time)
{
    Eigen::Vector4f result; // x坐标,y坐标,x速度,y速度
    Eigen::Vector4f xyz_vec = trans_mat.topRightCorner<4, 1>();
    result(0) = xyz_vec(0);
    result(1) = xyz_vec(1);
    result(2) = xyz_vec(0) / time;
    result(3) = xyz_vec(1) / time;
    return result;
}

Eigen::Matrix4f NDTCloudWHX::CallbackPoseMatrix(const PoseDataWHX &pose_otl_1,const PoseDataWHX &pose_otl_2)
{
    Eigen::Matrix4f result;
    // result = pose_otl_2.CallbackMatrix4f(PoseDataWHX::code_odom_camera).inverse() * pose_otl_1.CallbackMatrix4f(PoseDataWHX::code_odom_camera);//实验2
    result = pose_otl_2.CallbackMatrix4f(PoseDataWHX::code_odom_camera) * pose_otl_1.CallbackMatrix4f(PoseDataWHX::code_odom_camera).inverse(); // 实验1
    return result;
}
