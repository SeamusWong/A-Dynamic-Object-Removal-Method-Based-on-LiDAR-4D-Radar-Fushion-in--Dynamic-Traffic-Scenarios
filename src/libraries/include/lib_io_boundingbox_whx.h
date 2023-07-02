#ifndef LIB_IO_BOUNDINGBOX_WHX_
#define LIB_IO_BOUNDINGBOX_WHX_

#include "libs_system_all.h"
#include "lib_others_whx.h"

using namespace std;

namespace LIB_IO_BOUNDINGBOX_WHX
{
    struct BoundingBox3DWHX
    {
        float box_height, box_width, box_length;
        float center_x, center_y, center_z;
        float angle_yaw;
        std::string type_name = "NONE";
        float velocity = NAN;
        bool if_dynamic = true;
        int id;
    };

    typedef vector<BoundingBox3DWHX> BoxListWHX;

    bool IfPointInBoundingBox(const pcl::PointXYZI &point_single, const BoundingBox3DWHX &box);
    void SetBoundingBox(BoundingBox3DWHX &box, float h, float w, float l, float x, float y, float z, float yaw, std::string type_name = "NONE", float velocity = 0);
    void BoxlistChange(BoxListWHX &boxes, float m_h_add=0.0, float m_w_add=0.0, float m_l_add=0.0, float m_x_add=0.0, float m_y_add=0.0, float m_z_add=0.0, float m_yaw_add=0.0);

    void From3DBoxToCornerPoint(const BoundingBox3DWHX &box, Eigen::Matrix<float, 3, 8> &corners_point);
    void FromPointToLineMaker(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2, visualization_msgs::Marker &line_maker, const std_msgs::ColorRGBA &color = std_msgs::ColorRGBA(), const std::string frame_id = "odom", const ros::Duration &lifetime = ros::Duration(1));
    void From3DBoxIDToMaker(const BoundingBox3DWHX &box, visualization_msgs::Marker &maker, const std_msgs::ColorRGBA &color = std_msgs::ColorRGBA(),const std::string frame_id = "odom",const ros::Duration &lifetime = ros::Duration(1));
    void From3DBoxesToMakerArray(const BoxListWHX &boxes, visualization_msgs::MarkerArray &boxes_array_maker, const std_msgs::ColorRGBA &color = std_msgs::ColorRGBA(), const std::string frame_id = "odom",const ros::Duration &lifetime = ros::Duration(1));

    void PrintBoxes(const BoundingBox3DWHX &box_single);
    void PrintBoxes(const BoxListWHX &boxes);
    void PrintBoxes(const BoxListWHX &boxes,const std::string target_type);

    void PublishBoxesToRviz(ros::Publisher &msg_publisher, const BoxListWHX &boxes, const std_msgs::ColorRGBA &color = std_msgs::ColorRGBA(), const std::string frame_id = "odom", const ros::Duration &lifetime = (ros::Duration(1)));
    void TramsformBoundingBox3D(BoxListWHX &boxes, Eigen::Matrix4f matrix);
    void TramsformBoundingBox3D(BoundingBox3DWHX &box_single, Eigen::Matrix4f matrix);

    void BoundingBox3DhwlaTohlwa(BoxListWHX &boxes);
    void BoundingBox3DhwlaTohlwa(BoundingBox3DWHX &box_single);

    void ExpandBoundingBox3D(BoxListWHX &boxes, float expand_meter);
    void ExpandBoundingBox3D(BoundingBox3DWHX &box_single, float expand_meter);
    
    float BoundingBoxCentralDistance(const BoundingBox3DWHX &box1, const BoundingBox3DWHX &box2);
    void SetIfDynamic(BoxListWHX &boxes,float dynamic_threshold = 0.0);
    void RemoveStaticBoundingBoxes(BoxListWHX &boxes);
    void RemoveDynamicBoundingBoxes(BoxListWHX &boxes);

    void CopyBoundingBox(BoxListWHX &boxes_1, const BoxListWHX &boxes_2);

    void BoundingBoxVelocityCompensation(BoundingBox3DWHX &box,const Eigen::Vector4f &self_trans_vec);
    void BoundingBoxVelocityCompensation(BoxListWHX &box_list,const Eigen::Vector4f &self_trans_vec);
    Eigen::VectorXf VectorProjection(const Eigen::VectorXf &vec_from, const Eigen::VectorXf &vec_to);

    bool operator==(const BoundingBox3DWHX &box_1,const BoundingBox3DWHX &box_2);
    std::ostream &operator<<(std::ostream &os, const BoxListWHX &out);
    std::ostream &operator<<(std::ostream &os, const BoundingBox3DWHX &out);
    BoxListWHX &operator<<(BoxListWHX &boxes_1, const BoxListWHX &boxes_2);
}

#endif