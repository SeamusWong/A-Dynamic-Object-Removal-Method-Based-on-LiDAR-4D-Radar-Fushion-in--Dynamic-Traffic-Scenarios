/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2023-03-07 14:19:54
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2023-06-19 23:52:08
 * @FilePath: /Dynamic_filter/src/libraries/src/lib_io_boundingbox_whx.cpp
 * @Description:
 *
 * Copyright (c) 2023 by ${git_name_email}, All Rights Reserved.
 */
#include "lib_io_boundingbox_whx.h"

bool LIB_IO_BOUNDINGBOX_WHX::IfPointInBoundingBox(const pcl::PointXYZI &point_single, const BoundingBox3DWHX &box)
{
    Eigen::Matrix<float, 3, 8> corner_point;
    // Eigen::Matrix<float, 3, 8> calc_point;
    From3DBoxToCornerPoint(box, corner_point);
    // std::cout << corner_point<< endl;

    // 0点为边界框中xyz均最大的点，鸟瞰图中0，1，2，3逆时针排列在同一平面，下一平面同顺序4，5，6，7
    // P为待测点
    // 计算是否在左右两个面中间
    Eigen::Vector3f vector_0_P;
    Eigen::Vector3f vector_1_P;
    Eigen::Vector3f vector_1_0;
    // 计算向量0P
    vector_0_P(0) = point_single.x - corner_point(0, 0);
    vector_0_P(1) = point_single.y - corner_point(1, 0);
    vector_0_P(2) = point_single.z - corner_point(2, 0);
    // 计算向量1P
    vector_1_P(0) = point_single.x - corner_point(0, 1);
    vector_1_P(1) = point_single.y - corner_point(1, 1);
    vector_1_P(2) = point_single.z - corner_point(2, 1);
    // 计算向量10
    vector_1_0(0) = corner_point(0, 0) - corner_point(0, 1);
    vector_1_0(1) = corner_point(1, 0) - corner_point(1, 1);
    vector_1_0(2) = corner_point(2, 0) - corner_point(2, 1);
    // 1P点乘10
    float dot_1P_10;
    dot_1P_10 = vector_1_P.dot(vector_1_0);
    // 0P点乘10
    float dot_0P_10;
    dot_0P_10 = vector_0_P.dot(vector_1_0);
    // 计算是否在上下两个面中间
    Eigen::Vector3f vector_5_1;
    Eigen::Vector3f vector_5_P;
    // 计算向量51
    vector_5_1(0) = corner_point(0, 1) - corner_point(0, 5);
    vector_5_1(1) = corner_point(1, 1) - corner_point(1, 5);
    vector_5_1(2) = corner_point(2, 1) - corner_point(2, 5);
    // 计算向量5P
    vector_5_P(0) = point_single.x - corner_point(0, 5);
    vector_5_P(1) = point_single.y - corner_point(1, 5);
    vector_5_P(2) = point_single.z - corner_point(2, 5);
    // 51点乘5P
    float dot_51_5P;
    dot_51_5P = vector_5_1.dot(vector_5_P);
    // 51点乘1P
    float dot_51_1P;
    dot_51_1P = vector_5_1.dot(vector_1_P);
    // 计算是否在上下两个面中间
    Eigen::Vector3f vector_6_5;
    Eigen::Vector3f vector_6_P;
    // 计算向量65
    vector_6_5(0) = corner_point(0, 5) - corner_point(0, 6);
    vector_6_5(1) = corner_point(1, 5) - corner_point(1, 6);
    vector_6_5(2) = corner_point(2, 5) - corner_point(2, 6);
    // 计算向量6P
    vector_6_P(0) = point_single.x - corner_point(0, 6);
    vector_6_P(1) = point_single.y - corner_point(1, 6);
    vector_6_P(2) = point_single.z - corner_point(2, 6);
    // 6P点乘65
    float dot_6P_65;
    dot_6P_65 = vector_6_P.dot(vector_6_5);
    // 5P点乘65
    float dot_5P_65;
    dot_5P_65 = vector_5_P.dot(vector_6_5);

    if (dot_1P_10 * dot_0P_10 <= 0 && dot_51_5P * dot_51_1P <= 0 && dot_6P_65 * dot_5P_65 <= 0)
    {
        return true;
    }
    else
    {
        return false;
    }
}

void LIB_IO_BOUNDINGBOX_WHX::SetBoundingBox(BoundingBox3DWHX &box, float h, float w, float l, float x, float y, float z, float yaw, std::string type_name, float velocity)
{
    box.box_height = h;
    box.box_width = w;
    box.box_length = l;
    box.center_x = x;
    box.center_y = y;
    box.center_z = z;
    box.angle_yaw = yaw;
    box.type_name = type_name;
    box.velocity = velocity;
}

void LIB_IO_BOUNDINGBOX_WHX::BoxlistChange(BoxListWHX &boxes, float m_h_add, float m_w_add, float m_l_add, float m_x_add, float m_y_add, float m_z_add, float m_yaw_add)
{
    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        boxes.at(count_box).box_height += m_h_add;
        boxes.at(count_box).box_width += m_w_add;
        boxes.at(count_box).box_length += m_l_add;
        boxes.at(count_box).center_x += m_x_add;
        boxes.at(count_box).center_y += m_y_add;
        boxes.at(count_box).center_z += m_z_add;
        boxes.at(count_box).velocity += m_yaw_add;
    }
}

Eigen::Matrix3f RotZ(float t)
{
    float c, s;
    c = cos(t);
    s = sin(t);
    Eigen::Matrix3f rot_mat;
    rot_mat << c, -s, 0, s, c, 0, 0, 0, 1;
    return rot_mat;
}

void LIB_IO_BOUNDINGBOX_WHX::From3DBoxToCornerPoint(const BoundingBox3DWHX &box_single, Eigen::Matrix<float, 3, 8> &corners_point)
{
    float l, w, h;
    // Eigen::AngleAxisf rot(box.angle_yaw, Eigen::Vector3f::UnitZ());
    Eigen::Matrix3f rot_mat;
    rot_mat = RotZ(box_single.angle_yaw);
    l = box_single.box_length;
    w = box_single.box_width;
    h = box_single.box_height;
    float x_corners[8] = {l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2};
    float y_corners[8] = {w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2};
    float z_corners[8] = {h / 2, h / 2, h / 2, h / 2, -h / 2, -h / 2, -h / 2, -h / 2};

    // Eigen::Matrix<float, 3,8> corners_point;
    corners_point << l / 2, l / 2, -l / 2, -l / 2, l / 2, l / 2, -l / 2, -l / 2,
        w / 2, -w / 2, -w / 2, w / 2, w / 2, -w / 2, -w / 2, w / 2,
        h / 2, h / 2, h / 2, h / 2, -h / 2, -h / 2, -h / 2, -h / 2;

    corners_point = rot_mat * corners_point;

    for (int count = 0; count < 8; count++)
    {
        corners_point(0, count) = corners_point(0, count) + box_single.center_x;
        corners_point(1, count) = corners_point(1, count) + box_single.center_y;
        corners_point(2, count) = corners_point(2, count) + box_single.center_z;
    }
    // corners_point.transpose();
}

void LIB_IO_BOUNDINGBOX_WHX::FromPointToLineMaker(const geometry_msgs::Point point_1, const geometry_msgs::Point point_2, visualization_msgs::Marker &line_maker, const std_msgs::ColorRGBA &color, const std::string frame_id, const ros::Duration &lifetime)
{
    line_maker.header.frame_id = frame_id;
    line_maker.header.stamp = ros::Time::now();
    line_maker.type = line_maker.LINE_LIST;
    line_maker.ns = "basic_shapes";

    line_maker.action = line_maker.ADD;
    line_maker.lifetime = lifetime;
    line_maker.id = abs(point_1.x * point_2.x) + abs(point_1.y * point_2.y) + abs(point_1.z * point_2.z);

    line_maker.color.r = color.r;
    line_maker.color.g = color.g;
    line_maker.color.b = color.b;
    line_maker.color.a = color.a;

    line_maker.scale.x = 0.1;
    line_maker.scale.y = 0.1;
    line_maker.scale.z = 0.1;

    line_maker.points.push_back(point_1);
    line_maker.points.push_back(point_2);
}

void LIB_IO_BOUNDINGBOX_WHX::From3DBoxIDToMaker(const BoundingBox3DWHX &box, visualization_msgs::Marker &maker, const std_msgs::ColorRGBA &color, const std::string frame_id, const ros::Duration &lifetime)
{
    maker.header.frame_id = frame_id;
    maker.header.stamp = ros::Time::now();
    maker.type = maker.TEXT_VIEW_FACING;
    maker.ns = "basic_shapes";

    maker.pose.orientation.w = 1.0; // 文字的方向
    maker.id = box.id + box.center_x + box.center_y + box.center_z;
    maker.lifetime = lifetime;
    maker.scale.x = 1.5;
    maker.scale.y = 1.5;
    maker.scale.z = 1.5; // 文字的大小

    maker.color.r = color.r; // 文字的颜色
    maker.color.g = color.g;
    maker.color.b = color.b;
    maker.color.a = color.a; // 必写，否则rviz无法显示

    geometry_msgs::Pose pose;
    pose.position.x = box.center_x;
    pose.position.y = box.center_y;
    pose.position.z = box.center_z + box.box_height;
    maker.pose = pose; // 文字的位置

    maker.text = "ID:";
    maker.text += std::to_string(box.id).substr(0, 5); // 文字内容
}

void LIB_IO_BOUNDINGBOX_WHX::From3DBoxesToMakerArray(const BoxListWHX &boxes, visualization_msgs::MarkerArray &boxes_array_maker, const std_msgs::ColorRGBA &color, const std::string frame_id, const ros::Duration &lifetime)
{
    visualization_msgs::Marker boxes_single_maker;
    visualization_msgs::Marker boxes_id_maker;

    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        Eigen::Matrix<float, 3, 8> corner_point;
        From3DBoxToCornerPoint(boxes.at(count_box), corner_point);
        int line_list[12][2] = {{0, 1}, {1, 2}, {2, 3}, {3, 0}, {0, 4}, {1, 5}, {2, 6}, {3, 7}, {4, 5}, {5, 6}, {6, 7}, {7, 4}};

        for (int count_line = 0; count_line < 12; count_line++)
        {
            geometry_msgs::Point point_1, point_2;
            point_1.x = corner_point(0, line_list[count_line][0]);
            point_1.y = corner_point(1, line_list[count_line][0]);
            point_1.z = corner_point(2, line_list[count_line][0]);
            point_2.x = corner_point(0, line_list[count_line][1]);
            point_2.y = corner_point(1, line_list[count_line][1]);
            point_2.z = corner_point(2, line_list[count_line][1]);

            if (boxes.at(count_box).if_dynamic == true)
            {
                FromPointToLineMaker(point_1, point_2, boxes_single_maker, color, frame_id, lifetime);
            }
            else
            {
                FromPointToLineMaker(point_1, point_2, boxes_single_maker, SetColor(0, 1, 0, 1), frame_id, lifetime);
            }

            boxes_array_maker.markers.push_back(boxes_single_maker);
        }
        if (boxes.at(count_box).id >= 0)
        {
            From3DBoxIDToMaker(boxes.at(count_box), boxes_id_maker, color, frame_id, lifetime);
            // boxes_array_maker.markers.push_back(boxes_id_maker);
        }
    }
}

void LIB_IO_BOUNDINGBOX_WHX::PrintBoxes(const BoundingBox3DWHX &box_single)
{
    std::cout << setw(0) << setfill(' ')
              << setw(0) << setfill(' ') << " 目标类型Type: " << setw(10) << setfill(' ') << box_single.type_name
                << setw(0) << setfill(' ') << " h: " << setw(10) << setfill(' ') << box_single.box_height
                << setw(0) << setfill(' ') << " w: " << setw(10) << setfill(' ') << box_single.box_width
                << setw(0) << setfill(' ') << " l: " << setw(10) << setfill(' ') << box_single.box_length
                << setw(0) << setfill(' ') << " x: " << setw(10) << setfill(' ') << box_single.center_x
                << setw(0) << setfill(' ') << " y: " << setw(10) << setfill(' ') << box_single.center_y
                << setw(0) << setfill(' ') << " z: " << setw(10) << setfill(' ') << box_single.center_z
                // << setw(0) << setfill(' ') << " yaw: " << setw(10) << setfill(' ') << box_single.angle_yaw
              << setw(0) << setfill(' ') << " 径向速度velocity: " << setw(10) << setfill(' ') << box_single.velocity
              << setw(0) << setfill(' ') << " 动态: " << setw(2) << setfill(' ') << box_single.if_dynamic
            //   << setw(0) << setfill(' ') << " id: " << setw(3) << setfill(' ') << box_single.id
              << endl;
}

void LIB_IO_BOUNDINGBOX_WHX::PrintBoxes(const BoxListWHX &boxes)
{
    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        PrintBoxes(boxes.at(count_box));
    }
}

void LIB_IO_BOUNDINGBOX_WHX::PrintBoxes(const BoxListWHX &boxes, const std::string target_type)
{
    BoxListWHX boxes_single_type;
    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        if (boxes.at(count_box).type_name == target_type)
        {
            boxes_single_type.push_back(boxes.at(count_box));
        }
    }
    PrintBoxes(boxes_single_type);
}

void LIB_IO_BOUNDINGBOX_WHX::PublishBoxesToRviz(ros::Publisher &msg_publisher, const BoxListWHX &boxes, const std_msgs::ColorRGBA &color, const std::string frame_id, const ros::Duration &lifetime)
{
    std_msgs::ColorRGBA color_cp = color;
    if (color_cp.a == 0 && color_cp.r == 0 && color_cp.g == 0 && color_cp.b == 0)
    {
        color_cp = SetColor(0, 0, 1, 0);
    }

    visualization_msgs::MarkerArray boxes_maker_array;
    From3DBoxesToMakerArray(boxes, boxes_maker_array, color_cp, frame_id, lifetime);
    msg_publisher.publish(boxes_maker_array);
    ros::spinOnce();
}

void LIB_IO_BOUNDINGBOX_WHX::TramsformBoundingBox3D(BoxListWHX &boxes, Eigen::Matrix4f matrix)
{
    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        LIB_IO_BOUNDINGBOX_WHX::TramsformBoundingBox3D(boxes.at(count_box), matrix);
    }
}

void LIB_IO_BOUNDINGBOX_WHX::TramsformBoundingBox3D(BoundingBox3DWHX &box_single, Eigen::Matrix4f matrix)
{
    Eigen::Vector4f box_center, output;
    box_center(0) = box_single.center_x;
    box_center(1) = box_single.center_y;
    box_center(2) = box_single.center_z;
    box_center(3) = 1;
    output = matrix * box_center;
    box_single.center_x = output(0);
    box_single.center_y = output(1);
    box_single.center_z = output(2);
}

void LIB_IO_BOUNDINGBOX_WHX::BoundingBox3DhwlaTohlwa(BoxListWHX &boxes)
{
    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        BoundingBox3DhwlaTohlwa(boxes.at(count_box));
    }
}

void LIB_IO_BOUNDINGBOX_WHX::BoundingBox3DhwlaTohlwa(BoundingBox3DWHX &box_single)
{
    float h, w, l;
    h = box_single.box_height;
    w = box_single.box_width;
    l = box_single.box_length;

    box_single.box_height = h;
    box_single.box_width = l;
    box_single.box_length = w;

    box_single.angle_yaw = -box_single.angle_yaw;

    // box_single.center_z += 0.7;
}

void LIB_IO_BOUNDINGBOX_WHX::ExpandBoundingBox3D(BoxListWHX &boxes, float expand_meter)
{
    for (int count_box = 0; count_box < boxes.size(); count_box++)
    {
        ExpandBoundingBox3D(boxes.at(count_box), expand_meter);
    }
}

void LIB_IO_BOUNDINGBOX_WHX::ExpandBoundingBox3D(BoundingBox3DWHX &box_single, float expand_meter)
{
    box_single.box_height += expand_meter;
    box_single.box_width += expand_meter;
    box_single.box_length += expand_meter;
    box_single.center_z += expand_meter / 2;
    box_single.center_z -= 0.6;
}

float LIB_IO_BOUNDINGBOX_WHX::BoundingBoxCentralDistance(const BoundingBox3DWHX &box1, const BoundingBox3DWHX &box2)
{
    float xx = abs(box1.center_x - box2.center_x);
    float yy = abs(box1.center_y - box2.center_y);
    float zz = abs(box1.center_z - box2.center_z);
    return sqrt(pow(xx, 2) + pow(yy, 2) + pow(zz, 2));
}

void LIB_IO_BOUNDINGBOX_WHX::SetIfDynamic(BoxListWHX &boxes, float dynamic_threshold)
{
    for (int count = 0; count < boxes.size(); count++)
    {
        if (std::abs(boxes.at(count).velocity) >= 20000)
        {
            boxes.at(count).if_dynamic = false;
            continue;
        }

        if (std::abs(boxes.at(count).velocity) >= dynamic_threshold)
        {
            boxes.at(count).if_dynamic = true;
            boxes.at(count).id = 1;
        }
        else
        {
            boxes.at(count).if_dynamic = false;
            boxes.at(count).id = 0;
        }
    };
}

void LIB_IO_BOUNDINGBOX_WHX::RemoveStaticBoundingBoxes(BoxListWHX &boxes)
{
    BoxListWHX boxes_result;
    for (int count = 0; count < boxes.size(); count++)
    {
        if (boxes.at(count).if_dynamic == true)
        {
            boxes_result.push_back(boxes.at(count));
        }
    }
    boxes.clear();
    boxes = BoxListWHX(boxes_result);
}

void LIB_IO_BOUNDINGBOX_WHX::RemoveDynamicBoundingBoxes(BoxListWHX &boxes)
{
    BoxListWHX boxes_result;
    for (int count = 0; count < boxes.size(); count++)
    {
        if (boxes.at(count).if_dynamic == false)
        {
            boxes_result.push_back(boxes.at(count));
        }
    }
    boxes.clear();
    boxes = BoxListWHX(boxes_result);
}

void LIB_IO_BOUNDINGBOX_WHX::CopyBoundingBox(BoxListWHX &boxes_1, const BoxListWHX &boxes_2)
{
    boxes_1.clear();
    for (int count = 0; count < boxes_2.size(); count++)
    {
        boxes_1.push_back(boxes_2.at(count));
    }
}

bool LIB_IO_BOUNDINGBOX_WHX::operator==(const BoundingBox3DWHX &box_1, const BoundingBox3DWHX &box_2)
{
    if (box_1.center_x == box_2.center_x && box_1.center_y == box_2.center_y && box_1.center_z == box_2.center_z)
    {
        if (box_1.box_height == box_2.box_height && box_1.box_length == box_2.box_length && box_1.box_width == box_2.box_width)
        {
            if (box_1.angle_yaw == box_2.angle_yaw)
            {
                return true;
            }
        }
    }
    return false;
}

std::ostream &LIB_IO_BOUNDINGBOX_WHX::operator<<(std::ostream &os, const BoxListWHX &out)
{
    PrintBoxes(out);
    return os;
}

std::ostream &LIB_IO_BOUNDINGBOX_WHX::operator<<(std::ostream &os, const BoundingBox3DWHX &out)
{
    PrintBoxes(out);
    return os;
}

vector<LIB_IO_BOUNDINGBOX_WHX::BoundingBox3DWHX> &LIB_IO_BOUNDINGBOX_WHX::operator<<(BoxListWHX &boxes_1, const BoxListWHX &boxes_2)
{
    boxes_1.clear();
    for (int count = 0; count < boxes_2.size(); count++)
    {
        boxes_1.push_back(boxes_2.at(count));
    }
    return boxes_1;
}

void LIB_IO_BOUNDINGBOX_WHX::BoundingBoxVelocityCompensation(BoxListWHX &box_list, const Eigen::Vector4f &self_trans_vec)
{
    for (int count = 0; count < box_list.size(); count++)
    {
        BoundingBoxVelocityCompensation(box_list.at(count), self_trans_vec);
    }
}

void LIB_IO_BOUNDINGBOX_WHX::BoundingBoxVelocityCompensation(BoundingBox3DWHX &box, const Eigen::Vector4f &self_trans_vec)
{
    Eigen::Vector2f target_location_vec;
    Eigen::Vector2f self_velocity_vec;
    Eigen::Vector2f compensate_velocity_vec;
    target_location_vec(0) = box.center_x;
    target_location_vec(1) = box.center_y;

    self_velocity_vec(0) = self_trans_vec(2);
    self_velocity_vec(1) = self_trans_vec(3);
    compensate_velocity_vec = VectorProjection(self_velocity_vec, target_location_vec);
    if (self_velocity_vec.dot(target_location_vec) > 0)
    {
        box.velocity -= compensate_velocity_vec.norm();
    }
    else
    {
        box.velocity += compensate_velocity_vec.norm();
    }
}

Eigen::VectorXf LIB_IO_BOUNDINGBOX_WHX::VectorProjection(const Eigen::VectorXf &vec_from, const Eigen::VectorXf &vec_to)
{
    Eigen::VectorXf vec_from_unit, vec_to_unit, result;

    if (vec_from.size() * vec_to.size())
    {
        result.resize(vec_from.size());
        vec_from_unit.resize(vec_from.size());
        vec_to_unit.resize(vec_from.size());
    }

    for (int count = 0; count < vec_from.size(); count++)
    {
        vec_from_unit(count) = vec_from(count) / vec_from.norm();
        vec_to_unit(count) = vec_to(count) / vec_to.norm();
        result(count) = vec_to_unit(count) * (vec_from.dot(vec_to) / vec_to.norm());
    }

    return result;
}