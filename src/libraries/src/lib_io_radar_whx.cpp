#include "lib_io_radar_whx.h"

using namespace LIB_IO_RADAR_WHX;

void RadarDataWHX::ReadFile(const std::string &file_path)
{
    std::ifstream input(file_path.c_str(), std::ios::in | std::ios::binary);
    if (!input.good())
    {
        std::cerr << "Could not read file: " << file_path << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    for (int count = 0; input.good(); count++)
    {
        RadarPointWHX temp;
        input.read((char *)&temp.point_x, sizeof(float));
        input.read((char *)&temp.point_y, sizeof(float));
        input.read((char *)&temp.point_z, sizeof(float));
        input.read((char *)&temp.point_rcs, sizeof(float));
        input.read((char *)&temp.v_relative, sizeof(float));
        input.read((char *)&temp.v_compensated, sizeof(float));
        input.read((char *)&temp.id_time, sizeof(float));
        radar_cloud.push_back(temp);
    }
    if (radar_cloud.size() == 0)
    {
        cout << "未检测到数据" << endl;
        exit(EXIT_FAILURE);
    }
}

int RadarDataWHX::Size()
{
    return this->radar_cloud.size();
}

void RadarDataWHX::Print()
{
    for (int count = 0; count < radar_cloud.size(); count++)
    {
        this->Print(count);
    }
}

void RadarDataWHX::Print(int number)
{
    std::cout << setw(0) << setfill(' ')
              << setw(0) << setfill(' ') << " x: " << setw(10) << setfill(' ') << radar_cloud.at(number).point_x
              << setw(0) << setfill(' ') << " y: " << setw(10) << setfill(' ') << radar_cloud.at(number).point_y
              << setw(0) << setfill(' ') << " z: " << setw(10) << setfill(' ') << radar_cloud.at(number).point_z
              << setw(0) << setfill(' ') << " rcs: " << setw(10) << setfill(' ') << radar_cloud.at(number).point_rcs
              << setw(0) << setfill(' ') << " v_r: " << setw(10) << setfill(' ') << radar_cloud.at(number).v_relative
              << setw(0) << setfill(' ') << " v_c: " << setw(10) << setfill(' ') << radar_cloud.at(number).v_compensated
              << setw(0) << setfill(' ') << " id: " << setw(10) << setfill(' ') << radar_cloud.at(number).id_time
              << endl;
}

void RadarDataWHX::Print(const BoundingBox3DWHX &box_single)
{
    for (int count = 0; count < radar_cloud.size(); count++)
    {
        if(IfPointInBoundingBox(this->ToPcl(count),box_single))
        {
            this->Print(count);
        }
    }
}

void RadarDataWHX::Print(const BoxListWHX &boxes)
{
    for(int count=0;count<boxes.size();count++)
    {
        this->Print(boxes.at(count));
    }
}

pcl::PointXYZI RadarDataWHX::ToPcl(int number)
{
    pcl::PointXYZI result;
    result.x = this->radar_cloud.at(number).point_x;
    result.y = this->radar_cloud.at(number).point_y;
    result.z = this->radar_cloud.at(number).point_z;
    result.intensity = 1;
    return result;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr RadarDataWHX::ToPcl()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int count = 0; count < radar_cloud.size(); count++)
    {
        pcl_cloud->push_back(ToPcl(count));
    }
    return pcl_cloud;
}

void RadarDataWHX::PublishToRviz(ros::Publisher &msg_publisher, const std::string frame_id)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr radar_cloud_pcl = *this;
    PublishCloudToRviz(msg_publisher, radar_cloud_pcl, frame_id);
    ros::spinOnce();
}

/* float RadarDataWHX::CallbackVelocity(BoundingBox3DWHX &box_single,bool if_v_compensated)
{
    float sum = 0;
    int count_point = 0;
    for (int count = 0; count < radar_cloud.size(); count++)
    {
        if (IfPointInBoundingBox(ToPcl(count), box_single))
        {
            if (if_v_compensated == true)
            {
                sum += this->radar_cloud.at(count).v_compensated;
            }
            else
            {
                sum += this->radar_cloud.at(count).v_relative;
            }
            count_point++;
        }
    }
    
    box_single.velocity = sum / count_point;
    if(box_single.velocity != box_single.velocity)
    {
        box_single.velocity = 23333;
    }
    return (sum / count_point);
} */

float RadarDataWHX::CallbackVelocity(BoundingBox3DWHX &box_single, bool if_v_compensated)
{
    vector<float> velocity_list;
    for (int count = 0; count < this->radar_cloud.size(); count++)
    {
        if (IfPointInBoundingBox(this->ToPcl(count), box_single))
        {
            if (if_v_compensated == true)
            {
                velocity_list.push_back(this->radar_cloud.at(count).v_compensated);
            }
            else
            {
                velocity_list.push_back(this->radar_cloud.at(count).v_relative);
            }
        }
    }
    std::sort(velocity_list.begin(), velocity_list.end());

    if (velocity_list.size() == 0)
    {
        box_single.velocity = 23333;
        return box_single.velocity;
    }

    if (velocity_list.size() % 2 == 0)
    {
        box_single.velocity = (velocity_list.at(velocity_list.size() / 2) + velocity_list.at(velocity_list.size() / 2 - 1)) / 2;
    }
    else
    {
        box_single.velocity = velocity_list.at(std::floor(velocity_list.size() / 2));
    }
    return box_single.velocity;
}

RadarPointWHX LIB_IO_RADAR_WHX::RadarDataWHX::CallbackPoint(int m_number_point)
{
    return this->radar_cloud.at(m_number_point);
}

bool CompareVeloXYVec(const Eigen::Vector2f vec_1,const Eigen::Vector2f vec_2)
{
    return vec_1(0) < vec_2(0);
}

Eigen::Vector2f RadarDataWHX::CallbackGroundVelo(const BoxListWHX &boxes, bool if_v_compensated)
{
    Eigen::Vector2f result;
    RadarDataWHX backup_this;
    backup_this << *this;
    this->RemovePointInBoxlist(boxes);

    vector<Eigen::Vector2f> velo_xy_vec;

    for (int count = 0; count < this->Size(); count++)
    {
        if (this->radar_cloud.at(count).point_z > -1.5 || this->radar_cloud.at(count).point_z < -2)
        {
            continue;
        }
        if(std::sqrt(std::pow(this->radar_cloud.at(count).point_x, 2) + std::pow(this->radar_cloud.at(count).point_y, 2)) > 100)
        {
            continue;
        }
        // if (std::sqrt(std::pow(this->radar_cloud.at(count).point_x, 2) + std::pow(this->radar_cloud.at(count).point_y, 2)) < 5)
        // {
        //     continue;
        // }
        // if(this->radar_cloud.at(count).point_rcs < -20)
        // {
        //     continue;
        // }

        Eigen::Vector2f target_velo_vec;
        float delta = std::atan(this->radar_cloud.at(count).point_z / (std::sqrt(std::pow(this->radar_cloud.at(count).point_x, 2) + std::pow(this->radar_cloud.at(count).point_y, 2))));

        float alpha = std::atan(this->radar_cloud.at(count).point_y / this->radar_cloud.at(count).point_x);
        if (if_v_compensated == true)
        {
            target_velo_vec(0) = this->radar_cloud.at(count).v_compensated * std::cos(delta) * std::cos(alpha);
            target_velo_vec(1) = this->radar_cloud.at(count).v_compensated * std::cos(delta) * std::sin(alpha);
        }
        else
        {
            target_velo_vec(0) = this->radar_cloud.at(count).v_relative * std::cos(alpha);
            target_velo_vec(1) = this->radar_cloud.at(count).v_relative * std::sin(alpha);
        }
        velo_xy_vec.push_back(target_velo_vec);
    }
    std::sort(velo_xy_vec.begin(), velo_xy_vec.end(), CompareVeloXYVec);
    *this << backup_this;
/******************************zhong wei shu*****************************/
    if (velo_xy_vec.size() % 2 == 0)
    {
        result = velo_xy_vec.at(velo_xy_vec.size() / 2);
    }
    else
    {
        result = velo_xy_vec.at(std::floor(velo_xy_vec.size() / 2));
    }
    return result;
/******************************zhong wei shu*****************************/


}

RadarDataWHX &RadarDataWHX::VelocityCompensation(const Eigen::Vector4f &self_trans_vec)
{
    for(int count = 0;count < this->Size();count ++)
    {
        RadarPointWHX radar_point = this->radar_cloud.at(count);
        Eigen::Vector2f target_location_vec(radar_point.point_x,radar_point.point_y);
        Eigen::Vector2f self_velocity_vec(self_trans_vec(2),self_trans_vec(3));
        Eigen::Vector2f compensate_velocity_vec;
        compensate_velocity_vec = VectorProjection(self_velocity_vec, target_location_vec);

        if (self_velocity_vec.dot(target_location_vec) > 0)
        {
            radar_point.v_relative -= compensate_velocity_vec.norm();
        }
        else
        {
            radar_point.v_relative += compensate_velocity_vec.norm();
        }
        this->radar_cloud[count] = radar_point;

    }
    return *this;
}

void RadarDataWHX::AddBoxListVelo(BoxListWHX &boxes,bool if_v_compensated)
{
    for (int count = 0; count < boxes.size(); count++)
    {
        this->CallbackVelocity(boxes.at(count),if_v_compensated);
    }
}

void RadarDataWHX::AddBoxListVelo(BoxListWHX &boxes, int box_number,bool if_v_compensated)
{
    this->CallbackVelocity(boxes.at(box_number), if_v_compensated);
}

RadarDataWHX &RadarDataWHX::Transform(const Eigen::Matrix4f &mat)
{
    Eigen::Vector4f point_source, point_target;
    pcl::PointCloud<pcl::PointXYZI>::Ptr source_cloud = this->ToPcl();
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::transformPointCloud(*source_cloud, *target_cloud, mat);

    for (int count = 0; count < target_cloud->size(); count++)
    {
        this->radar_cloud.at(count).point_x = target_cloud->at(count).x;
        this->radar_cloud.at(count).point_y = target_cloud->at(count).y;
        this->radar_cloud.at(count).point_z = target_cloud->at(count).z;
    }
    return *this;
}

RadarDataWHX &RadarDataWHX::TransToLidar(const CalibDataWHX &lidar_calib, const CalibDataWHX &radar_calib)
{
    Eigen::Matrix4f mat_vel_to_cam_inverse, mat_rect_inverse, mat_radar_lidar;
    mat_vel_to_cam_inverse = lidar_calib.CallbackMatrix4f(CalibDataWHX::vtc_code).inverse();
    mat_rect_inverse = lidar_calib.CallbackMatrix4f(CalibDataWHX::rect_code).inverse();

    mat_radar_lidar = radar_calib.CallbackMatrix4f(CalibDataWHX::rect_code) * radar_calib.CallbackMatrix4f(CalibDataWHX::vtc_code);
    mat_radar_lidar = mat_vel_to_cam_inverse * mat_rect_inverse * mat_radar_lidar;
    this->Transform(mat_radar_lidar);
    return *this;
}

bool RadarDataWHX::IsEmpty()
{
    return this->radar_cloud.empty();
}

RadarDataWHX &RadarDataWHX::Pushback(const RadarPointWHX &point)
{
    this->radar_cloud.push_back(point);
    return *this;
}

RadarDataWHX &RadarDataWHX::RemovePointInBoxlist(const BoxListWHX &boxlist)
{
    RemovePointInBoxlist(boxlist,*this);
    return *this;
}

RadarDataWHX &RadarDataWHX::RemovePointInBoxlist(const BoxListWHX &boxlist,RadarDataWHX &out)
{
    RadarDataWHX out_cloud;
    bool flag = true;
    for (int count_point = 0; count_point < this->radar_cloud.size(); count_point++)
    {
        flag = true;
        pcl::PointXYZI point_single;
        point_single = this->ToPcl(count_point);
        for (int count_box = 0; count_box < boxlist.size(); count_box++)
        {
            if (IfPointInBoundingBox(point_single, boxlist.at(count_box)) == true)
            {
                flag = false;
            }
        }
        if(flag == true)
        {
            out_cloud.Pushback(this->radar_cloud.at(count_point));
        }
    }
    out << out_cloud;
    return *this;
}

RadarDataWHX &LIB_IO_RADAR_WHX::RadarDataWHX::RemovePointInGround(void)
{
    RadarDataWHX out_cloud;
    bool flag = true;
    for (int count_point = 0; count_point < this->radar_cloud.size(); count_point++)
    {
        flag = true;
        pcl::PointXYZI point_single;
        point_single = this->ToPcl(count_point);
        if (!(this->radar_cloud.at(count_point).point_z > -1.5 || this->radar_cloud.at(count_point).point_z < -2))
        {
            flag=false;
        }
        if (!(std::sqrt(std::pow(this->radar_cloud.at(count_point).point_x, 2) + std::pow(this->radar_cloud.at(count_point).point_y, 2)) < 100))
        {
            flag = false;
        }
        // // if (std::sqrt(std::pow(this->radar_cloud.at(count).point_x, 2) + std::pow(this->radar_cloud.at(count).point_y, 2)) < 5)
        // // {
        // //     continue;
        // // }
        // if (!(this->radar_cloud.at(count_point).point_rcs < -10))
        // {
        //     flag = false;
        // }
        if (flag == true)
        {
            out_cloud.Pushback(this->radar_cloud.at(count_point));
        }
    }
    *this << out_cloud;
    return *this;
}

RadarDataWHX::operator pcl::PointCloud<pcl::PointXYZI>::Ptr()
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZI>);
    for (int count = 0; count < radar_cloud.size(); count++)
    {
        pcl_cloud->push_back(ToPcl(count));
    }
    return pcl_cloud;
}

