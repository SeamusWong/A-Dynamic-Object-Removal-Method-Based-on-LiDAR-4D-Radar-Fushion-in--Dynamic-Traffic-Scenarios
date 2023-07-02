#include "lib_io_pointcloud_whx.h"

void LIB_IO_POINTCLOUD_WHX::ReadBinToPcdPtr(const std::string &in_file, const pcl::PointCloud<pcl::PointXYZI>::Ptr &point_ptr)
{
    // load point cloud
    point_ptr->clear();
    std::fstream input(in_file.c_str(), std::ios::in | std::ios::binary);
    if (!input.good())
    {
        std::cerr << "Could not read file: " << in_file << std::endl;
        exit(EXIT_FAILURE);
    }
    input.seekg(0, std::ios::beg);

    while (input.good())
    {
        pcl::PointXYZI point_temp;
        input.read((char *)&point_temp.x, 3 * sizeof(float));
        input.read((char *)&point_temp.intensity, sizeof(float));
        point_ptr->push_back(point_temp);
    }
    input.close();

    // std::cout << "完成读取，共 " << i << " points" << std::endl;
    //  pcl::PCDWriter writer;

    // Save DoN features
    // writer.write<pcl::PointXYZI>(out_file, *points);
    // pcl::io::savePCDFileASCII(out_file, *points);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LIB_IO_POINTCLOUD_WHX::ReadBinToPcdPtr(const std::string &in_file)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    ReadBinToPcdPtr(in_file, result);
    return result;
}

pcl::PointCloud<pcl::PointXYZI> LIB_IO_POINTCLOUD_WHX::ReadBinToPcd(const std::string &in_file)
{
    pcl::PointCloud<pcl::PointXYZI> result;
    ReadBinToPcdPtr(in_file, result.makeShared());
    return result;
}

void LIB_IO_POINTCLOUD_WHX::PublishCloudToRviz(ros::Publisher &msg_publisher, const pcl::PointCloud<pcl::PointXYZI>::Ptr &pub_cloud, const std::string frame_id)
{
    pub_cloud->header.frame_id = frame_id;
    msg_publisher.publish(pub_cloud);
    ros::spinOnce();
}

bool LIB_IO_POINTCLOUD_WHX::ComputePairNum(const std::string &pair1, const std::string &pair2)
{
    return pair1 < pair2;
}

// 获取所有的文件名
void LIB_IO_POINTCLOUD_WHX::GetFileNamesInFolder(const std::string &path, vector<std::string> &filenames_path_all)
{
    filenames_path_all.clear();
    DIR *dir_pointer;
    struct dirent *ptr;
    if (!(dir_pointer = opendir(path.c_str())))
    {
        cout << "Folder doesn't Exist!" << endl;
        return;
    }
    while ((ptr = readdir(dir_pointer)) != 0)
    {
        if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
        {
            filenames_path_all.push_back(path + "/" + ptr->d_name);
        }
    }
    closedir(dir_pointer);

    std::sort(filenames_path_all.begin(), filenames_path_all.end(), ComputePairNum);
}

void LIB_IO_POINTCLOUD_WHX::RemoveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud, const vector<BoundingBox3DWHX> &boxes)
{
    bool flag = true;
    for (int count_point = 0; count_point < real_cloud->size(); count_point++)
    {
        flag = true;
        pcl::PointXYZI point_single;
        point_single = real_cloud->at(count_point);
        for (int count_box = 0; count_box < boxes.size(); count_box++)
        {
            if(boxes.at(count_box).if_dynamic == false)
            {
                continue;
            }
            if (IfPointInBoundingBox(point_single, boxes.at(count_box)) == true)
            {
                flag = false;
            }
        }
        if (flag == true)
        {
            out_cloud->push_back(point_single);
        }
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LIB_IO_POINTCLOUD_WHX::RemoveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, const vector<BoundingBox3DWHX> &boxes)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    RemoveBoundingBoxesFromCloud(real_cloud,result,boxes);
    return result;
}

void LIB_IO_POINTCLOUD_WHX::ReserveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud, const vector<BoundingBox3DWHX> &boxes)
{
    bool flag = true;
    for (int count_point = 0; count_point < real_cloud->size(); count_point++)
    {
        flag = true;
        pcl::PointXYZI point_single;
        point_single = real_cloud->at(count_point);
        for (int count_box = 0; count_box < boxes.size(); count_box++)
        {
            if(boxes.at(count_box).if_dynamic == false)
            {
                continue;
            }
            if (IfPointInBoundingBox(point_single, boxes.at(count_box)) == true)
            {
                flag = false;
            }
        }
        if (flag == false)
        {
            out_cloud->push_back(point_single);
        }
    }
    pcl::PointXYZI point_single_zero(1);
    out_cloud->push_back(point_single_zero);

}

pcl::PointCloud<pcl::PointXYZI>::Ptr LIB_IO_POINTCLOUD_WHX::ReserveBoundingBoxesFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, const vector<BoundingBox3DWHX> &boxes)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    ReserveBoundingBoxesFromCloud(real_cloud, result, boxes);
    return result;
}

void LIB_IO_POINTCLOUD_WHX::RemoveRearPointFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, pcl::PointCloud<pcl::PointXYZI>::Ptr &out_cloud, float distance)
{
    bool flag = true;
    for (int count_point = 0; count_point < real_cloud->size(); count_point++)
    {
        flag = true;
        pcl::PointXYZI point_single;
        point_single = real_cloud->at(count_point);
        if (point_single.y < distance)
        {
            flag = false;
        }
        if (flag == true)
        {
            out_cloud->push_back(point_single);
        }
    }
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LIB_IO_POINTCLOUD_WHX::RemoveRearPointFromCloud(const pcl::PointCloud<pcl::PointXYZI>::Ptr &real_cloud, float distance)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    RemoveRearPointFromCloud(real_cloud, result, distance);
    return result;
}

int LIB_IO_POINTCLOUD_WHX::PointInBoundingBox(const pcl::PointCloud<pcl::PointXYZI>::Ptr &m_cloud, const BoundingBox3DWHX &m_box)
{
    int result= 0;
    for (int count_point = 0; count_point < m_cloud->size(); count_point++)
    {
        if (IfPointInBoundingBox(m_cloud->at(count_point),m_box) == true)
        {
            result ++;
        }
    }
    return result;
}

void LIB_IO_POINTCLOUD_WHX::CloudVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, const pcl::PointCloud<pcl::PointXYZI>::Ptr &filtered_cloud, float leaf_size_square)
{
    pcl::ApproximateVoxelGrid<pcl::PointXYZI> cloud_grid_filter;
    cloud_grid_filter.setInputCloud(input_cloud);
    cloud_grid_filter.setLeafSize(leaf_size_square, leaf_size_square, leaf_size_square);
    cloud_grid_filter.filter(*filtered_cloud);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr LIB_IO_POINTCLOUD_WHX::CloudVoxelGrid(const pcl::PointCloud<pcl::PointXYZI>::Ptr &input_cloud, float leaf_size_square)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr result(new pcl::PointCloud<pcl::PointXYZI>);
    CloudVoxelGrid(input_cloud,result,leaf_size_square);
    return result;
}
