#ifndef PROCESSPOINTCLOUDS_H_
#define PROCESSPOINTCLOUDS_H_

#include <unordered_set>
#include <iostream> 
#include <string>  
#include <vector>
#include <ctime>
#include <chrono>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include "lib_cluster_box.h" //检测框
#include "lib_cluster_kdtree.h" //kd树分类

template <typename PointT>
class ProcessPointClouds
{
private:

public:
    ProcessPointClouds(); // Constructor

    ~ProcessPointClouds(); // Destructor

    void numPoints(typename pcl::PointCloud<PointT>::Ptr cloud);

    typename pcl::PointCloud<PointT>::Ptr FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint);

    std::unordered_set<int> Ransac2D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold);

    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTolerance, bool usePCLClustering = false, int minSize = 1, int maxSize = 1000);

    Box BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster);

    void savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file);

    typename pcl::PointCloud<PointT>::Ptr loadPcd(std::string file);

    std::vector<boost::filesystem::path> streamPcd(std::string dataPath);
};

// Constructor
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

// Destructor
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Voxel size should be large enough to help speed up the processing but not so large that object definition is completely lost  体素大小应该足够大，以帮助加快处理速度，但不要太大，以至于对象定义完全丢失
    // TOTRY: setting camera angles in environment.cpp can help pick a good region of interest. E.g. set the camera to have a top down or side overview. 在 environment.cpp 中设置相机角度可以帮助选择一个好的感兴趣区域。例如。将相机设置为自上而下或侧面概览

    auto startTime = std::chrono::steady_clock::now();
    typename pcl::PointCloud<PointT>::Ptr filteredCloud(new pcl::PointCloud<PointT>);
    // Voxel-grid subsampling of points  // TODO: check best order  点的体素栅格子采样//TODO:检查最佳顺序
    pcl::VoxelGrid<PointT> voxelGrid;
    voxelGrid.setInputCloud(cloud);
    voxelGrid.setLeafSize(filterRes, filterRes, filterRes); // using filterRes as leaf size
    voxelGrid.filter(*filteredCloud);                       // filteredCloud is a pointer, so we dereference it  是指针所以没有用引用

    // Region-based filtering 基于区域的过滤
    typename pcl::PointCloud<PointT>::Ptr croppedCloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropBox(true); // true because dealing with points inside cropBox
    cropBox.setMin(minPoint);
    cropBox.setMax(maxPoint);
    cropBox.setInputCloud(filteredCloud);
    cropBox.filter(*croppedCloud); // saving results in croppedCloud

    // Roof-point filtering  顶部点过滤
    // Using a PCL CropBox to find the roof point indices and then feeding those indices to a PCL ExtractIndices object to remove them (similar to the way the segmentation algorithm extracts points)
    // 使用PCL CropBox查找屋顶点索引，然后将这些索引提供给PCL ExtractIndices对象以删除它们（类似于分割算法提取点的方式）

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(croppedCloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
        inliers->indices.push_back(point);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(croppedCloud);
    extract.setIndices(inliers);   // indices of roof points
    extract.setNegative(true);     // removing points  移除点
    extract.filter(*croppedCloud); // extracting those indices from inliers

    // TODO: use renderBox to visualise the area where the ego car's roof points were contained  使用renderBox可视化包含自我汽车车顶点的区域
    // TOTRY: use the renderBox function to figure out how big boxes will look in the scene  使用renderBox函数计算场景中的长方体大小

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    return croppedCloud;
}

template <typename PointT>
std::unordered_set<int> Ransac2D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult; // starts as 0
    srand(time(NULL));
    // 对于最大迭代次数
    // 随机采样子集和拟合线
    // 测量每个点与拟合线之间的距离
    // 如果距离小于阈值，则将其计数为inlier
    // 与大多数进气管匹配的管路的进气管返回标记

    while (maxIterations--) // > 0
    {
        std::unordered_set<int> inliers; // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
        while (inliers.size() <= 2)
            inliers.insert(rand() % (cloud->points.size())); // using modulo, value between 0 and the size of cloud  // inliers will hold the indices of points

        float x1, y1, x2, y2;

        auto itr = inliers.begin(); // pointer to the beginning of inliers
        x1 = cloud->points[*itr].x; // checking what value it is by dereferencing the pointer
        y1 = cloud->points[*itr].y;
        itr++; // incrementing the iterator by one
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;

        float a = static_cast<float>(y1 - y2);
        float b = static_cast<float>(x2 - x1);
        float c = static_cast<float>(x1 * y2 - x2 * y1);

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers.count(i) > 0) // if point is one of the two points that make the line
                continue;

            pcl::PointXYZ point = cloud->points[i];
            float x3 = point.x; // member x from point
            float y3 = point.y;
            float distance = fabs(a * x3 + b * y3 + c) / sqrtf(static_cast<float>(a * a + b * b)); // float absolute
            if (distance <= distanceTol)
                inliers.insert(i);
        }
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

template <typename PointT>
std::unordered_set<int> Ransac3D(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
    std::unordered_set<int> inliersResult; // starts as 0
    srand(time(NULL));

    // For max iterations
    // Randomly sample subset and fit line
    // Measure distance between every point and fitted line
    // If distance is smaller than threshold count it as inlier
    // Return indicies of inliers from fitted line with most inliers

    while (maxIterations--) // > 0
    {
        std::unordered_set<int> inliers; // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
        while (inliers.size() <= 3)
            inliers.insert(rand() % (cloud->points.size())); // using modulo, value between 0 and the size of cloud  // inliers will hold the index of points

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers.begin(); // pointer to the beginning of inliers
        x1 = cloud->points[*itr].x; // checking what value it is by dereferencing the pointer
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++; // incrementing the iterator by one
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // 3D
        double u1, u2, u3, v1, v2, v3;
        u1 = (x2 - x1);
        u2 = (y2 - y1);
        u3 = (z2 - z1);
        v1 = (x3 - x1);
        v2 = (y3 - y1);
        v3 = (z3 - z1);
        // v1 [3] = {u1, u2, u3};
        // v2 [3] = {v1, v2, v3};

        double i, j, k;
        i = u2 * v3 - v2 * u3;
        j = v1 * u3 - u1 * v3;
        k = u1 * v2 - v1 * u2;

        // crossProd [3] = {i, j, k};

        double a, b, c, d;
        a = i;
        b = j;
        c = k;
        d = -1.0 * (i * x1 + j * y1 + k * z1);

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers.count(i) > 0) // if point is one of the two points that make the line
                continue;

            pcl::PointXYZ point = cloud->points[i];
            float x3 = point.x; // member x from point
            float y3 = point.y;
            float z3 = point.z;
            double distance = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c);
            if (distance <= distanceTol)
                inliers.insert(i);
        }
        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    return inliersResult;
}

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud)
{
    typename pcl::PointCloud<PointT>::Ptr obstacleCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr groundCloud(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices)
    {
        groundCloud->points.push_back(cloud->points[index]);
    }

    // Creating filtering object and extracting inliers  创建过滤对象并提取inliers
    pcl::ExtractIndices<PointT> extract;

    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacleCloud); // dereferencing pointer
    std::cerr << "PointCloud representing the planar component: " << cloud->width * cloud->height << " data points." << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacleCloud, groundCloud);

    return segResult;
}

// 分割平面
template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    std::unordered_set<int> inliersResult;                                            // starts as 0
    srand(time(NULL));

    // For max iterations
    // Randomly sample subset and fit line
    // Measure distance between every point and fitted plane
    // If distance is smaller than threshold count it as inlier
    // Return indicies of inliers from fitted line with most inliers
    // 对于最大迭代
    // 随机采样子集和拟合线
    // 测量每个点和拟合平面之间的距离
    // 如果距离小于阈值，则将其视为内点
    // 从具有最多内点的拟合线返回内点的指标

    while (maxIterations--) // > 0
    {
        std::unordered_set<int> inliers; // hash set - order doesn't matter, we're just hashing into the index  // in sets, elements have to be unique, else it won't insert them
        while (inliers.size() <= 3)
            inliers.insert(rand() % (cloud->points.size())); // using modulo, value between 0 and the size of cloud  // inliers will hold the index of points

        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers.begin(); // pointer to the beginning of inliers
        x1 = cloud->points[*itr].x; // checking value by dereferencing pointer
        y1 = cloud->points[*itr].y;
        z1 = cloud->points[*itr].z;
        itr++; // increment the iterator by one
        x2 = cloud->points[*itr].x;
        y2 = cloud->points[*itr].y;
        z2 = cloud->points[*itr].z;
        itr++;
        x3 = cloud->points[*itr].x;
        y3 = cloud->points[*itr].y;
        z3 = cloud->points[*itr].z;

        // 3D
        double u1, u2, u3, v1, v2, v3;
        u1 = (x2 - x1);
        u2 = (y2 - y1);
        u3 = (z2 - z1);
        v1 = (x3 - x1);
        v2 = (y3 - y1);
        v3 = (z3 - z1);
        // v1 [3] = {u1, u2, u3};
        // v2 [3] = {v1, v2, v3};

        double i, j, k;
        i = u2 * v3 - v2 * u3;
        j = v1 * u3 - u1 * v3;
        k = u1 * v2 - v1 * u2;

        // crossProd [3] = {i, j, k};

        double a, b, c, d;
        a = i;
        b = j;
        c = k;
        d = -1.0 * (i * x1 + j * y1 + k * z1); // for 3D

        for (int i = 0; i < cloud->points.size(); i++)
        {
            if (inliers.count(i) > 0) // if point is one of the two points that make the line
                continue;

            PointT point = cloud->points[i];
            float x3 = point.x; // member x from point
            float y3 = point.y;
            float z3 = point.z;

            double distance = fabs(a * x3 + b * y3 + c * z3 + d) / sqrt(a * a + b * b + c * c);

            if (distance <= distanceThreshold)
                inliers.insert(i);
        }

        if (inliers.size() > inliersResult.size())
        {
            inliersResult = inliers;
        }
    }

    std::unordered_set<int> inliers = inliersResult;

    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new typename pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new typename pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];

        if (inliers.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult;
    segResult.first = cloudOutliers;
    segResult.second = cloudInliers;

    // TONOTE: not using SeparateClouds function at all

    return segResult;
}

template <typename PointT>
void euclideanClusteringHelper(int index, const typename pcl::PointCloud<PointT>::Ptr cloud, std::vector<int> &cluster, std::vector<bool> &processed, KdTree<PointT> *tree, float distanceTolerance) // & cluster (works), & tree (doesn't)
{
    // If we have never processed this point before
    processed[index] = true;

    cluster.push_back(index);
    std::vector<int> nearby = tree->search(index, distanceTolerance);

    int unprocessed_points = 0;
    for (int id : nearby)
    {
        if (processed[id] != true)
        {
            unprocessed_points += 1;
        }
    }
    for (int id : nearby)
    {
        if (processed[id] != true)
        {
            euclideanClusteringHelper(id, cloud, cluster, processed, tree, distanceTolerance);
        }
    }
}

// 聚类
template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> euclideanClustering(typename pcl::PointCloud<PointT>::Ptr cloud, KdTree<PointT> *tree, float distanceTol) // TODO: should tree be passed by reference
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> processed(cloud->points.size(), false); // same size as points, each of which starts as false

    int i = 0;
    while (i < cloud->points.size())
    {
        if (processed[i] == false)
        {
            // If the point has not been processed, create a new cluster
            std::vector<int> newIdCluster;
            euclideanClusteringHelper(i, cloud, newIdCluster, processed, tree, distanceTol); // i: point id, cluster passed by reference

            typename pcl::PointCloud<PointT>::Ptr newPointCluster(new typename pcl::PointCloud<PointT>());
            for (int id : newIdCluster)
            {
                newPointCluster->points.push_back((cloud->points[id]));
            }
            clusters.push_back(newPointCluster); // Assertion failed: (px != 0), function operator->, file /usr/local/include/boost/smart_ptr/shared_ptr.hpp, line 734. // Abort trap: 6
        }
        i++;
    }

    return clusters;
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float distanceTolerance, bool usePCLClustering, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; // vector of point clouds

    if (usePCLClustering == true) // use built-in PCL euclidean-clustering functions
    {
        typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>); // KdTree object for optimised search during extraction
        tree->setInputCloud(cloud);

        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(distanceTolerance); // 2cm  //  PCL example uses 0.02
        ec.setMinClusterSize(minSize);             // PCL example uses 100
        ec.setMaxClusterSize(maxSize);             // PCL example uses 25000
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud);
        ec.extract(clusterIndices);

        for (pcl::PointIndices getIndices : clusterIndices)
        {
            typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

            for (int index : getIndices.indices)
                cloudCluster->points.push_back(cloud->points[index]);

            cloudCluster->width = cloudCluster->points.size();
            cloudCluster->height = 1;
            cloudCluster->is_dense = true;
            clusters.push_back(cloudCluster);
        }
    }
    else // use custom clustering algorithm
    {
        KdTree<PointT> *tree3D = new KdTree<PointT>(cloud); // on stack: KdTree<PointT> tree3D;  // TOCHECK: difference between () and no ()

        for (int i = 0; i < cloud->points.size(); i++)
            tree3D->insertPointIndex(i);

        clusters = euclideanClustering(cloud, tree3D, distanceTolerance);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    return clusters;
}

template <typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{
    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1)
        PCL_ERROR("Couldn't read file \n");
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    sort(paths.begin(), paths.end()); // sorting files in ascending order so playback is chronological

    return paths;
}

#endif /* PROCESSPOINTCLOUDS_H_ */
