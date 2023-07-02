#include "lib_detection_whx.h"
#include "lib_io_pointcloud_whx.h"
#include "lib_io_boundingbox_whx.h"
#include "lib_io_calib_whx.h"
#include "lib_io_radar_whx.h"
#include "lib_io_label_whx.h"
#include "lib_io_flow_whx.h"
#include "lib_EKF_whx.h"
#include "lib_others_whx.h"
#include "lib_cluster_whx.h"
#include "libs_system_all.h"
#include "lib_ground_whx.h"

#define DEF_ENABLE_GROUND_CULLING 1  /*启用地面滤除功能*/
#define DEF_ENABLE_BOX_CULLING 1     /*启用点云聚类功能*/
#define DEF_ENABLE_EKF 1             /*启用卡尔曼滤波功能*/

using namespace std;
using namespace LIB_DETECTION_WHX;
using namespace LIB_IO_POINTCLOUD_WHX;
using namespace LIB_IO_BOUNDINGBOX_WHX;
using namespace LIB_IO_CALIB_WHX;
using namespace LIB_IO_RADAR_WHX;
using namespace LIB_IO_LABEL_WHX;
using namespace LIB_IO_FLOW_WHX;
using namespace LIB_EKF_WHX;
using namespace LIB_CLUSTER_WHX;
using namespace LIB_GROUND_WHX;

int main(int argc, char **argv)
{

    /**
     * @description: 初始变量定义，无需修改
     */
    ros::init(argc, argv, "Dynamic_flow_node");
    ros::NodeHandle nh_msg;
    ros::Time::init();
    ros::Time time_start, time_finish;
    ros::Time time_zero = ros::Time::now();
    ExtendedKalmanFilterWHX tool_ekf; // EKF工具
    GroundWHX tool_ground;
    std::cout << "开始:" << endl;

    /**
     * @description: ros消息发布定义
     */
    ros::Publisher pub_boxlist_now = nh_msg.advertise<visualization_msgs::MarkerArray>("boxes_now", 1, true);
    ros::Publisher pub_cloud_now = nh_msg.advertise<sensor_msgs::PointCloud2>("cloud_now", 1, true);
    ros::Publisher pub_cloud_filter_now = nh_msg.advertise<sensor_msgs::PointCloud2>("cloud_filter_now", 1, true);
    ros::Publisher pub_cloud_object_now = nh_msg.advertise<sensor_msgs::PointCloud2>("cloud_object_now", 1, true);
    ros::Publisher pub_radar_now = nh_msg.advertise<sensor_msgs::PointCloud2>("radar_now", 1, true);

    /**
     * @description: 数据集路径读取
     */
    LabelDataFlowWHX data_label_all(argv[1]);
    RadarDataFlowWHX data_radar_all(argv[2]);
    PointCloudFlowWHX data_cloud_all(argv[3]);
    CalibDataWHX data_calib_radar_all(argv[4]);
    CalibDataWHX data_calib_lidar_all(argv[5]);

    /**
     * @description: 点云、边界框、矩阵、路径等变量定义
     */
    BoxListWHX data_boxlist_now;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> data_cloud_vector_filter;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> data_cloud_vector;
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> data_cloud_vector_object;
    vector<PoseDataWHX> data_pose_vector;
    vector<RadarDataWHX> data_radar_vector;

    /**
     * @description: 全局参数赋值
     */
    ros::Rate loop_rate(4); // 显示帧率
    tool_ekf.DefaultSet(0.1);
    int data_max_frame = 0;

    for (int frame = 0; frame < data_cloud_all.Size() - 2 && nh_msg.ok(); data_max_frame = frame++)
    {
        cout << "*********" << frame << "************" << endl;

        /**
         * @description: 向容器中填入当前帧的点云数据
         */
        data_cloud_vector.push_back(
#if DEF_ENABLE_GROUND_CULLING
            tool_ground.PointCb(data_cloud_all.CallbackPtrByFrame(frame)).CallbackNoGround()
#else
            data_cloud_all.CallbackPtrByFrame(frame)
#endif
        );
        data_radar_vector.push_back(data_radar_all.CallbackByFrame(frame).TransToLidar(data_calib_lidar_all, data_calib_radar_all));

        time_start = ros::Time::now();
        /**
         * @description: 聚类当前帧，得到Boxlist，做适当扩展
         */
        PointCloudCluster(data_cloud_vector.at(frame), data_boxlist_now);
        ExpandBoundingBox3D(data_boxlist_now, 0.6);

        /**
         * @description: 获得地面速度和自车运动估计
         */
        Eigen::Vector2f ground_velo;
        Eigen::Vector4f self_velo;
        ground_velo = data_radar_vector.at(frame).CallbackGroundVelo(data_boxlist_now);
        self_velo << 0, 0, (ground_velo(0)), (ground_velo(1));

        /**
         * @description: 使用匀速运动模型做自车运动速度的卡尔曼滤波
         */
#if DEF_ENABLE_EKF
        tool_ekf.UpdateMeasurement(self_velo);
        self_velo = tool_ekf.GetResult();
#else
#endif

        /**
         * @description: 参照自车运动估计补偿雷达点云速度并获得被测目标的地面速度
         */
        data_radar_vector.at(frame).VelocityCompensation(self_velo).AddBoxListVelo(data_boxlist_now, false);

        /**
         * @description: 第一帧的初始化
         */
        if (frame == 0)
        {
            data_cloud_vector_filter.push_back(data_cloud_vector.at(frame));
            data_cloud_vector_object.push_back(data_cloud_vector.at(frame));
            continue;
        }

        /**
         * @description: 设定动态阈值
         */
        SetIfDynamic(data_boxlist_now, 1.0);

        time_finish = ros::Time::now();
        cout << "剔除用时：" << time_finish - time_start << endl;
        /**
         * @description: 从激光点云剔除动态点，并做记录
         */
        data_cloud_vector_filter.push_back(
#if DEF_ENABLE_BOX_CULLING
            RemoveBoundingBoxesFromCloud(data_cloud_vector.at(frame), data_boxlist_now)
#else
            data_cloud_vector.at(frame)
#endif
        );

        data_cloud_vector_object.push_back(
#if DEF_ENABLE_BOX_CULLING
            ReserveBoundingBoxesFromCloud(data_cloud_vector.at(frame), data_boxlist_now)
#else
            data_cloud_vector.at(frame)
#endif
        );

        /**
         * @description: Rviz消息发布
         */
        if (true)
        {
            PublishBoxesToRviz(pub_boxlist_now, data_boxlist_now, SetColor(0, 1, 0, 1), "odom", ros::Duration(loop_rate.expectedCycleTime() * 1.5));
            PublishCloudToRviz(pub_cloud_filter_now, data_cloud_vector_filter.at(frame), "odom");
            PublishCloudToRviz(pub_cloud_object_now, data_cloud_vector_object.at(frame), "odom");
            PublishCloudToRviz(pub_cloud_now, data_cloud_vector.at(frame), "odom");
            data_radar_vector.at(frame).PublishToRviz(pub_radar_now, "odom");
            ros::spinOnce();
            // loop_rate.sleep();

            if (frame == std::atoi(argv[6]))
            {

                while (nh_msg.ok())
                {
                    PublishBoxesToRviz(pub_boxlist_now, data_boxlist_now, SetColor(0, 1, 0, 1), "odom", ros::Duration(loop_rate.expectedCycleTime() * 1.5));
                    PublishCloudToRviz(pub_cloud_filter_now, data_cloud_vector_filter.at(frame), "odom");
                    PublishCloudToRviz(pub_cloud_object_now, data_cloud_vector_object.at(frame), "odom");
                    PublishCloudToRviz(pub_cloud_now, data_cloud_vector.at(frame), "odom");
                    data_radar_vector.at(frame).PublishToRviz(pub_radar_now, "odom");
                    ros::spinOnce();
                    loop_rate.sleep();
                }
                /*特定帧处理*/
            }
        }

        if (frame >= 10)
        {
            data_cloud_vector.at(frame - 10).reset();
            data_cloud_vector_filter.at(frame - 10).reset();
            data_cloud_vector_object.at(frame - 10).reset();
        }
    }

    /**
     * @description: 运行结束，持续发布最后一帧消息
     */
    std::cout << "end" << endl;
    while (nh_msg.ok())
    {
        PublishBoxesToRviz(pub_boxlist_now, data_boxlist_now, SetColor(0, 1, 0, 1), "odom", ros::Duration(loop_rate.expectedCycleTime() * 1.5));
        PublishCloudToRviz(pub_cloud_now, data_cloud_vector_filter.at(data_max_frame), "odom");
        PublishCloudToRviz(pub_cloud_now, data_cloud_vector_object.at(data_max_frame), "odom");
        data_radar_vector.at(data_max_frame).PublishToRviz(pub_radar_now, "odom");
        ros::spinOnce();
        loop_rate.sleep();
    }

    ros::spin();
    return 0;
}
