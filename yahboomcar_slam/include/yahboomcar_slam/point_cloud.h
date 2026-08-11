#ifndef POINTCLOUDMAPPING_H
#define POINTCLOUDMAPPING_H

#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <mutex>
#include <thread>
#include <chrono>
#include <condition_variable>
#include <ctime>
#include <climits>
#include <Eigen/Core>
#include <Eigen/Geometry> // Eigen 几何模块

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/compression/compression_profiles.h>
#include <pcl/compression/octree_pointcloud_compression.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <cv_bridge/cv_bridge.h>

#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

using namespace std;

class PointCloudMapper: public rclcpp::Node
{
public:
    typedef pcl::PointXYZRGBA PointT;
    typedef pcl::PointCloud<PointT> PointCloud;
    bool mbuseExact, mbuseCompressed, use_viewer;
    std::string local_frame_id, global_frame_id;
    size_t queueSize;

    PointCloudMapper();

    ~PointCloudMapper();

    Eigen::Isometry3d convert2Eigen(geometry_msgs::msg::PoseStamped &Tcw);

    // 插入一个keyframe，会更新一次地图
    void insertKeyFrame(cv::Mat &color, cv::Mat &depth, geometry_msgs::msg::PoseStamped &Tcw);

    void viewer();

    void getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &outputMap);

    void reset();

    void shutdown();

    void callback(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                    const sensor_msgs::msg::Image::SharedPtr msgD,  
                    const geometry_msgs::msg::PoseStamped::SharedPtr tcw);
protected:
    unsigned int index = 0;
    float mresolution = 0.04;
    float mDepthMapFactor = 1000.0;
    float mcx = 0, mcy = 0, mfx = 0, mfy = 0;
    std::string mNodePath;

    pcl::VoxelGrid<PointT> voxel; // 点云显示精度
    int lastKeyframeSize = 0;       //
    size_t mGlobalPointCloudID = 0; // 点云ID
    size_t mLastGlobalPointCloudID = 0;

    // data to generate point clouds
    vector<cv::Mat> colorImgs, depthImgs; // image buffer
    cv::Mat depthImg, colorImg, mpose;
    vector<PointCloud> mvGlobalPointClouds; // 存储关键帧对应的点云序列
    vector<geometry_msgs::msg::PoseStamped> mvGlobalPointCloudsPose;

    PointCloud::Ptr globalMap, localMap;

    bool shutDownFlag = false; // 程序退出标志位
    mutex shutDownMutex;

    bool mbKeyFrameUpdate = false; // 有新的关键帧插入
    mutex keyframeMutex;
    mutex keyFrameUpdateMutex;
    mutex deletekeyframeMutex;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_global_pointcloud;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_local_pointcloud;

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::msg::Image, sensor_msgs::msg::Image, geometry_msgs::msg::PoseStamped> approximate_sync_policy;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> rgb_sub;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> depth_sub;
    std::shared_ptr<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>> tcw_sub;

    std::shared_ptr<message_filters::Synchronizer<approximate_sync_policy>> syncApproximate;

    PointCloud::Ptr generatePointCloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T);

    void compressPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, std::stringstream &compressedData);

    void depressPointCloud(std::stringstream &compressedData, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloudOut);

    Eigen::Matrix4f cvMat2Eigen(const cv::Mat &cvT);

    void dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue);
};
#endif // POINTCLOUDMAPPING_H
