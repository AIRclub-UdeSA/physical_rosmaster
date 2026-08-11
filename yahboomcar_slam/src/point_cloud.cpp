#include "point_cloud.h"

using namespace std;

PointCloudMapper::PointCloudMapper() : Node("pointcloud_mapping")
{
    float fx_, fy_, cx_, cy_, resolution_, depthfactor_;
    int queueSize_;
    bool mbuseExact_;

    mbuseCompressed = false;
    lastKeyframeSize = 0;
    mGlobalPointCloudID = 0; // 点云IDls
    mLastGlobalPointCloudID = 0;
    queueSize = 10;

    std::string topicColor, topicDepth, topicTcw;

    this->declare_parameter<std::string>("topicColor", "/RGBD/RGB/Image");
    this->declare_parameter<std::string>("topicDepth", "/RGBD/Depth/Image");
    this->declare_parameter<std::string>("topicTcw", "/RGBD/CameraPose");
    this->declare_parameter<std::string>("local_frame_id", "camera");
    this->declare_parameter<std::string>("global_frame_id", "camera");
    this->declare_parameter<std::string>("node_path", "/root/yahboomcar_ros2_ws/yahboomcar_lucio_ws/src/yahboomcar_slam/pcl/");
    this->declare_parameter<bool>("use_viewer", false);
    this->declare_parameter<float>("fx", 517.306408);
    this->declare_parameter<float>("fy", 516.469215);
    this->declare_parameter<float>("cx", 318.643040);
    this->declare_parameter<float>("cy", 255.313989);
    this->declare_parameter<float>("resolution", 0.05);
    this->declare_parameter<float>("depthfactor", 1000.0);
    this->declare_parameter<int>("queueSize", 10);
    this->declare_parameter<bool>("buseExact", false);

    this->get_parameter<std::string>("topicColor", topicColor);
    this->get_parameter<std::string>("topicDepth", topicDepth);
    this->get_parameter<std::string>("topicTcw", topicTcw);
    this->get_parameter<std::string>("local_frame_id", local_frame_id);
    this->get_parameter<std::string>("global_frame_id", global_frame_id);
    this->get_parameter<std::string>("node_path", mNodePath);
    this->get_parameter<bool>("use_viewer", use_viewer);
    this->get_parameter<float>("fx", fx_);
    this->get_parameter<float>("fy", fy_);
    this->get_parameter<float>("cx", cx_);
    this->get_parameter<float>("cy", cy_);
    this->get_parameter<float>("resolution", resolution_);
    this->get_parameter<float>("depthfactor", depthfactor_);
    this->get_parameter<int>("queueSize", queueSize_);
    this->get_parameter<bool>("buseExact", mbuseExact_);

    mbuseExact = mbuseExact_;
    queueSize = queueSize_;
    mcx = cx_;
    mcy = cy_;
    mfx = fx_;
    mfy = fy_;
    mresolution = resolution_;
    mDepthMapFactor = depthfactor_;

    rgb_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_ptr<rclcpp::Node>(this), topicColor);
    depth_sub = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(shared_ptr<rclcpp::Node>(this), topicDepth);
    tcw_sub = std::make_shared<message_filters::Subscriber<geometry_msgs::msg::PoseStamped>>(shared_ptr<rclcpp::Node>(this), topicTcw);

    syncApproximate = std::make_shared<message_filters::Synchronizer<approximate_sync_policy>>(approximate_sync_policy(10), *rgb_sub, *depth_sub, *tcw_sub);
    syncApproximate->registerCallback(&PointCloudMapper::callback, this);

    voxel.setLeafSize(mresolution, mresolution, mresolution);
    globalMap = PointCloud::Ptr(new PointCloud());
    localMap = PointCloud::Ptr(new PointCloud());

    pub_global_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Global/PointCloudOutput", 1);
    pub_local_pointcloud = this->create_publisher<sensor_msgs::msg::PointCloud2>("/Local/PointCloudOutput", 10);
}

void PointCloudMapper::callback(const sensor_msgs::msg::Image::SharedPtr msgRGB,
                                const sensor_msgs::msg::Image::SharedPtr msgD,
                                const geometry_msgs::msg::PoseStamped::SharedPtr tcw)
{
    // RCLCPP_INFO(this->get_logger(), "callback start");
    // RCLCPP_INFO(this->get_logger(), "tcw->pose.position.x = %f",tcw->pose.position.x);
    // RCLCPP_INFO(this->get_logger(), "tcw->pose.position.y = %f",tcw->pose.position.y);
    // RCLCPP_INFO(this->get_logger(), "tcw->pose.position.z = %f",tcw->pose.position.z);

    cv::Mat color, depth, depthDisp;
    geometry_msgs::msg::PoseStamped Tcw = *tcw;
    cv_bridge::CvImageConstPtr pCvImage;

    pCvImage = cv_bridge::toCvShare(msgRGB, "bgr8");
    pCvImage->image.copyTo(color);
    pCvImage = cv_bridge::toCvShare(msgD, msgD->encoding); // imageDepth->encoding
    pCvImage->image.copyTo(depth);
    // IR image input
    if (color.type() == CV_16U)
    {
        cv::Mat tmp;
        color.convertTo(tmp, CV_8U, 0.02);
        cv::cvtColor(tmp, color, CV_GRAY2BGR);
    }
    if (depth.type() != CV_32F)
        depth.convertTo(depth, CV_32F);

    insertKeyFrame(color, depth, Tcw);
}

PointCloudMapper::~PointCloudMapper()
{
    shutdown();
}

// 由外部函数调用，每生成一个关键帧调用一次该函数
void PointCloudMapper::insertKeyFrame(cv::Mat &color, cv::Mat &depth, geometry_msgs::msg::PoseStamped &T)
{
    unique_lock<mutex> lck(keyframeMutex);
    // 已测试接受到的数据没有问题
    // cout<< BLUE<<"--------------------------------T:\n"<<T.matrix()<<WHITE<<endl;
    mvGlobalPointCloudsPose.push_back(T);

    colorImgs.push_back(color.clone());
    depthImgs.push_back(depth.clone());

    // mLastGlobalPointCloudID=mGlobalPointCloudID;
    mGlobalPointCloudID++;
    mbKeyFrameUpdate = true;
    RCLCPP_INFO(this->get_logger(), "receive a keyframe, id = %ld", mGlobalPointCloudID);
    // cout << GREEN << "--------receive a keyframe, id = " << mGlobalPointCloudID << WHITE<< endl;
}

/**
 * @function 更加关键帧生成点云、并对点云进行滤波处理
 * 备注：点云生成函数在　台式机上调用时间在0.1ｓ 左右
 */
pcl::PointCloud<PointCloudMapper::PointT>::Ptr PointCloudMapper::generatePointCloud(cv::Mat &color, cv::Mat &depth, Eigen::Isometry3d &T)
{
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    PointCloud::Ptr tmp(new PointCloud()); // point cloud is null ptr
    for (int m = 0; m < depth.rows; m += 3)
    {
        for (int n = 0; n < depth.cols; n += 3)
        {
            float d = depth.ptr<float>(m)[n] / mDepthMapFactor;
            if (d < 0.01 || d > 10)
            {
                continue;
            }
            // cout << "d = "<< d  <<endl;
            PointT p;
            p.z = d;
            p.x = (n - mcx) * p.z / mfx;
            p.y = (m - mcy) * p.z / mfy;

            p.b = color.ptr<uchar>(m)[n * 3];
            p.g = color.ptr<uchar>(m)[n * 3 + 1];
            p.r = color.ptr<uchar>(m)[n * 3 + 2];

            tmp->points.push_back(p);
        }
    }

    PointCloud::Ptr cloud_voxel_tem(new PointCloud);
    tmp->is_dense = false;
    voxel.setInputCloud(tmp);
    voxel.setLeafSize(mresolution, mresolution, mresolution);
    voxel.filter(*cloud_voxel_tem);

    PointCloud::Ptr cloud1(new PointCloud);

    // 这里对点云进行变换
    pcl::transformPointCloud(*cloud_voxel_tem, *cloud1, T.matrix());

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>(t2 - t1);

    cout << "generate point cloud from  kf-ID:" << mLastGlobalPointCloudID << ", size="
         << cloud1->points.size() << " cost time: " << time_used.count() * 1000 << " ms ." << endl;
    mLastGlobalPointCloudID++;
    return cloud1;
}

// 由于随着尺寸的增加以后,显示函数会异常退出
void PointCloudMapper::viewer()
{
    RCLCPP_INFO(this->get_logger(), "PointCloudMapper::viewer()");
    int N = 0, i = 0;
    bool KFUpdate = false;
    if (use_viewer) {
        pcl::visualization::CloudViewer pcl_viewer("viewer");
        while (rclcpp::ok())
        {
            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag)
                {
                    break;
                }
            }
            // keyframe is updated
            KFUpdate = false;
            {
                unique_lock<mutex> lck(keyframeMutex);
                N = (int)mvGlobalPointCloudsPose.size();
                KFUpdate = mbKeyFrameUpdate;
                mbKeyFrameUpdate = false;
            }
            if (KFUpdate)
            {
                std::cout << "----KFUpdate N:" << N << std::endl;
                for (i = lastKeyframeSize; i < N && i < (lastKeyframeSize + 5); i++)
                {
                    if ((mvGlobalPointCloudsPose.size() != colorImgs.size()) ||
                        (mvGlobalPointCloudsPose.size() != depthImgs.size()) ||
                        (depthImgs.size() != colorImgs.size()))
                    {
                        cout << " depthImgs.size != colorImgs.size()  " << endl;
                        continue;
                    }
                    PointCloud::Ptr tem_cloud1(new PointCloud());
                    cout << "i: " << i << "  mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << endl;
                    // 生成一幅点云大约在0.1s左右 点云数据
                    Eigen::Isometry3d transform = convert2Eigen(mvGlobalPointCloudsPose[i]);
                    // std::cout << "transform.matrix() = "<<transform.matrix() << std::endl;
                    // cv::imshow("colorImgs"+i, colorImgs[i]);  //在窗口显示图像
                    // cv::waitKey(0);  //暂停，保持图像显示，等待按键结束
                    tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i], transform);

                    if (tem_cloud1->empty())
                        continue;
                    
                    RCLCPP_INFO(this->get_logger(), "tem_cloud1 is not empty()");
                    *globalMap += *tem_cloud1;

                    sensor_msgs::msg::PointCloud2 local;
                    pcl::toROSMsg(*tem_cloud1, local); // 转换成ROS下的数据类型 最终通过topic发布
                    local.header.stamp = this->get_clock()->now();
                    local.header.frame_id = local_frame_id;
                    pub_local_pointcloud->publish(local);
                    RCLCPP_INFO(this->get_logger(), "pub_local_pointcloud->publish(local)");
                }
                lastKeyframeSize = i;
                sensor_msgs::msg::PointCloud2 output;
                pcl::toROSMsg(*globalMap, output);
                output.header.stamp = this->get_clock()->now();
                output.header.frame_id = global_frame_id;
                pub_global_pointcloud->publish(output);
                cout << "show global map, size=" << globalMap->points.size() << endl;
                pcl_viewer.showCloud(globalMap);
            }
        }
    } else {
        while (rclcpp::ok())
        {
            {
                unique_lock<mutex> lck_shutdown(shutDownMutex);
                if (shutDownFlag)
                {
                    break;
                }
            }
            // keyframe is updated
            KFUpdate = false;
            {
                unique_lock<mutex> lck(keyframeMutex);
                N = (int)mvGlobalPointCloudsPose.size();
                KFUpdate = mbKeyFrameUpdate;
                mbKeyFrameUpdate = false;
            }
            if (KFUpdate)
            {
                std::cout << "----KFUpdate N:" << N << std::endl;
                for (i = lastKeyframeSize; i < N && i < (lastKeyframeSize + 5); i++)
                {
                    if ((mvGlobalPointCloudsPose.size() != colorImgs.size()) ||
                        (mvGlobalPointCloudsPose.size() != depthImgs.size()) ||
                        (depthImgs.size() != colorImgs.size()))
                    {
                        cout << " depthImgs.size != colorImgs.size()  " << endl;
                        continue;
                    }
                    PointCloud::Ptr tem_cloud1(new PointCloud());
                    cout << "i: " << i << "  mvPosePointClouds.size(): " << mvGlobalPointCloudsPose.size() << endl;
                    // 生成一幅点云大约在0.1s左右 点云数据
                    Eigen::Isometry3d transform = convert2Eigen(mvGlobalPointCloudsPose[i]);
                    // std::cout << "transform.matrix() = "<<transform.matrix() << std::endl;
                    // cv::imshow("colorImgs"+i, colorImgs[i]);  //在窗口显示图像
                    // cv::waitKey(0);  //暂停，保持图像显示，等待按键结束
                    tem_cloud1 = generatePointCloud(colorImgs[i], depthImgs[i], transform);

                    if (tem_cloud1->empty())
                        continue;

                    RCLCPP_INFO(this->get_logger(), "tem_cloud1 is not empty()");   
                    *globalMap += *tem_cloud1;

                    sensor_msgs::msg::PointCloud2 local;
                    pcl::toROSMsg(*tem_cloud1, local); // 转换成ROS下的数据类型 最终通过topic发布
                    local.header.stamp = this->get_clock()->now();
                    local.header.frame_id = local_frame_id;
                    pub_local_pointcloud->publish(local);
                    RCLCPP_INFO(this->get_logger(), "pub_local_pointcloud->publish(local)");
                }
                lastKeyframeSize = i;
                sensor_msgs::msg::PointCloud2 output;
                pcl::toROSMsg(*globalMap, output);
                output.header.stamp = this->get_clock()->now();
                output.header.frame_id = global_frame_id;
                pub_global_pointcloud->publish(output);
                cout << "show global map, size=" << globalMap->points.size() << endl;
            }
        }
    }
}

Eigen::Matrix4f PointCloudMapper::cvMat2Eigen(const cv::Mat &cvT)
{
    Eigen::Matrix<float, 4, 4> T;
    T << cvT.at<float>(0, 0), cvT.at<float>(0, 1), cvT.at<float>(0, 2), cvT.at<float>(0, 3),
        cvT.at<float>(1, 0), cvT.at<float>(1, 1), cvT.at<float>(1, 2), cvT.at<float>(1, 3),
        cvT.at<float>(2, 0), cvT.at<float>(2, 1), cvT.at<float>(2, 2), cvT.at<float>(2, 3),
        0, 0, 0, 1;

    return T;
}
/**
 * @ 将深度图像映射成彩色图
 *
 */
void PointCloudMapper::dispDepth(const cv::Mat &in, cv::Mat &out, const float maxValue)
{
    cv::Mat tmp = cv::Mat(in.rows, in.cols, CV_8U);
    const uint32_t maxInt = 255;

    for (int r = 0; r < in.rows; ++r)
    {
        const uint16_t *itI = in.ptr<uint16_t>(r);
        uint8_t *itO = tmp.ptr<uint8_t>(r);

        for (int c = 0; c < in.cols; ++c, ++itI, ++itO)
        {
            *itO = (uint8_t)std::min((*itI * maxInt / maxValue), 255.0f);
        }
    }

    cv::applyColorMap(tmp, out, cv::COLORMAP_JET);
}

Eigen::Isometry3d PointCloudMapper::convert2Eigen(geometry_msgs::msg::PoseStamped &Tcw)
{
    Eigen::Quaterniond q = Eigen::Quaterniond(Tcw.pose.orientation.w, Tcw.pose.orientation.x,
                                              Tcw.pose.orientation.y, Tcw.pose.orientation.z);
    Eigen::AngleAxisd V6(q);
    //  V6.fromRotationMatrix<double,3,3>(q.toRotationMatrix());
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity(); // 三维变换矩阵
    T.rotate(V6);                                        // 旋转部分赋值
    T(0, 3) = Tcw.pose.position.x;
    T(1, 3) = Tcw.pose.position.y;
    T(2, 3) = Tcw.pose.position.z;
    return T;
}

void PointCloudMapper::getGlobalCloudMap(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &outputMap)
{
    unique_lock<mutex> lck_keyframeUpdated(keyFrameUpdateMutex);
    outputMap = globalMap;
}

// 复位点云显示模块
void PointCloudMapper::reset()
{
    mvGlobalPointCloudsPose.clear();
    mvGlobalPointClouds.clear();
    mGlobalPointCloudID = 0;
    mLastGlobalPointCloudID = 0;
}

void PointCloudMapper::shutdown()
{
    {
        unique_lock<mutex> lck(shutDownMutex);
        shutDownFlag = true;
    }
    string save_path = mNodePath + "resultPointCloudFile.pcd";
    pcl::io::savePCDFile(save_path, *globalMap, true);
    cout << "save pcd files to :  " << save_path << endl;
}
