//
// Created by hyx on 24-12-17.
//
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/transform_datatypes.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/conversions.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <opencv4/opencv2/opencv.hpp>

#include <condition_variable>
#include <deque>
#include <mutex>
#include <chrono>
#include <string>
#include <utility>
#include <thread>

typedef pcl::PointXYZINormal PointType;
typedef pcl::PointCloud<PointType> PointCloudXYZI;

using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class KeyframePublisher : public rclcpp::Node {
public:
    KeyframePublisher()
        : Node("publish_keyframe") {
        RCLCPP_INFO(this->get_logger(), "----> Pub KeyFrame Started.");

        // Parameters
        this->declare_parameter<bool>("save_map", true);
        this->declare_parameter<double>("filter_size_map", 0.2);
        bool save_map = this->get_parameter("save_map").as_bool();
        filter_size = this->get_parameter("filter_size_map").as_double();

        // Subscribers
        sub_odometry_ = std::make_shared<message_filters::Subscriber<nav_msgs::msg::Odometry> >(
            this, "/aft_mapped_to_init");
        sub_laser_cloud_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::PointCloud2> >(
            this, "/cloud_registered");

        // Synchronizer
        sync_ = std::make_shared<syncPolicy>(*sub_odometry_, *sub_laser_cloud_, 100);
        sync_->registerCallback(std::bind(&KeyframePublisher::laserCloudAndOdometryHandler, this, _1, _2));

        // Publishers
        pub_keyframe_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/registered_scan", 1000);
        pub_keypose_ = this->create_publisher<nav_msgs::msg::Odometry>("/state_estimation", 1000);

        // Voxel Grid Filter
        down_size_filter.setLeafSize(filter_size, filter_size, filter_size);

        // Set up transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    void laserCloudAndOdometryHandler(
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg,
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &pc_msg) {
        std::lock_guard<std::mutex> lock(mtx);

        auto odom = std::make_shared<nav_msgs::msg::Odometry>(*odom_msg);
        auto pc = std::make_shared<sensor_msgs::msg::PointCloud2>(*pc_msg);

        double time = odom_msg->header.stamp.sec + 1e-9 * odom_msg->header.stamp.nanosec;
        pose_scan_pair_buf_.push_back(std::make_pair(time, std::make_pair(odom, pc)));
    }
    //
    //
    // cv::Vec3b getColorFromHeight(float height) {
    //     if (height <= 0.05)
    //         return cv::Vec3b(255, 255, 255);
    //     // 将高度归一化到 0-1 范围
    //     height = std::min(std::max(height, 0.0f), 30.0f) / 30.0f;
    //     // 定义高度阈值和对应的颜色
    //     float heightThresholds[3] = {0.05f, 1.5f / 30.0f, 5.0f / 30.0f}; // 归一化后的阈值
    //     cv::Vec3b colors[4] = {
    //         cv::Vec3b(255, 0, 0), // 红色
    //         cv::Vec3b(255, 255, 0), // 黄色
    //         cv::Vec3b(0, 255, 0), // 绿色
    //         cv::Vec3b(0, 0, 255)
    //     }; // 蓝色（用于绿色到蓝色的渐变）
    //
    //     // 确定高度在哪个范围内
    //     int rangeIndex = -1;
    //     for (int i = 0; i < 3; ++i) {
    //         if (height <= heightThresholds[i]) {
    //             rangeIndex = i;
    //             break;
    //         }
    //     }
    //
    //     // 如果高度超过最大范围阈值，则使用最后一个范围的颜色渐变（绿色到蓝色）
    //     if (rangeIndex == -1) {
    //         rangeIndex = 3;
    //     } else if (rangeIndex == 3) {
    //         float ratio = (height - heightThresholds[2]) / (1.0f - heightThresholds[2]);
    //         int blue = static_cast<int>(255 * ratio);
    //         return cv::Vec3b(0, 255 - blue, blue);
    //     } else {
    //         // 计算颜色渐变
    //         cv::Vec3b color1 = colors[rangeIndex];
    //         cv::Vec3b color2 = colors[rangeIndex + 1];
    //         float rangeDiff = heightThresholds[rangeIndex + 1] - heightThresholds[rangeIndex];
    //         float ratio = (height - heightThresholds[rangeIndex]) / rangeDiff;
    //
    //         int r = static_cast<int>(color1[0] * (1 - ratio) + color2[0] * ratio);
    //         int g = static_cast<int>(color1[1] * (1 - ratio) + color2[1] * ratio);
    //         int b = static_cast<int>(color1[2] * (1 - ratio) + color2[2] * ratio);
    //
    //         return cv::Vec3b(r, g, b);
    //     }
    // }
    //
    // void save_2d(const PointCloudXYZI::Ptr &cloud) {
    //     // 使用体素网格滤波进行降采样
    //     double d = 2;
    //     pcl::VoxelGrid<PointType> voxel_grid;
    //     voxel_grid.setInputCloud(cloud);
    //     voxel_grid.setLeafSize(1 / d, 1 / d, 1 / d);
    //     PointCloudXYZI::Ptr filtered_cloud(new PointCloudXYZI);
    //     voxel_grid.filter(*filtered_cloud);
    //
    //     // 使用统计滤波去除噪声
    //     pcl::StatisticalOutlierRemoval<PointType> sor;
    //     sor.setInputCloud(filtered_cloud);
    //     sor.setMeanK(15);
    //     sor.setStddevMulThresh(0.5);
    //     PointCloudXYZI::Ptr cloud_filtered(new PointCloudXYZI);
    //     sor.filter(*cloud_filtered);
    //
    //     // 计算点云的投影尺寸并创建图像
    //     float max_x = -std::numeric_limits<float>::max();
    //     float min_x = std::numeric_limits<float>::max();
    //     float max_y = -std::numeric_limits<float>::max();
    //     float min_y = std::numeric_limits<float>::max();
    //     for (const auto &point: cloud_filtered->points) {
    //         max_x = std::max(max_x, point.x);
    //         min_x = std::min(min_x, point.x);
    //         max_y = std::max(max_y, point.y);
    //         min_y = std::min(min_y, point.y);
    //     }
    //
    //     int width = std::ceil(max_x - min_x) + 1;
    //     int height = std::ceil(max_y - min_y) + 1;
    //     int img_width = width * d;
    //     int img_height = height * d;
    //
    //     cv::Mat image(img_height, img_width, CV_8UC3, cv::Scalar(123, 124, 56));
    //     std::unordered_map<int, std::unordered_map<int, float> > min_heights;
    //
    //     // 投影点云到二维平面并记录最低高度
    //     for (const auto &point: cloud_filtered->points) {
    //         if (point.z > 30) continue;
    //
    //         int x_index = static_cast<int>((point.x - min_x) * d);
    //         int y_index = static_cast<int>((point.y - min_y) * d);
    //
    //         if (min_heights.find(x_index) == min_heights.end() ||
    //             min_heights[x_index].find(y_index) == min_heights[x_index].end() ||
    //             point.z < min_heights[x_index][y_index]) {
    //             min_heights[x_index][y_index] = point.z;
    //         }
    //     }
    //
    //     // 将当前里程计位置添加到路径点
    //     pcl::PointXYZ odom_point;
    //     odom_point.x = odomAftMapped.pose.pose.position.x;
    //     odom_point.y = odomAftMapped.pose.pose.position.y;
    //     odom_point.z = odomAftMapped.pose.pose.position.z;
    //     path_points->points.push_back(odom_point);
    //
    //     // std::cout<<path_points->points.size()<<std::endl;
    //
    //     // 重新构建KDTree
    //     kdtree.setInputCloud(path_points);
    //
    //     // 投影并设置颜色
    //     for (const auto &[x_index, heights]: min_heights) {
    //         for (const auto &[y_index, height]: heights) {
    //             // 查询最近的路径点高度
    //             float min_dist = std::numeric_limits<float>::max();
    //             float nearest_path_height = 0;
    //             pcl::PointXYZ searchPoint;
    //             searchPoint.x = min_x + x_index / d;
    //             searchPoint.y = min_y + y_index / d;
    //             searchPoint.z = 30; // 初始化为一个很大的值，因为我们只关心z坐标
    //
    //             std::vector<int> pointIdxNKNSearch(1);
    //             std::vector<float> pointNKNSquaredDistance(1);
    //             if (kdtree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0) {
    //                 nearest_path_height = path_points->points[pointIdxNKNSearch[0]].z;
    //             }
    //
    //             float height_diff = height - nearest_path_height;
    //             cv::Vec3b color = getColorFromHeight(height_diff);
    //             cv::circle(image, cv::Point(x_index, img_height - y_index), 2, color, cv::FILLED);
    //         }
    //     }
    //
    //     // // 腐蚀操作，去除小白点噪声
    //     // cv::Mat eroded_image;
    //     // int erosion_size = 0; // 结构元素的大小
    //     // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT,
    //     //                                             cv::Size(2 * erosion_size + 1, 2 * erosion_size + 1),
    //     //                                             cv::Point(erosion_size, erosion_size));
    //     // cv::erode(image, eroded_image, element);
    //
    //     // // 膨胀操作，使边缘更加突出
    //     cv::Mat dilated_image;
    //     // int dilation_size = 0; // 结构元素的大小，可以根据需要调整
    //     // element = cv::getStructuringElement(cv::MORPH_RECT,
    //     //                                     cv::Size(2 * dilation_size + 1, 2 * dilation_size + 1),
    //     //                                     cv::Point(dilation_size, dilation_size));
    //     // cv::dilate(eroded_image, dilated_image, element);
    //     dilated_image = image.clone();
    //
    //     // 边缘检测
    //     cv::Mat blurred_image;
    //     cv::GaussianBlur(dilated_image, blurred_image, cv::Size(5, 5), 0);
    //     cv::Mat edges;
    //     cv::Canny(blurred_image, edges, 100, 200);
    //
    //     // 将边缘绘制到原始图像上，边缘为黑色
    //     edges.convertTo(edges, CV_8UC1); // 确保边缘图像是单通道的
    //     cv::Mat edges_inv;
    //     cv::bitwise_not(edges, edges_inv); // 反转边缘图像，使边缘为白色（255）
    //
    //     // 创建一个与原图同样大小的掩码
    //     cv::Mat mask = cv::Mat::zeros(dilated_image.size(), CV_8UC3);
    //     edges_inv.convertTo(mask, CV_8UC3, 0, 255); // 将边缘图像转换为三通道，并将边缘设置为黑色（因为背景是0，边缘是255）
    //     // 但由于我们只需要黑色边缘，我们可以直接操作单通道然后扩展到三通道
    //     cv::Mat mask_single_channel;
    //     cv::cvtColor(edges_inv, mask_single_channel, cv::COLOR_GRAY2BGR); // 直接转换为三通道，边缘为白色（255,255,255）
    //     mask_single_channel.setTo(cv::Scalar(0, 0, 0), edges_inv); // 将白色边缘设置为黑色（0,0,0）
    //
    //     // 但上面的两行实际上可以简化为直接在原图上设置边缘为黑色
    //     for (int y = 0; y < edges.rows; ++y) {
    //         for (int x = 0; x < edges.cols; ++x) {
    //             if (edges.at<uchar>(y, x) != 0) {
    //                 // 如果边缘存在
    //                 dilated_image.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 0, 0); // 将边缘设置为黑色
    //             }
    //         }
    //     }
    //
    //
    //     // 从里程计数据中获取了yaw（弧度）和位置（x, y）
    //     float yaw = atan2(2 * (odomAftMapped.pose.pose.orientation.w * odomAftMapped.pose.pose.orientation.z),
    //                       1 - 2 * odomAftMapped.pose.pose.orientation.z * odomAftMapped.pose.pose.orientation.z);
    //     /* 从四元数或其他方式获取的yaw角（弧度） */
    //     ;
    //     float x = odomAftMapped.pose.pose.position.x;
    //     float y = odomAftMapped.pose.pose.position.y;
    //
    //     // 将位姿点投影到图像坐标
    //     int img_x = static_cast<int>((x - min_x) * d);
    //     int img_y = static_cast<int>((y - min_y) * d);
    //     img_y = img_height - img_y; // 转换y坐标以匹配OpenCV的图像坐标系统
    //
    //     // 创建一个与dilated_image相同大小的Mat用于临时存储（如果dilated_image不是640x480）
    //     cv::Mat temp_image = dilated_image.clone();
    //
    //     // 设置箭头的大小和颜色
    //     int arrow_size = 5; // 箭头三角形边长的一半（在放大前的大小）
    //     cv::Vec3b arrow_color(0, 255, 0); // 绿色箭头
    //
    //     // 计算箭头的顶点（注意：这里的arrow_size在放大后会相应变大，如果需要保持箭头大小不变，则需要在放大后重新计算arrow_size）
    //     float arrow_angle = yaw - CV_PI; // 调整方向以匹配图像坐标系（可能需要根据实际情况调整）
    //     float arrow_width = 1;
    //     cv::Point arrow_base(img_x, img_y);
    //     cv::Point arrow_tip1(img_x + cvRound(arrow_size * std::cos(arrow_angle)),
    //                          img_y - cvRound(arrow_size * std::sin(arrow_angle)));
    //     cv::Point arrow_tip2(img_x + cvRound(arrow_size * std::cos(arrow_angle + CV_PI / 10)),
    //                          img_y - cvRound(arrow_size * std::sin(arrow_angle + CV_PI / 10)));
    //     cv::Point arrow_tip3(img_x + cvRound(arrow_size * std::cos(arrow_angle - CV_PI / 10)),
    //                          img_y - cvRound(arrow_size * std::sin(arrow_angle - CV_PI / 10)));
    //
    //     // 构造一个三角形数组用于填充
    //     std::vector<cv::Point> arrow_triangle = {arrow_base, arrow_tip2, arrow_tip3};
    //     // 绘制并填充三角形箭头
    //     cv::fillConvexPoly(temp_image, arrow_triangle, arrow_color);
    //
    //     // 创建一个640x480的Mat用于显示和保存带有箭头的图像
    //     cv::Mat display_image;
    //     cv::resize(temp_image, display_image, cv::Size(640, 480));
    //
    //     // 显示带有箭头的图像
    //     cv::imshow("Map with Arrow", display_image);
    //     cv::waitKey(1);
    //
    //     // 保存带有箭头的图像
    //     std::string output_dir = root_dir + "/PCD/";
    //     std::string output_file = output_dir + "output_image_with_arrow.jpg";
    //     cv::imwrite(output_file, dilated_image);
    //     std::ofstream file(output_dir + "transData.txt", std::ios::trunc);
    //     file << min_x << "  " << min_y << "  " << d << " " << img_height << " " << img_width << std::endl;
    //     file.close();
    // }


    void run() {
        rclcpp::Rate rate(5000);

        while (rclcpp::ok()) {
            rclcpp::spin_some(shared_from_this());

            if (pose_scan_pair_buf_.empty()) continue;

            double timestamp = pose_scan_pair_buf_.front().first;
            auto key_odom_ = *pose_scan_pair_buf_.front().second.first;
            auto key_frame_ = *pose_scan_pair_buf_.front().second.second;
            pose_scan_pair_buf_.pop_front();
            frame_num++;

            // Process PointCloud
            PointCloudXYZI::Ptr cloud(new PointCloudXYZI);
            pcl::fromROSMsg(key_frame_, *cloud);
            *global_map += *cloud;

            key_odom_.header.frame_id = "map";
            key_odom_.header.stamp = this->now();
            key_frame_.header.frame_id = "map";
            key_frame_.header.stamp = this->now();

            pub_keypose_->publish(key_odom_);
            pub_keyframe_->publish(key_frame_);

            // Publish TF
            geometry_msgs::msg::TransformStamped transform_stamped;
            transform_stamped.header.stamp = this->now();
            transform_stamped.header.frame_id = "world";
            transform_stamped.child_frame_id = "sensor";
            // 设置平移
            transform_stamped.transform.translation.x = key_odom_.pose.pose.position.x;
            transform_stamped.transform.translation.y = key_odom_.pose.pose.position.y;
            transform_stamped.transform.translation.z = key_odom_.pose.pose.position.z;

            // 设置旋转
            transform_stamped.transform.rotation.w = key_odom_.pose.pose.orientation.w;
            transform_stamped.transform.rotation.x = key_odom_.pose.pose.orientation.x;
            transform_stamped.transform.rotation.y = key_odom_.pose.pose.orientation.y;
            transform_stamped.transform.rotation.z = key_odom_.pose.pose.orientation.z;


            tf_broadcaster_->sendTransform(transform_stamped);

            rate.sleep();
        }

        // Apply Voxel Grid Filter
        // down_size_filter.setInputCloud(global_map);
        // down_size_filter.filter(*global_map);
        // pcl::io::savePCDFileBinaryCompressed("/tmp/global_map.pcd", *global_map);

    }

    PointCloudXYZI::Ptr getPointCloud() {
        return std::make_shared<PointCloudXYZI>(*global_map);
    }

private:
    using syncPolicy = message_filters::TimeSynchronizer<nav_msgs::msg::Odometry, sensor_msgs::msg::PointCloud2>;

    std::mutex mtx;
    int frame_num = 0;
    bool flg_exit = false;

    PointCloudXYZI::Ptr global_map = std::make_shared<PointCloudXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr path_points = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    pcl::VoxelGrid<PointType> down_size_filter;
    double filter_size;

    std::deque<std::pair<double, std::pair<nav_msgs::msg::Odometry::SharedPtr,
        sensor_msgs::msg::PointCloud2::SharedPtr> > > pose_scan_pair_buf_;

    std::shared_ptr<message_filters::Subscriber<nav_msgs::msg::Odometry> > sub_odometry_;
    std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::PointCloud2> > sub_laser_cloud_;
    std::shared_ptr<syncPolicy> sync_;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_keyframe_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_keypose_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    auto kp = std::make_shared<KeyframePublisher>();
    // std::thread KPThread(&KeyframePublisher::run, kp);
    kp->run();

    rclcpp::shutdown();


    // std::cout << "Shutdown ||||||||||" << std::endl;
    // auto pcd = kp->getPointCloud();
    // pcl::io::savePCDFileBinaryCompressed("/tmp/global_map.pcd", *pcd);

    // KPThread.join();

    return 0;
}