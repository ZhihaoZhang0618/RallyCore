/******************************************************************************
 * @file        lidarMapping.h
 * @brief       Declaration of the lio module class and related functions.
 *
 * @author      Yunxiang He
 * @date        2024-11-27
 * @version     2.0.0
 * @copyright   WESTLAKE ROBOTICS TECHNOLOGY (HANGZHOU) CO.,LTD
 *
 * @details     This file contains the definition and assignment of the lidar-inertial-odometry in the slam module,
 *              and initialize the global pose.
 *
 * @license     MIT License
 ******************************************************************************/
#pragma once
#ifndef LIDARMAPPING_H
#define LIDARMAPPING_H

#include <arps/utility.h>
#include <arps/imuProcessing.h>
#include <arps/preProcess.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <algorithm>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <ikd-Tree/ikd_Tree.h>
#include <boost/filesystem.hpp>
#include <boost/bind.hpp>

namespace arps {
    namespace slam {
        namespace arps_lidar_inertial_odometry {
            class LidarMapping : public rclcpp::Node {
            public:
                rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr livox_sub_;
                rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
                rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

                rclcpp::CallbackGroup::SharedPtr callbackGroupLidar;
                rclcpp::CallbackGroup::SharedPtr callbackGroupImu;
                rclcpp::TimerBase::SharedPtr process_timer_;

                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudFullRes_body;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudEffect;
                rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubLaserCloudMap;
                rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pubSyncImu;
                rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pubOdomAftMapped;
                rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath;

                Eigen::IOFormat CleanFmt;
                std::mutex lidar_mtx;
                std::mutex imu_mtx;
                std::condition_variable sig_buffer;

                // Time Log Variables
                const float MOV_THRESHOLD = 1.5f;
                const float INIT_TIME = 0.5;
                int kdtree_size_st = 0, kdtree_size_end = 0, add_point_size = 0;
                double cube_len = 0, total_distance = 0, lidar_end_time = 0, first_lidar_time = 0.0;
                double last_timestamp_lidar = 0;
                double last_timestamp_imu = 0.0;
                bool lidar_pushed = false, flg_reset = false, flg_exit = false, flg_EKF_inited = true;
                int iterCount = 0, feats_down_size = 0, effect_feat_num = 0;
                double res_mean_last = 0.05;
                int points_cache_size = 0;
                std::vector<BoxPointType> cub_needrm;
                std::deque<PointCloudXYZI::Ptr> lidar_buffer;
                std::deque<double> time_buffer;
                std::deque<sensor_msgs::msg::Imu::SharedPtr> imu_buffer;
                std::vector<std::vector<int> > pointSearchInd_surf;
                std::vector<PointVector> Nearest_Points;
                bool point_selected_surf[100000] = {0};
                float res_last[100000] = {0.0};
                double total_residual;

                //surf feature in map
                PointCloudXYZI::Ptr featsFromMap;
                PointCloudXYZI::Ptr feats_undistort;
                PointCloudXYZI::Ptr feats_down_body;
                PointCloudXYZI::Ptr feats_down_world;
                PointCloudXYZI::Ptr normvec;
                PointCloudXYZI::Ptr laserCloudOri;
                PointCloudXYZI::Ptr corr_normvect;
                PointCloudXYZI::Ptr _featsArray;

                pcl::VoxelGrid<PointType> downSizeFilterSurf;
                pcl::VoxelGrid<PointType> downSizeFilterMap;

                KD_TREE ikdtree;

                M3D last_rot;
                V3F XAxisPoint_body;
                V3F XAxisPoint_world;
                V3D euler_cur;
                V3D position_last;
                V3D last_odom;

                V3D Lidar_T_wrt_IMU;
                M3D Lidar_R_wrt_IMU;
                M3D ExtrinRPY;


                /*** variables definition ***/
                VD(DIM_STATE) solution;
                MD(DIM_STATE, DIM_STATE) G, H_T_H, I_STATE;
                V3D rot_add, T_add, vel_add, gyr_add;

                StatesGroup state_propagat;
                PointType pointOri, pointSel, coeff;

                double deltaT, deltaR;
                bool flg_EKF_converged, EKF_stop_flg = 0;

                //estimator inputs and output;
                MeasureGroup Measures;
                StatesGroup state;

                PointCloudXYZI::Ptr pcl_wait_save;
                pcl::PCDWriter pcd_writer;
                string all_points_dir;

                nav_msgs::msg::Path path;
                nav_msgs::msg::Odometry odomAftMapped;
                geometry_msgs::msg::Quaternion geoQuat;
                geometry_msgs::msg::PoseStamped msg_body_pose;
                // sensor_msgs::msg::Imu IMU_sync;

                BoxPointType LocalMap_Points;
                bool Localmap_Initialized = false;

                double timediff_imu_wrt_lidar = 0.0;
                bool timediff_set_flg = false;

                std::shared_ptr<Preprocess> p_pre;
                std::shared_ptr<ImuProcess> p_imu;

                tf2::Quaternion quat;
                std::unique_ptr<tf2_ros::TransformBroadcaster> br;

                std::string config_file_path_;

                LidarMapping(const rclcpp::NodeOptions &node_options);

                void livox_pcl_cbk(const livox_ros_driver2::msg::CustomMsg::ConstSharedPtr msg);
                void standard_pcl_cbk(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
                void imu_cbk(const sensor_msgs::msg::Imu::ConstSharedPtr msg_in);


                void run();

            private:
                void getParameters(std::string &config_file_path);

                void initialization();

                void calcBodyVar(Eigen::Vector3d &pb, const float range_inc, const float degree_inc,
                                 Eigen::Matrix3d &var);

                Eigen::Matrix3d eulerToRotationMatrix(double roll, double pitch, double yaw);

                void pointBodyToWorld(PointType const *const pi, PointType *const po);

                void points_cache_collect();

                template<typename T>
                void pointBodyToWorld(const Eigen::Matrix<T, 3, 1> &pi, Eigen::Matrix<T, 3, 1> &po);

                void RGBpointBodyToWorld(PointType const *const pi, PointTypeRGB *const po);

                void lasermap_fov_segment();

                bool sync_packages(MeasureGroup &meas);

                void map_incremental();

                void publish_frame_world(
                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes);

                void publish_frame_body(
                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudFullRes_body);

                void publish_effect_world(
                    const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudEffect);

                void publish_map(const rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr &pubLaserCloudMap);

                template<typename T>
                void set_posestamp(T &out);

                void publish_odometry(const rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr &pubOdomAftMapped);

                void publish_path(const rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pubPath);


                // parameters
                std::string lid_topic, imu_topic;
                double laser_point_cov;
                int point_filter_num, max_iteration;
                double cube_side_length;

                int lidar_type;
                bool feature_extract_en;
                int scan_line;
                double blind;
                int timestamp_unit;

                int cut_frame_num;
                int orig_odom_freq;
                double mean_acc_norm;
                int max_init_count;
                double online_refine_time;

                double filter_size_surf, filter_size_map;
                bool imu_en;
                int imu_int_frame;
                double gyr_cov = 0.1;
                double acc_cov = 0.1;
                double b_gyr_cov = 0.0001;
                double b_acc_cov = 0.0001;
                float DET_RANGE = 300.0f;

                std::vector<double> extrinT;
                std::vector<double> extrinR;
                std::vector<double> extrinRPY;

                bool path_en;
                bool scan_pub_en;
                bool dense_pub_en;
                bool scan_body_pub_en;

                bool pcd_save_en;
                int interval;
            };
        }
    }
}

#endif //LIDARMAPPING_H
