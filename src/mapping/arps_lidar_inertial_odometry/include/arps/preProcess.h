/******************************************************************************
 * @file        perProcess.h
 * @brief       Declaration of the preprocess class and related functions.
 *
 * @author      Yunxiang He
 * @date        2024-11-26
 * @version     2.0.0
 * @copyright   WESTLAKE ROBOTICS TECHNOLOGY (HANGZHOU) CO.,LTD
 *
 * @details     This file contains the definition and assignment of the pre-process of point cloud in the slam module,
 *              and convert the msg to pcl-point.
 *
 * @license     MIT License
 ******************************************************************************/

#pragma once
#ifndef PREPROCESS_H
#define PREPROCESS_H

#include <rclcpp/rclcpp.hpp>
#include <arps/utility.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>

#define IS_VALID(a) ( ( abs( a ) > 1e8 ) ? true : false )

/*** Velodyne ***/

struct EIGEN_ALIGN16 VelodynePoint {
    PCL_ADD_POINT4D;
    float intensity;
    float time;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (float , time, time)(uint16_t, ring, ring))

/****************/

/*** Ouster ***/

struct EIGEN_ALIGN16 LivoxPoint {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t tag;
    uint8_t line;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(LivoxPoint,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
                                  (uint8_t , tag, tag)(uint8_t, line, line)(double, timestamp, timestamp)
                                 )

/****************/

/*** Hesai_XT32 ***/
struct EIGEN_ALIGN16 XT32Point {
    PCL_ADD_POINT4D;
    float intensity;
    double timestamp;
    uint16_t ring;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(XT32Point,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(
                                      double, timestamp, timestamp)( uint16_t, ring, ring))

/*****************/

namespace arps {
    namespace slam {
        namespace arps_lidar_inertial_odometry {
            // enum LID_TYPE{AVIA = 1, VELO16, OUST64, L515}; //{1, 2, 3}
            enum Feature {
                Nor,
                Poss_Plane,
                Real_Plane,
                Edge_Jump,
                Edge_Plane,
                Wire,
                ZeroPoint
            };

            enum Surround {
                Prev,
                Next
            };

            enum E_jump {
                Nr_nor,
                Nr_zero,
                Nr_180,
                Nr_inf,
                Nr_blind
            };

            struct orgtype {
                double range;
                double dista;
                double angle[2];
                double intersect;
                E_jump edj[2];
                Feature ftype;

                orgtype() {
                    range = 0;
                    edj[Prev] = Nr_nor;
                    edj[Next] = Nr_nor;
                    ftype = Nor;
                    intersect = 2;
                }
            };

            class Preprocess {
            public:
                //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

                Preprocess();

                ~Preprocess();

                void process(const livox_ros_driver2::msg::CustomMsg::ConstPtr &msg, PointCloudXYZI::Ptr &pcl_out);

                void process(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg, PointCloudXYZI::Ptr &pcl_out);

                void set(bool feat_en, int lid_type, double bld, int pfilt_num);

                // sensor_msgs::msg::PointCloud2::ConstSharedPtr pointcloud;
                PointCloudXYZI pl_full, pl_corn, pl_surf;
                PointCloudXYZI pl_buff[128]; // maximum 128 line lidar
                std::vector<orgtype> typess[128]; // maximum 128 line lidar
                int lidar_type, point_filter_num, N_SCANS, time_unit;
                double blind, blind_sqr, blind_bak_rate;
                double time_unit_scale;
                bool feature_enabled, given_offset_time, calib_laser;
                // ros::Publisher pub_full, pub_surf, pub_corn;

            private:
                void avia_handler(const livox_ros_driver2::msg::CustomMsg::ConstPtr &msg);

                void livox_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

                void velodyne_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

                // void velodyne32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

                void xt32_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

                // void l515_handler(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

                void give_feature(PointCloudXYZI &pl, std::vector<orgtype> &types);

                void pub_func(PointCloudXYZI &pl, const rclcpp::Time &ct);

                int
                plane_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, uint &i_nex,
                            Eigen::Vector3d &curr_direct);

                bool small_plane(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i_cur, uint &i_nex,
                                 Eigen::Vector3d &curr_direct);

                bool edge_jump_judge(const PointCloudXYZI &pl, std::vector<orgtype> &types, uint i, Surround nor_dir);

                int group_size;
                double disA, disB, inf_bound;
                double limit_maxmid, limit_midmin, limit_maxmin;
                double p2l_ratio;
                double jump_up_limit, jump_down_limit;
                double cos160;
                double edgea, edgeb;
                double smallp_intersect, smallp_ratio;
                double vx, vy, vz;
            };
        }
    }
}

#endif //PREPROCESS_H
